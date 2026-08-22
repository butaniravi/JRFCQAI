void read_compass() 
{
  // Send compass address
  Wire1.beginTransmission(compass_address);
  // Send the address of the compass to read from
  Wire1.write(0x00);
  // Release the bus
  Wire1.endTransmission();

  // Request 6 bytes from the compass
  Wire1.requestFrom(compass_address, 6);

  /* Magnetometer installed in forward direction */
  #ifdef COMPASS_INSTALL_DIRECTION
  // Add low byte and high byte to compass_x variable
  compass_x = Wire1.read() | Wire1.read() << 8;
  // Add low byte and high byte to compass_y variable
  compass_y = Wire1.read() | Wire1.read() << 8;
  // Add low byte and high byte to compass_z variable
  compass_z = Wire1.read() | Wire1.read() << 8;
  compass_z *= -1;
  #else
  /* Magnetometer installed in reverse direction */
  compass_x = Wire1.read() | Wire1.read() << 8;
  // Invert axis direction
  compass_x *= -1;
  // Add low byte and high byte to compass_y variable
  compass_y = Wire1.read() | Wire1.read() << 8;
  // Invert axis direction
  compass_y *= -1;
  // Add low byte and high byte to compass_z variable
  compass_z = Wire1.read() | Wire1.read() << 8;
  #endif

  /**
   * Before the compass can give accurate measurements, it needs to be calibrated.
   * At startup, the compass offset and scale variables are calculated.
   * The following section adjusts the raw compass values so they can be used for heading calculation.
   */
  if (compass_calibration_on == 0) {  // When the compass is not being calibrated
    compass_y += compass_offset_y;    // Add y-offset to the raw value
    compass_y *= compass_scale_y;     // Scale y-value to match the other axes
    compass_z += compass_offset_z;    // Add z-offset to the raw value
    compass_z *= compass_scale_z;     // Scale z-value to match the other axes
    compass_x += compass_offset_x;    // Add x-offset to the raw value
  }

  /**
   * When the roll and pitch angles of the quadcopter change, the compass values change as well.
   * This is why the x and y values need to be calculated for a virtual horizontal position.
   * The value 0.0174533 is pi/180, as the trigonometric functions use radians instead of degrees.
   */
  compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
  compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

  /**
   * Now that horizontal values are known, the heading can be calculated in degrees.
   * Note that atan2 uses radians instead of degrees, which is why 180/3.14 is used.
   */
  if (compass_y_horizontal < 0)actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
  else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

  actual_compass_heading += declination;                                 // Add magnetic declination to heading to get geographic North
  if (actual_compass_heading < 0) actual_compass_heading += 360;         // Keep heading within 0 to 360 degree range
  else if (actual_compass_heading >= 360) actual_compass_heading -= 360; // Keep heading within 0 to 360 degree range
}

/**
 * At startup, the registers of the compass need to be set.
 * Then the calibration offset and scale values are calculated.
 */
void setup_compass() 
{
  Wire1.beginTransmission(compass_address); // Configure control register
  Wire1.write(0x09);
  Wire1.write(0x1d);
  Wire1.endTransmission();

  Wire1.beginTransmission(compass_address); // Set reset period register
  Wire1.write(0x0b);
  Wire1.write(0x01);
  Wire1.endTransmission();
  Wire1.beginTransmission(compass_address);
  Wire1.write(0x20);
  Wire1.write(0x40);
  Wire1.endTransmission();
  Wire1.beginTransmission(compass_address);
  Wire1.write(0x21);
  Wire1.write(0x01);
  Wire1.endTransmission();

  // Read calibration values from EEPROM
  for (error = 0; error < 6; error ++) {
    compass_cal_values[error] = EEPROM_Read(0x10 + error);
  }
    
    
  error = 0;
  // Calculate calibration offset and scale values
  compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
  compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

  compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
  compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
  compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
}

/**
 * The following subroutine calculates the smallest difference between two heading values.
 */
float course_deviation(float course_b, float course_c) 
{
  course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    if (course_c > 180)base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180)actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}