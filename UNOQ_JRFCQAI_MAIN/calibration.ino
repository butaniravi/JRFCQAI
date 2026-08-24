/**
 * Calibrate compass
 */
void calibrate_compass(void) {
  /* Set the compass_calibration_on variable to disable the adjustment of the raw compass values */
  compass_calibration_on = 1;                                                //Set the compass_calibration_on variable to disable the adjustment of the raw compass values.
  /* The red LED will indicate that compass calibration is active */ 
  red_led(HIGH);                                                             //The red led will indicate that the compass calibration is active.
  /* Turn off green LED */
  green_led(LOW);                                                            //Turn off the green led as we don't need it.
  /* Stay in this loop until transmitter pitch stick is pulled down */
  while (channel_2 < 1900) {                                                 //Stay in this loop until the pilot lowers the pitch stick of the transmitter.
    /* Send telemetry data to ground station */
    read_ibus_pkt();
    send_telemetry_data();                                                   //Send telemetry data to the ground station.
    /* Simulate a 250Hz program loop */
    delayMicroseconds(3200);                                                 //Simulate a 250Hz program loop.
    /* Read raw compass values */ 
    read_compass();                                                          //Read the raw compass values.
    /* Detect and store maximum and minimum compass values */
    //In the following lines the maximum and minimum compass values are detected and stored.
    if (compass_x < compass_cal_values[0])compass_cal_values[0] = compass_x;
    if (compass_x > compass_cal_values[1])compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2])compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3])compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4])compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5])compass_cal_values[5] = compass_z;
  }
  /* Reset compass calibration variable */
  compass_calibration_on = 0;

  /* Store electronic compass calibration values */
  //The maximum and minimum values are needed for the next startup and are stored
  
  for (error = 0; error < 6; error ++) {
    EEPROM_Write(0x10 + error, compass_cal_values[error]);
    delay(5);
  }
  
  /* Initialize compass and set correct registers */
  setup_compass();
  /* Read and calculate compass data */
  read_compass();
  /* Set initial compass heading */
  angle_yaw = actual_compass_heading;

  red_led(LOW);
  for (error = 0; error < 15; error ++) {
    green_led(HIGH);
    delay(50);
    green_led(LOW);
    delay(50);
  }

  error = 0;
  /* Set timer for next loop */
  loop_timer = micros();
}

/**
 * Calibrate level
 */
void calibrate_level(void) {
  level_calibration_on = 1;

  while (channel_2 < 1100) {
    /* Send telemetry data to ground station */
    read_ibus_pkt();
    send_telemetry_data();
    delay(10);
  }
  red_led(HIGH);
  green_led(LOW);

  acc_pitch_cal_value = 0;
  acc_roll_cal_value = 0;

  for (error = 0; error < 64; error ++) {
    /* Send telemetry data to ground station */
    send_telemetry_data();
    gyro_signalen();
    acc_pitch_cal_value += acc_y;
    acc_roll_cal_value += acc_x;
    if (acc_y > 500 || acc_y < -500)error = 80;
    if (acc_x > 500 || acc_x < -500)error = 80;
    delayMicroseconds(3700);
  }

  acc_pitch_cal_value /= 64;
  acc_roll_cal_value /= 64;

  red_led(LOW);
  if (error < 80) {
    EEPROM_Write(0x16, acc_pitch_cal_value);
    delay(5);
    EEPROM_Write(0x17, acc_roll_cal_value);
    delay(5);
    
    
    for (error = 0; error < 15; error ++) {
      green_led(HIGH);
      delay(50);
      green_led(LOW);
      delay(50);
    }
    error = 0;
  }
  else error = 3;
  level_calibration_on = 0;
  gyro_signalen();
  /** 
   *  Accelerometer angle calculation
   *  Calculate total accelerometer vector
   */
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  /**
   * Prevent asin function from producing NaN
   * Calculate pitch angle
   */
  if (abs(acc_y) < acc_total_vector) {
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }
  /**
   * Prevent asin function from producing NaN
   * Calculate roll angle
   */
  if (abs(acc_x) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;
  }
  /* Set gyro pitch angle to accelerometer pitch angle at quadcopter startup */
  angle_pitch = angle_pitch_acc;
  angle_roll = angle_roll_acc;
  /* Set timer for next loop */
  loop_timer = micros();
}