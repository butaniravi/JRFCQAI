///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this part the various registers of the MPU-6050 are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_setup(void) {
  Wire1.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire1.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex).
  Wire1.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro.
  Wire1.endTransmission();                                      //End the transmission with the gyro.

  Wire1.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire1.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex).
  Wire1.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale).
  Wire1.endTransmission();                                      //End the transmission with the gyro.

  Wire1.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire1.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex).
  Wire1.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range).
  Wire1.endTransmission();                                      //End the transmission with the gyro.

  Wire1.beginTransmission(gyro_address);                        //Start communication with the MPU-6050.
  Wire1.write(0x1A);                                            //We want to write to the CONFIG register (1A hex).
  Wire1.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).
  Wire1.endTransmission();                                      //End the transmission with the gyro.

  acc_pitch_cal_value  = EEPROM_Read(0x16);
  acc_roll_cal_value  = EEPROM_Read(0x17);
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This subroutine handles the calibration of the gyro. It stores the avarage gyro offset of 2000 readings.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calibrate_gyro(void) {
  cal_int = 0;                                                                        //Set the cal_int variable to zero.
  if (cal_int != 2000) {
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++) {                                  //Take 2000 readings for calibration.
      if (cal_int % 25 == 0) digitalWrite(LED_R, !digitalRead(LED_R));                    //Change the led status every 125 readings to indicate calibration.
      gyro_signalen();                                                                //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                                     //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                                                   //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                                       //Ad yaw value to gyro_yaw_cal.
      delay(4);                                                                       //Small delay to simulate a 250Hz loop during calibration.
    }
    red_led(HIGH);                                                                     //Set output PB3 low.
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                                            //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                                           //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                                             //Divide the yaw total by 2000.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This part reads the raw gyro and accelerometer data from the MPU-6050
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(void) {
  Wire1.beginTransmission(gyro_address);                       //Start communication with the gyro.
  Wire1.write(0x3B);                                           //Start reading @ register 43h and auto increment with every read.
  Wire1.endTransmission();                                     //End the transmission.
  Wire1.requestFrom(gyro_address, 14);                         //Request 14 bytes from the MPU 6050.
  acc_y = Wire1.read() << 8 | Wire1.read();                    //Add the low and high byte to the acc_x variable.
  acc_x = Wire1.read() << 8 | Wire1.read();                    //Add the low and high byte to the acc_y variable.
  acc_z = Wire1.read() << 8 | Wire1.read();                    //Add the low and high byte to the acc_z variable.
  temperature = Wire1.read() << 8 | Wire1.read();              //Add the low and high byte to the temperature variable.
  gyro_roll = Wire1.read() << 8 | Wire1.read();                //Read high and low part of the angular data.
  gyro_pitch = Wire1.read() << 8 | Wire1.read();               //Read high and low part of the angular data.
  gyro_yaw = Wire1.read() << 8 | Wire1.read();                 //Read high and low part of the angular data.
  gyro_pitch *= -1;                                            //Invert the direction of the axis.
  gyro_yaw *= -1;                                              //Invert the direction of the axis.

  if (level_calibration_on == 0) {
    acc_y -= acc_pitch_cal_value;                              //Subtact the manual accelerometer pitch calibration value.
    acc_x -= acc_roll_cal_value;                               //Subtact the manual accelerometer roll calibration value.
  }
  if (cal_int >= 2000) {
    gyro_roll -= gyro_roll_cal;                                  //Subtact the manual gyro roll calibration value.
    gyro_pitch -= gyro_pitch_cal;                                //Subtact the manual gyro pitch calibration value.
    gyro_yaw -= gyro_yaw_cal;                                    //Subtact the manual gyro yaw calibration value.
  }
}
