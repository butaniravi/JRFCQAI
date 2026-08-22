///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//In this file the timers for reading the receiver pulses and for creating the output ESC pulses are set.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//More information can be found in these two videos:
//STM32 for Arduino - Connecting an RC receiver via input capture mode: https://youtu.be/JFSFbSg0l2M
//STM32 for Arduino - Electronic Speed Controller (ESC) - STM32F103C8T6: https://youtu.be/Nju9rvZOjVQ
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool read_ibus_pkt() {
  static uint8_t ibusBuffer[32];
  static uint8_t bufferIndex = 0;
  
  while (Serial1.available() > 0) {
    uint8_t incomingByte = Serial1.read();
    
    if (bufferIndex >= 32) bufferIndex = 0;
    if (bufferIndex == 0 && incomingByte != 0x20) continue; 
    if (bufferIndex == 1 && incomingByte != 0x40) { bufferIndex = 0; continue; }
    
    ibusBuffer[bufferIndex++] = incomingByte;
    
    if (bufferIndex == 32) {
      bufferIndex = 0; 
      
      uint16_t calculatedChecksum = 0xFFFF;
      for (uint8_t i = 0; i < 30; i++) {
        calculatedChecksum -= ibusBuffer[i];
      }
      
      uint16_t transmittedChecksum = ibusBuffer[30] | (ibusBuffer[31] << 8);
      
      if (calculatedChecksum == transmittedChecksum) {
        // Map the digital iBus stream straight into the original global variables
        channel_1 = (int32_t)(ibusBuffer[2]  | (ibusBuffer[3]  << 8));
        channel_2 = (int32_t)(ibusBuffer[4]  | (ibusBuffer[5]  << 8));
        channel_3 = (int32_t)(ibusBuffer[6]  | (ibusBuffer[7]  << 8));
        channel_4 = (int32_t)(ibusBuffer[8]  | (ibusBuffer[9]  << 8));
        channel_5 = (int32_t)(ibusBuffer[10] | (ibusBuffer[11] << 8));
        channel_6 = (int32_t)(ibusBuffer[12] | (ibusBuffer[13] << 8));
		    channel_7 = (int32_t)(ibusBuffer[14] | (ibusBuffer[15] << 8));
		    channel_8 = (int32_t)(ibusBuffer[16] | (ibusBuffer[17] << 8));
        channel_9 = (int32_t)(ibusBuffer[18] | (ibusBuffer[19] << 8));
        channel_10 = (int32_t)(ibusBuffer[20] | (ibusBuffer[21] << 8));
        return true; 
      }
    }
  }
  return false;
}

void ibus_setup(void) {
  Serial1.begin(115200);
  
  analogWrite(M1_ESC,1000);                              //Set the throttle receiver input pulse to the ESC 1 output pulse.
  analogWrite(M2_ESC,1000);                              //Set the throttle receiver input pulse to the ESC 2 output pulse.
  analogWrite(M3_ESC,1000);                              //Set the throttle receiver input pulse to the ESC 3 output pulse.
  analogWrite(M4_ESC,1000);
  delay(50);
}

