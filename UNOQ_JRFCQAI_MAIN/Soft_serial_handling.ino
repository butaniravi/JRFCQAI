// FIXED: Using a reference to a byte vector matches the MsgPack signature perfectly!
void stm_rx_tel(std::vector<uint8_t> si_received_bytes) {
    // std::vector natively uses .size() 
    if (si_received_bytes.size() >= 12) {
      // Access array elements directly using list indexing
      si_check_byte = 0x00;
      
      // Calculate XOR checksum over the first 11 bytes
      for (count_var = 0; count_var <= 10; count_var++) {
        si_check_byte ^= si_received_bytes[count_var];
      }

      // Verify checksum against received checksum byte (index 11)
      if (si_check_byte == si_received_bytes[11]) {
        // Check for 'W' 'P' header signatures
        if (si_received_bytes[0] == 'W' && si_received_bytes[1] == 'P') {
          new_waypoint_available = 1;
          //si_received_bytes[0] = 0x00; // Clear signature byte

          // Extract 32-bit integer Latitude (Little-Endian)
          wp_lat_gps = (int32_t)si_received_bytes[2] | 
                       ((int32_t)si_received_bytes[3] << 8) | 
                       ((int32_t)si_received_bytes[4] << 16) | 
                       ((int32_t)si_received_bytes[5] << 24);

          // Extract 32-bit integer Longitude (Little-Endian)
          wp_lon_gps = (int32_t)si_received_bytes[6] | 
                       ((int32_t)si_received_bytes[7] << 8) | 
                       ((int32_t)si_received_bytes[8] << 16) | 
                       ((int32_t)si_received_bytes[9] << 24);

          // Trigger waypoint navigation if in autonomous/hold mode
          if (waypoint_set == 1 && home_point_recorded == 1 && flight_mode == 3) {
            fly_to_new_waypoint = 1;
            fly_to_new_waypoint_step = 0;
            fly_to_waypoint_lat_factor = 0;
            fly_to_waypoint_lon_factor = 0;
          }
        }
      }
      
    }
}

void si_translate_bytes(void) {
  // Obsolete: bridge provide callback handles received packet processing
}

void Serial_input_handler(void) {
  // Obsolete: bridge handle this automatically
}

