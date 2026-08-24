void send_telemetry_data(void) {
  telemetry_loop_counter++;

  // Send the entire telemetry message on the 1st tick of the 125-loop cycle
  if (telemetry_loop_counter == 1) {
    uint8_t check_byte = 0;

    // Header
    packet[0] = 'J';
    packet[1] = 'B';
    
    // Status & Battery
    packet[2] = error;
    packet[3] = flight_mode + return_to_home_step + waypoint_monitor;
    packet[4] = battery_voltage * 10;

    // Temperature (16-bit)
    int16_t temp_val = temperature;
    packet[5] = temp_val & 0xFF;
    packet[6] = (temp_val >> 8) & 0xFF;

    // Attitude & Start
    packet[7] = angle_roll + 100;
    packet[8] = angle_pitch + 100;
    packet[9] = start;

    // Altitude (16-bit)
    int16_t alt_val;
    if (start == 2) {
      alt_val = 10000 + lroundf((ground_pressure - actual_pressure) * 10.0);
    } else {
      alt_val = 10000;
    }
    packet[10] = alt_val & 0xFF;
    packet[11] = (alt_val >> 8) & 0xFF;

    // Throttle (16-bit)
    int16_t throttle_val = 1500 + takeoff_throttle;
    packet[12] = throttle_val & 0xFF;
    packet[13] = (throttle_val >> 8) & 0xFF;

    // Yaw (16-bit)
    int16_t yaw_val = angle_yaw;
    packet[14] = yaw_val & 0xFF;
    packet[15] = (yaw_val >> 8) & 0xFF;

    // GPS Status
    packet[16] = heading_lock;
    packet[17] = number_used_sats;
    packet[18] = fix_type;

    // Latitude (32-bit)
    int32_t lat_val = l_lat_gps;
    //if(l_lat_waypoint != 0)lat_val = l_lat_waypoint;                                                      //Store the latitude position as it can change during the next loop.
    //else lat_val = l_lat_gps;
    packet[19] = lat_val & 0xFF;
    packet[20] = (lat_val >> 8) & 0xFF;
    packet[21] = (lat_val >> 16) & 0xFF;
    packet[22] = (lat_val >> 24) & 0xFF;

    // Longitude (32-bit)
    int32_t lon_val = l_lon_gps;
    //if(l_lon_waypoint != 0)lon_val = l_lon_waypoint;                                                      //Store the longitude position as it can change during the next loop.
    //else lon_val = l_lon_gps; 
    packet[23] = lon_val & 0xFF;
    packet[24] = (lon_val >> 8) & 0xFF;
    packet[25] = (lon_val >> 16) & 0xFF;
    packet[26] = (lon_val >> 24) & 0xFF;

    // Settings (16-bit each)
    int16_t s1 = adjustable_setting_1 * 100;
    packet[27] = channel_1 & 0xFF;
    packet[28] = (channel_1 >> 8) & 0xFF;

    int16_t s2 = adjustable_setting_2 * 100;
    packet[29] = channel_2 & 0xFF;
    packet[30] = (channel_2 >> 8) & 0xFF;

    int16_t s3 = adjustable_setting_3 * 100;
    packet[31] = channel_3 & 0xFF;
    packet[32] = (channel_3 >> 8) & 0xFF;

    int16_t s4 = adjustable_setting_3 * 100;
    packet[33] = channel_4 & 0xFF;
    packet[34] = (channel_4 >> 8) & 0xFF;

    int16_t s5 = adjustable_setting_3 * 100;
    packet[35] = channel_4 & 0xFF;
    packet[36] = (channel_4 >> 8) & 0xFF;

    int16_t s6 = adjustable_setting_3 * 100;
    packet[37] = channel_4 & 0xFF;
    packet[38] = (channel_4 >> 8) & 0xFF;

    // Calculate Checksum over bytes 0 through 32
    for (int i = 0; i < 39; i++) {
      check_byte ^= packet[i];
    }
    packet[39] = check_byte;

    // Send the entire 34-byte packet over Serial1
    // Serial1.write(packet, 34);
    // Fire-and-forget: Sends the std::array without template errors
    Bridge.notify("stm_tx_tel", packet);
  }

  // Reset the loop counter every 125 calls (preserves exact transmission interval)
  if (telemetry_loop_counter >= 60) {
    telemetry_loop_counter = 0;
  }
}