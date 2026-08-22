///////////////////////////////////////////////////////////////////////////////////////////////////////////
//This is the fly waypoints step program
///////////////////////////////////////////////////////////////////////////////////////////////////////////
void fly_waypoints(void) {
  waypoint_monitor = 0;
  if (flight_mode != 3 && start == 2){
    fly_to_new_waypoint = 0;
    waypoint_monitor = 0;
  }

  if (fly_to_new_waypoint == 1) {
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step 0 - make some basic calculations
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (fly_to_new_waypoint_step == 0) {
      fly_to_waypoint_move_factor = 0.0;
      if (fly_to_waypoint_lat_factor == 1 || fly_to_waypoint_lon_factor == 1)fly_to_new_waypoint_step = 1;
      //cos(((float)l_lat_gps / 1000000.0)
      if (abs(wp_lat_gps - l_lat_waypoint) >= abs(wp_lon_gps - l_lon_waypoint)) {
        fly_to_waypoint_lon_factor = (float)abs(wp_lon_gps - l_lon_waypoint) / (float)abs(wp_lat_gps - l_lat_waypoint);
        fly_to_waypoint_lat_factor = 1;
      }
      else {
        fly_to_waypoint_lon_factor = 1;
        fly_to_waypoint_lat_factor = (float)abs(wp_lat_gps - l_lat_waypoint) / (float)abs(wp_lon_gps - l_lon_waypoint);
      }
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Step 2 - Fly to the next waypoint
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if (fly_to_new_waypoint_step == 1) {
      waypoint_monitor = 6;
      if (wp_lat_gps == l_lat_waypoint && wp_lon_gps == l_lon_waypoint)fly_to_new_waypoint = 0;

      if (abs(wp_lat_gps - l_lat_waypoint) < 160 && abs(wp_lon_gps - l_lon_waypoint) < 160 && fly_to_waypoint_move_factor > 0.05)fly_to_waypoint_move_factor -= 0.00015;
      else if (fly_to_waypoint_move_factor < 0.20)fly_to_waypoint_move_factor += 0.0001;

      if (wp_lat_gps != l_lat_waypoint) {
        if (wp_lat_gps > l_lat_waypoint) l_lat_gps_float_adjust += fly_to_waypoint_move_factor * fly_to_waypoint_lat_factor;
        if (wp_lat_gps < l_lat_waypoint) l_lat_gps_float_adjust -= fly_to_waypoint_move_factor * fly_to_waypoint_lat_factor;
      }
      if (wp_lon_gps != l_lon_waypoint) {
        if (wp_lon_gps > l_lon_waypoint) l_lon_gps_float_adjust += fly_to_waypoint_move_factor * fly_to_waypoint_lon_factor;
        if (wp_lon_gps < l_lon_waypoint) l_lon_gps_float_adjust -= fly_to_waypoint_move_factor * fly_to_waypoint_lon_factor;
      }
    }
  }
}
