ament_add_gtest(estimate_track_error_test estimate_track_error_test.cpp)
target_link_libraries(estimate_track_error_test
  ${gps_to_nav}
)
