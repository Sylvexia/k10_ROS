target_link_libraries(test_node
test_lib
${OpenCV_LIBS}
${catkin_LIBRARIES}
)

target_link_libraries(webcam_node
webcam_lib
${OpenCV_LIBS}
${catkin_LIBRARIES}
)

target_link_libraries(cam_to_wheel_node
cam_to_wheel_lib
pid
${OpenCV_LIBS}
${catkin_LIBRARIES}
)

target_link_libraries(wheel_node
wheel_lib
${catkin_LIBRARIES}
)

# target_link_libraries(track_node
#    track_lib
#    graph_lib
#    sense_data_lib
#    act_data_lib
#    pid
#    ${OpenCV_LIBS}
#    ${catkin_LIBRARIES}
# )

# target_link_libraries(final_move_node
#   final_move_lib
#   act_data_lib
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(plan_node
#   plan_lib
#   plan_base_lib
#   block_chooser_lib
#   block_lib
#   collision_lib
#   base_lib
#   billiard_data
#   graph_lib
#   sense_data_lib
#   act_data_lib
#   ${OpenCV_LIBS}
#   ${catkin_LIBRARIES}
#   )

# target_link_libraries(io_link_node
#     io_link_lib
#    ${MODBUS_LIBRARIES}
#     ${catkin_LIBRARIES}
# )
