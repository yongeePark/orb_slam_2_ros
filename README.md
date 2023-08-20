### in case you are using compressed image
roslaunch orb_slam_2_ros scout.launch
rosrun image_transport republish compressed in:=$(compressed_image_topic) _image_transport:=compressed raw out:=$(raw_image_topic)

