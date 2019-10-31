gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch bebop_driver bebop2_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch vrpn_client_ros sample.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun cotrack state_estimation; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun cotrack bebop_singtrack; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun cotrack statemachine; exec bash"' \
