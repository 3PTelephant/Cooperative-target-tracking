gnome-terminal --window -e 'bash -c "sleep 3; rosrun cotrack bebop_track4; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun cotrack bebop_track3; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun cotrack bebop_track2; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun cotrack bebop_track1; exec bash"' \
--tab -e 'bash -c "sleep 3; rosrun cotrack statemachine; exec bash"' \
