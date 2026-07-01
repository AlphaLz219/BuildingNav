#! /bin/bash

gnome-terminal -- bash -c "cd /home/$USER/dog; source devel/setup.bash; roslaunch mydog_control_sim_ros sim.launch; exec bash"
