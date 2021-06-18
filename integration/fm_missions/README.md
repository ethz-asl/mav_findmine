# fm_missions
Scripts to start all system components and state machine to execute automatic survey.
In particular this package sets the [state machine behavior](src/fm_missions/single_trajectory.py) and gathers the different [launch files](launch) for sensors, software components, and logging.

## Mission configuration
The command `configure_mission $UAV_NAME` opens the [mission configuration file](cfg/mission/single_trajectory_mission.yaml) on the UAV.
This file contains relevant mission settings that let the user define the behavior of the robot.
