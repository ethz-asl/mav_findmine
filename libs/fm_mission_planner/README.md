# fm_mission_planner
A RVIZ plugin to plan and visualize survey trajectories.

## User manual
The mission planner can be used both in offline mode from the desk in the office, or in online mode, when the UAV is powered in the field.
The only difference is that offline, the ENU origin, which selects the map segment to be loaded, is specified by the user, while in the field the DJI ENU origin of the UAV is used.
The online mode is automatically opened, when the [mission control center](../../#user-manual) is launched.

### Planning a survey mission offline
1. Edit the [GNSS home point's](cfg/home.gps) latitude and longitude. Common home points are:<br>
THU:<br>
latitude: 48.418718<br>
longitude: 9.945754<br><br>
Soccer field Hoengg:<br>
latitude: 47.409047<br>
longitude: 8.496931<br><br>
Test site Spiez:<br>
latitude: 46.690852<br>
longitude: 7.647493<br><br>
Test site Gannertshofen:<br>
latitude: 48.260556<br>
longitude: 10.170054<br>
2. Start the mission planner<br>
```roslaunch fm_mission_planner mission_planner_offline.launch```<br>
RVIZ will pop up with a backdrop image of the loaded GNSS home point.
By default the backdrop image will be sourced from google tiles server.
Only if the [local map tile server](#local-map-tile-server) is loaded the map is sourced from personal records.
![The default mission planner view](https://user-images.githubusercontent.com/11293852/105852053-3c131d80-5fe4-11eb-992b-761ee8659ac3.png)
3. Select the trajectory type you want to add.
![Trajectory type selection](https://user-images.githubusercontent.com/11293852/105853631-2ef72e00-5fe6-11eb-9661-8722384a66db.png)
4. Edit the trajectory according to the selection parameters. Press <kbd>Confirm</kbd> to save trajectory segment.
![Edit hotpoint trajectory](https://user-images.githubusercontent.com/11293852/105853772-5a7a1880-5fe6-11eb-9da1-e26dcdb277fc.png)
5. Repeat step 3 and 4 until trajectory is complete.
6. Export the trajectory to a `.yaml` file. This file needs to be uploaded to `/home/$USER/trajectories` on the UAV.
![Export trajectory](https://user-images.githubusercontent.com/11293852/105854247-f4da5c00-5fe6-11eb-8f77-ec2842a8a662.png)

#### Coverage Planner
The mission planner also has the trajectory type `Coverage` which is an interface for [polygon_coverage_planning](https://github.com/ethz-asl/polygon_coverage_planning).
This trajectory type can be used to plan camera survey missions.

The coverage planner allows the user to draw a polygon hull with multiple holes.
The planner will combine all the polygons and subtract the holes from the combined polygons.
When using the tool, the user must first select the polygon.
![Select polygon](https://user-images.githubusercontent.com/11293852/105856531-a24e6f00-5fe9-11eb-9a6f-d79cc74be93b.png)
The tool info in the bottom bar shows the controls to modify the polygon shape and (flight path) altitude.
The polygon selection is confirmed with <kbd>Enter</kbd>.
Finally press <kbd>Generate trajectory</kbd> to create the coverage path and <kbd>Confirm</kbd>.
The first element in the list is always a polygon.

## Local map tile server
As described in [fm_pix4d](../fm_pix4d) we can create up-to-date map tiles from Pix4D maps.
The mission planner searches for these tiles in the folder `~/map_tiles` in the home directory of the user.

The map tiles are organized in an online ressource, in our case we can download this ressource with<br>
```
cd ~
git clone data_server_username@data.asl.ethz.ch:/shared/findmine/map_tiles
cd map_tiles
git pull origin
```

## Understanding mission yaml files
The mav_findmine package generates and stores missions based on the [mav_trajectory_generation package](https://github.com/ethz-asl/mav_trajectory_generation). This is a flexible tool to calculate and store parametrised paths for aerial vehicles. The [package readme](https://github.com/ethz-asl/mav_trajectory_generation/blob/master/README.md) provides a detailed explanation of the types of trajectories that can be calculated and stored using the package. For the purpose of mav_findmine, the trajectories are simple polynomials stored as `.yaml` files. There are three high-level tags as explained below.

- `input_constraints`: stores platform dynamic constraints. In most cases, this consists of maximum rotation rates (`omega_*`), forces (`f_*`) and velocities (`v_*`) in vehicle body axes.
- `geodetic_reference`: stores the geodetic reference point, or 'home' position of the trajectory, relative to which the rest of the trajectory is stored. The location is stored in the ENU frame.
- `trajectory`: stores the trajectory parametrised into `segments`, with each segment containing fields for:
    - `D`: dimensions of the trajectory, in mav_findmine D=4, corresponding to _x, y, z_ position and heading
    - `N`: number of polynomial coefficients, in mav_findmine N=10, corresponding to a 10th order polynomial, allowing for snap-continuous trajectory optimisation
    - `time`: length of the trajectory segment in seconds
    - `coefficients`: polynomial coefficients of the trajectory. Each dimension has an independent polynomial function in time, in our case one _N_-dimensional vector in order _x, y, z, ψ_. From [mav_trajectory](https://github.com/ethz-asl/mav_trajectory_generation/blob/master/mav_trajectory_generation/include/mav_trajectory_generation/polynomial.h) _"Coefficients are stored in increasing order with the power of t, i.e. c<sub>1</sub> + c<sub>2</sub>*t + c<sub>3</sub>*t<sup>2</sup> ==> coeffs = [c<sub>1</sub> c<sub>2</sub> c<sub>3</sub>]"_

## Implementation details
Inspired by http://docs.ros.org/kinetic/api/rviz_plugin_tutorials/html/panel_plugin_tutorial.html
