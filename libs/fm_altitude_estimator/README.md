# fm_altitude_estimator
A Kalman filter to fuse GNSS change in altitude, lidar, and radar altimeter into single altitude above ground estimate.

## Plotting
The plot script creates quick introspection into the altitude estimation.
```
rosrun fm_altitude_estimator plot --help
```

E.g.
```
rosrun fm_altitude_estimator plot sensors_2021-01-21-13-00-10.bag /owl/fm_altitude_estimator/state /owl/dji_sdk/local_position /owl/us_d1/data /owl/lidar_lite/data
```
![Example image of altitude estimation](https://user-images.githubusercontent.com/11293852/106176528-b59f3d00-6197-11eb-8d76-31c3f6488d1e.png)
