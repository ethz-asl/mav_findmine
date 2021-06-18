# fm_sensor_watchdog
A PyQT GUI to monitor sensor status and engage a mission.

## Chrony
Chrony synchronizes the system clock with GNSS time.
At the moment the sensor watchdog does not support visualizing the chrony offset thus this has to be done manually.

Before starting the radar ensure that the system is synchronized against GNSS time with chrony.
```
ssh user@host
chronyc sources -v
```
Chrony should be synced against `PPS0` and have an offset in nanoseconds range.

![Chrony sources information.](https://user-images.githubusercontent.com/11293852/58467886-c551bd00-813c-11e9-992a-8bb15043b3a3.png)
