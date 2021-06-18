# MIT License
#
# Copyright (c) 2020 Rik Baehnemann, ASL, ETH Zurich, Switzerland
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Install service.
echo "Do you wish to configure sensor autostart? [y or Y to accept]"
read configure_sensor_autostart
if [[ $configure_sensor_autostart == "Y" || $configure_sensor_autostart == "y" ]]; then
  echo "Configuring /etc/systemd/system/sensors.service"
  sudo rm /etc/systemd/system/sensors.service
  sudo sh -c "tee -a /etc/systemd/system/sensors.service << END
[Unit]
Description=Start sensors automatically on startup.
After=network-online.target
StartLimitBurst=5
StartLimitIntervalSec=60

[Service]
Type=simple
ExecStart=/home/$USER/catkin_ws/src/mav_findmine/install/startup_sensors.sh
Restart=always
RestartSec=5
User=$USER

[Install]
WantedBy=multi-user.target
END"
fi

sudo systemctl daemon-reload

#---------------- Udev Rule ----------------
echo " "
echo "Do you wish to create udev rule for Versavis? [y or Y to accept]"
read create_udev_rule
if [[ $create_udev_rule == "Y" || $create_udev_rule == "y" ]]; then
  echo "Configuring /etc/udev/rules.d/98-versa-vis.rules"
  sudo rm /etc/udev/rules.d/98-versa-vis.rules
  sudo sh -c "tee -a /etc/udev/rules.d/98-versa-vis.rules << END
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"2341\", ATTRS{idProduct}==\"804d\", SYMLINK+=\"versavis\", GROUP=\"dialout\"
SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"2341\", ATTRS{idProduct}==\"804d\", TAG+=\"systemd\", ENV{SYSTEMD_WANTS}+=\"sensors.service\"
END"
  sudo /etc/init.d/udev restart
fi

echo "Do you wish to add user to group dialout? [y or Y to accept]"
read join_dialout
if [[ $join_dialout == "Y" || $join_dialout == "y" ]]; then
  sudo usermod -a -G dialout ${USER}
fi
