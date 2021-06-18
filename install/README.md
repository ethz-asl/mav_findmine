# Installation from Source
This folder contains installation instructions to install the FindMine system from source.

In general, the same instructions apply for the UAV, the base station computer, and the GNSS base station.
In the following the individual requirements are marked with (UAV, BASE, GNSS).

## Step 0:
Connect the Up Squared that needs to be configured to a screen, keyboard, and Ethernet with internet access.
![Initial computer setup](https://user-images.githubusercontent.com/11293852/104500638-f9fadc80-55de-11eb-8420-4fb6f4826b59.jpg)

### Configure BIOS (UAV, GNSS)
- Start up the computer and while booting press <kbd>DEL</kbd>
- No password is required to do regular changes.

1. Switch to mSata
![mSata Switch](https://user-images.githubusercontent.com/11293852/104491109-12182f00-55d2-11eb-851a-52e6d13d6919.jpg)
2. Set SATA Device Type to SSD
![SATA Device Type SSD](https://user-images.githubusercontent.com/11293852/104491157-24926880-55d2-11eb-8a46-4a006b463b79.jpg)
3. Disable Console Redirection
![Disable Console Redirection](https://user-images.githubusercontent.com/11293852/104491146-20664b00-55d2-11eb-9537-a9aa6f1c0de1.jpg)
4. Save Changes and Reset
![Save and Reset](https://user-images.githubusercontent.com/11293852/104491212-396efc00-55d2-11eb-89dc-f148cda67646.jpg)

### Install Ubuntu (all)
- Prepare a pen drive with [Ubuntu 18.04 Server](https://releases.ubuntu.com/18.04/ubuntu-18.04.5-live-server-amd64.iso), e.g., using the [Balena Etcher tool](https://www.balena.io/etcher/).
- **OR** on the base station computer install [Ubuntu 18.04 Desktop](https://releases.ubuntu.com/18.04/ubuntu-18.04.5-desktop-amd64.iso)
- Start up the Up Squared with the pen drive inserted and follow the installation instructions.

1. Select desired language
![Select Language](https://user-images.githubusercontent.com/11293852/104500666-01ba8100-55df-11eb-96a0-8bac82093f20.jpg)
2. Update to the new installer (or do not if you desire to have a fixed Ubuntu version)
![Update to the new installer](https://user-images.githubusercontent.com/11293852/104500690-08e18f00-55df-11eb-9752-1d949541992f.jpg)
3. Select desired keyboard layout
![Keyboard layout Switzerland](https://user-images.githubusercontent.com/11293852/104500718-0e3ed980-55df-11eb-8de1-b3b8373b9ab9.jpg)
4. Make sure enp2s0 is connected to the internet
![Network connections](https://user-images.githubusercontent.com/11293852/104500737-1434ba80-55df-11eb-85c7-58067c41a1a2.jpg)
5. Select 500Gb Kingston hard drive for installation. Use entire disk. Do not enable LVM.
![Storage selection](https://user-images.githubusercontent.com/11293852/104500769-257dc700-55df-11eb-93bf-ccad51d906a1.jpg)
6. Reformat the 58 Gb disk that contains the stock OS to avoid confusions later
![Storage config](https://user-images.githubusercontent.com/11293852/104500880-43e3c280-55df-11eb-8102-370b22e764f6.jpg)
7. Set desired host name, user name, and password. Use the same name for host and user.
![Profile setup](https://user-images.githubusercontent.com/11293852/104500840-38909700-55df-11eb-9224-e981a180ea9a.jpg)
8. Already install OpenSSH
![Enable OpenSSH](https://user-images.githubusercontent.com/11293852/104500790-29114e00-55df-11eb-87b3-397037a6aa7e.jpg)
9. Do not install any additional features.
![No featured snap selection](https://user-images.githubusercontent.com/11293852/104500813-2f072f00-55df-11eb-8c92-f888f13ae330.jpg)
10. Start the installation

If you enabled updating the installer, it may take a little longer to install.
After a while you should be asked to reboot.
Remove the pen drive while rebooting.

### Initial Repo Download (all)
At this point you do not need to be connected via monitor anymore and can connect via SSH.
```
ssh-copy-id user@host
ssh user@host
```

Download a copy of the mav_findmine repository into the home directory.
```
sudo apt install -y git
git clone https://github.com/ethz-asl/mav_findmine
cd mav_findmine
git checkout v1.0.5
```

Then move into the installation folder.
```
cd ./install
```

## Step 1: Install hardware extensions (UAV, GNSS)
The Up Squared on the drone runs a specific Kernel that activates the peripheral, e.g., UART and GPIOs.
To update the Kernel and install some system utils run:
```
./01_install_hwe.sh
```

This installation will remove all other Kernels.
You can continue installation by selecting `no` in the opening window.
![Remove old kernels](https://user-images.githubusercontent.com/11293852/104506168-4e08bf80-55e5-11eb-80d4-335713dfc6dc.jpg)

After the installation the machine will reboot and you will have to connect again.
```
ssh user@host
cd mav_findmine/install
```

## Step 2: Install networking (UAV, GNSS)
We use netplan to configure WiFi access and network interfaces.
Run the following script to setup the necessary configurations:
```
./02_install_netplan.sh
```
**Important:** At this point make sure, that the computer is only connected to one network, and this network has an internet connection!
For example, do not have the GNSS base station running in parallel.

## Step 3: Install git (all)
We use git to manage software versioning.
Set up your git account with
```
./03_install_git.sh
```
During the installation you are asked to add the key to your Github account such that you can use SSH authentification from this machine.

## Step 4: Software building process (all)
Now, we can start the software installation process.
The following script will install the robot operating system (ROS) and all software required for sensing and navigation.
In particular it will create a catkin workspace with [versioned dependencies](dependencies.rosinstall).
```
./04_install_ros.sh
```

### Spinnaker camera driver (UAV)
During the installation of the camera you will be asked to modify the MAC address in the netplan config.
The RGB camera needs a specific static IP configuration, otherwise the following error occurs during sensor startup:
```
[ERROR] [1605608289.684076384]: Reconfigure Callback failed with error: [SpinnakerCamera::connect] Failed to get first connected camera. Error: Spinnaker: Index is out of range, list error = vector::_M_range_check: __n (which is 0) >= this->size() (which is 0) [-1009]
```
The camera requires jumbo frames in order to transmit its high resolution images.
For this purpose we need to adapt the packet size in the netplan config.

Connect to the software in a second terminal.
```
ssh user@host
```

1. Identify enp3s0 macaddress:

    1. `ip link show`, e.g., `00:07:32:73:8d:19`
    ```
    3: enp3s0: <BROADCAST,MULTICAST> mtu 1500 qdisc noop state DOWN mode DEFAULT group default qlen 1000
    link/ether 00:07:32:73:8d:19 brd ff:ff:ff:ff:ff:ff
    ```
2. Correct macaddress
    1. Edit macaddress in netplan config `sudo nano /etc/netplan/01-netcfg.yaml`, remove commented out static IP section and remove dhcp section
    ```
    enp3s0:
        match:
            macaddress: 00:07:32:73:8d:19
        addresses:
            - 169.254.44.1/16
        mtu: 9000
        optional: true
    ```
    2. `sudo netplan apply`
3. Check configuration
    1. `ifconfig` should now show the correct static IP address and MTU 9000

## Step 5: Install sensor autostart (UAV)
The sensors are automatically booted at startup once the Versavis is recognized.
To enable this functionality run the following script after entering the installation folder again.
```
cd ~/catkin_ws/src/mav_findmine/install
./05_install_sensor_autostart.sh
```

## Step 6: Install radar driver (UAV)
Next, we install the necessary system dependencies to run the radar driver.
```
./06_install_radar.sh
```

## Step 7: Install PPS sync (UAV, GNSS)
In order for the radar to be time stamped within GNSS time, the Up Squared needs to be synchronized with respect to the GNSS receiver.
This time synchronization can be installed automatically given the provided [installation script from ethz_piksi_ros driver](https://github.com/ethz-asl/ethz_piksi_ros/blob/master/piksi_pps_sync/install/install.sh).

```
./07_install_pps.sh
```

The installation asks you, if your GPIO is interruptable, which you confirm as well as all other requests.<br>
**Important:** At the end of the installation, you will be asked to sign the module.
You will have to set a password, that you will be asked during the reboot.
Make sure, you have the screen and keyboard still connected to supervise the signing process.

1. Interrupt the boot pressing any key, e.g., <kbd>→</kbd>.
![Interrupt boot](https://user-images.githubusercontent.com/11293852/104521250-cf1f8100-55fc-11eb-9588-c79a35ebb5db.jpg)
2. Enroll MOK
![Enroll MOK](https://user-images.githubusercontent.com/11293852/104515904-c0809c00-55f3-11eb-9812-d67e35e85cb9.jpg)
3. Continue
![Continue](https://user-images.githubusercontent.com/11293852/104515911-c4acb980-55f3-11eb-9d66-363c15f04cdd.jpg)
4. Enroll the key
![Enroll](https://user-images.githubusercontent.com/11293852/104515915-c5dde680-55f3-11eb-9376-7d85409c7e21.jpg)
5. Enter password
![Password](https://user-images.githubusercontent.com/11293852/104515918-c70f1380-55f3-11eb-8b99-8693aeb83d16.jpg)
6. Reboot
![Reboot](https://user-images.githubusercontent.com/11293852/104521254-d050ae00-55fc-11eb-8614-a2c6b56d5908.jpg)

## Step 8:
Congratulations, you finished setting up the UP Squared software!
You can mount the computer on the sensor pod! :partying_face:

## Step 9:
Set the following options in DJI Assistant 2 for Matrice
![DJI SDK Settings](https://user-images.githubusercontent.com/11293852/58762990-08a19680-8556-11e9-9dc4-652c4d4180f2.png)

We use firmware package version 1.0.1.67. (Some further reading [here](https://github.com/ethz-asl/mav_findmine/issues/177))

# Step 10:
Do not forget to install the other devices on the sensor pod.
- [RFD 868x Modem](https://github.com/ethz-asl/ethz_piksi_ros/wiki/RFD-868x-Modem)
- [Piksi GNSS Receivers](https://github.com/ethz-asl/mav_findmine/wiki/11-Hardware-Commissioning#piksi-multi-gnss)
- [VersaVIS](https://github.com/rikba/versavis/tree/feature/gnss_sync/firmware#installation-via-atmel-studio-and-hex)
