# Sensors Stack 2025

Clone this repo into src directory. 


If your looking for the nav/sensors onboarding go here instead: 
https://github.com/umigv/sensors-onboarding

# Follow the installation here to get dependinces:
https://www.stereolabs.com/docs/ros2


## Usage:
### (Assuming we already have everything installed)

Check if your GNSS module is detected:
```
ls /dev/tty*
```

You should see **/dev/ttyACM0**. To verify the raw GNSS data stream:

```
cat /dev/ttyACM0  # No sudo required
```
The output should look something like this:
```
$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,1*33
$GPGSV,1,1,00,0*65
$GNRMC,082019.00,V,,,,,,,230623,,,N,V*1D
```

To start up the gpsd daemon:
```
gpsd -nG -s 115200 /dev/ttyACM0
```
By default, `gpsd` runs as a **systemd service** (`systemctl start gpsd`), meaning it’s managed automatically as a **daemon**. However, for **USB GNSS modules**, this setup can be unreliable—sometimes the service starts before the module is detected, or it may not restart properly.

To avoid these issues, it’s better to start the **daemon** manually instead of relying on `systemd`. Since we still want `gpsd` to start at boot, we can set up a **cron job** that runs the command automatically when the system starts.

Then we can check if the GPS and gpsd is working using this command that displays GNSS and satellite data via a GUI:
```
xgps
```

⚠️ **Warning:** It may take several minutes for the GNSS module to acquire a GPS fix, especially in areas with poor satellite visibility. Ensure you are in an open area with a clear view of the sky for faster results. On cold start (No prior data), it can take around **3-5 minutes** in open areas (longer in poor conditions).


## Running the Sensors Stack
1. Open 2 terminals.
2. `cd ~/ros2_ws` in each terminal.
3. run `source /opt/ros/humble/setup.zsh` && `source install/setup.zsh` in each terminal.
4. Terminal 1: `ros2 launch ublox_gps gnss_test.launch.py`
5. Terminal 2: `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i`
   

## Installation and Setup 
### Install GPSD
The **simplest way** to retrieve GNSS data on Linux is by using **gpsd**, a service daemon that handles GNSS data and provides easy API access.
7. Open new terminal.
8. run: `source /opt/ros/humble/setup.bash`
9. run: `source install/setup.bash`
10. run: `ros2 launch marvin_bot_description teleop_launch.py`


This guide walks you through setting up your GNSS with the **ZED SDK’s Global Localization module**. While the instructions focus on the **Ublox ZED F9P GNSS module**, they should also apply to other GNSS modules. 
### Installation

The **simplest way** to retrieve GNSS data on Linux is by using **gpsd**, a service daemon that handles GNSS data and provides easy API access.

Installing `gpsd` from source ensures you have the most recent stable version.

```
# Install dependencies
sudo apt update && sudo apt install scons libgtk-3-dev

# Clone and compile gpsd
git clone https://gitlab.com/gpsd/gpsd.git
cd gpsd && git checkout 8910c2b60759490ed98970dcdab8323b957edf48
sudo ./gpsinit vcan
scons && scons check && sudo scons udev-install

# Add Python path for gpsd tools
echo 'export PYTHONPATH="$PYTHONPATH:/usr/local/lib/python3/dist-packages"' >> ~/.bashrc
```
### **Granting Permissions**

To allow gpsd to run without root privileges, add your user to the necessary groups:
```
sudo adduser $USER dialout
sudo adduser $USER tty
```

Log out or reboot for changes to take effect.
### Verifying GNSS Module Detection

Check if your GNSS module is detected:

```
ls /dev/tty*
```

You should see **/dev/ttyACM0**. To verify the raw GNSS data stream:

```
cat /dev/ttyACM0  # No sudo required
```

The output should look something like this:

```
$GNGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99,1*33
$GPGSV,1,1,00,0*65
$GNRMC,082019.00,V,,,,,,,230623,,,N,V*1D
```

## Running gpsd

By default, **gpsd** runs as a systemd service, but USB GNSS modules may work better if gpsd is started manually.

### **Starting gpsd Manually**

```
gpsd -nG -s 115200 /dev/ttyACM0
```

Replace **/dev/ttyACM0** with your actual GNSS device port.

### **Starting gpsd at Boot**

To automatically start gpsd at boot, add the following cron job using:

```
crontab -e
```

Add the following line to the file you're editing (with vim/nano/whatever you prefer)

```
@reboot sleep 10 && /usr/local/sbin/gpsd -nG -s 115200 /dev/ttyACM0
```
Then we check if **gpsd** is working properly using the graphical **xgps** tool:
```
xgps  # Displays GNSS and satellite data in a GUI
```

## Running the Sensors Stack
1. Open 2 terminals.
2. `cd ~/ros2_ws` in each terminal.
3. run `source /opt/ros/humble/setup.zsh` && `source install/setup.zsh` in each terminal.
4. Terminal 1: `ros2 launch ublox_gps gnss_test.launch.py`
5. Terminal 2: `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i`
