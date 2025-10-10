## Building Instructions

### Step 1 — Steering knuckles: add bearings
<img src="media/building-photos/building-instruction-photo.png" alt="" width="100%">
Insert four 6 mm ball bearings and axles.

---

### Step 2 — Prep for wheels
<img src="media/building-photos/building-instruction-photo2.png" alt="" width="100%">
Insert axle and screw in wheel.

---

### Step 3 — Link knuckles to control arm
<img src="media/building-photos/building-instruction-photo3.png" alt="" width="100%">
Connect both steering knuckles to the control arm.

---

### Step 4 — Mount steering to servo base
<img src="media/building-photos/building-instruction-photo4.png" alt="" width="100%">
Screw the control-arm block onto the servo base.

---

### Step 5 — Prepare servo horn
<img src="media/building-photos/building-instruction-photo5.png" alt="" width="100%">
Place the horn on the servo output.

---

### Step 6 — Fix horn to servo
<img src="media/building-photos/building-instruction-photo6.png" alt="" width="100%">
Tighten the horn screw.

---

### Step 7 — Add anti-sway arm
<img src="media/building-photos/building-instruction-photo7.png" alt="" width="100%">
Connect the anti-sway arm to the horn.

---

### Step 8 — Complete steering linkage
<img src="media/building-photos/building-instruction-photo8.png" alt="" width="100%">
Join anti-sway arm to control arm; confirm free motion.

---

### Step 9 — Rear body mount: add bearings
<img src="media/building-photos/building-instruction-photo9.png" alt="" width="100%">
Insert four 6 mm bearings and axles.

---

### Step 10 — Rear axle & wheel prep
<img src="media/building-photos/building-instruction-photo10.png" alt="" width="100%">
Insert axle and secure wheel.

---

### Step 11 — Install T-joint axle and gear
<img src="media/building-photos/building-instruction-photo11.png" alt="" width="100%">
Slide in T-joint axle; screw on the main gear.

---

### Step 12 — Attach motor mount
<img src="media/building-photos/building-instruction-photo12.png" alt="" width="100%">
Bolt the motor mount to the rear assembly.

---

### Step 13 — Mount the motor
<img src="media/building-photos/building-instruction-photo13.png" alt="" width="100%">
Screw the motor to the mount.

---

### Step 14 — Add small motor gear
<img src="media/building-photos/building-instruction-photo14.png" alt="" width="100%">
Press-fit/secure the pinion on the motor shaft.

---

### Step 15 — Belt and rear assembly install
<img src="media/building-photos/building-instruction-photo15.png" alt="" width="100%">
Loop belt over both gears; screw assembly to chassis.

---

### Step 16 — Chassis overview
<img src="media/building-photos/building-instruction-photo16.png" alt="" width="100%">
Identify bottom chassis, turning mechanism, and drive module.

---

### Step 17 — Mount RRC Lite controller
<img src="media/building-photos/building-instruction-photo17.png" alt="" width="100%">
Align controller posts; screw in firmly.

---

### Step 18 — Add top chassis & Pi mount
<img src="media/building-photos/building-instruction-photo18.png" alt="" width="100%">
Slide in the top plate; screw down Raspberry Pi 5.

---

### Step 19 — Install ESC
<img src="media/building-photos/building-instruction-photo19.png" alt="" width="100%">
Slide the ESC into the tower slot.

---

### Step 20 — Mount camera plate
<img src="media/building-photos/building-instruction-photo20.png" alt="" width="100%">
Screw camera to the top plate (four points).

---

### Step 21 — Final verification
<img src="media/building-photos/building-instruction-photo21.png" alt="" width="100%">
Check cable routing, belt tension, and free steering/drivetrain.

</br>

# Software Instructions

Since we use a CSI camera, we need to use picamera2. The problem with this is that we use ROS2 and picamera2 cannot be installed in a ROS2 docker container.

There are 3 possible solutions:

**Option1:** Use USB camera instead of CSI camera. A USB camera can be accessed without picamera2.


**Option2:** Split the project into two parts:
- Create a ROS2 node that runs on a Raspberry Pi host (instead of inside docker container). This node
publishes camera-related data
- Create the main ROS2 project in the docker container. It subscribes to the topic that publishes
camera data. It contains the main logic of the project.


**Option3:** Create the whole ROS2 project in a Raspberry pi host. It can access the camera via picamera2. It
also subscribes topics published by third-party ROS nodes in ROS2 container

*Note: Both options 2 and 3 require you to build a ROS2 environment from source in Raspberry Pi OS.*

We did not want to use a USB camera, instead we opted to build a ROS2 environment from source.

## Building the Environment


#### 1. Build a ROS2 environment from source in Raspberry Pi OS

</br>
   
#### 2. Switch from AP mode to STA mode so that Raspberry Pi has access to the internet
- Run command “nano hiwonder-toolbox/wifi_conf.py”, edit the file with your wifi and password.
Save the changes and reboot via command “sudo reboot”
- In the WIFI_STA_SSID, uncomment it if commented and type in your desired wifi SSID.
- In the WIFI_STA_PASSWORD, uncomment it if commented and type in your wifi password.

*Make sure to type in these correctly. If typed in incorrectly, you may need to reflash your SD card.*

</br>

<div align="center">

<table>
  <tr>
    <td style="border: 200px solid black; padding: 5px;">
      <img width="556" height="351" alt="image" src="https://github.com/user-attachments/assets/98e79421-b81a-4495-a92f-75dfc845175b" />
    </td>
  </tr>
</table>

</div>

</br>

#### 3. Find the IP address of your computer (which is connected to the same wifi access point) by running command “ipconfig” in windows command prompt.
*To open command prompt, hit windows + r and type in cmd.*

</br>

<div align="center">

<table>
  <tr>
    <td style="border: 200px solid black; padding: 5px;">
      <img width="553" height="291" alt="image" src="https://github.com/user-attachments/assets/75df32fb-d53d-4c06-8d33-bcc1eeecf349" />
    </td>
  </tr>
</table>

</div>

</br>

#### 4. Use Advanced IP Scanner or Angry IP Scanner to find the IP address of your Raspberry Pi.
*The IP range to scan should be the same as your computer’s IP address. For example,
my computer’s IP address is 172.16.0.223. The scan range is 172.16.0.1~172.16.0.254 (or 255)
After the scan is complete, Uncheck “View-&gt;Show Dead”. Check “View-&gt;Show Details Pane” so that you
can see the detailed pane on the right side as shown below. Navigate through the list of devices
found, looking for “ROS Image …” as shown below.*

</br>

<div align="center">

<table>
  <tr>
    <td style="border: 200px solid black; padding: 5px;">
      <img width="558" height="373" alt="image" src="https://github.com/user-attachments/assets/92b27077-e5c1-4ce5-a431-465f1974d1cd" />
    </td>
  </tr>
</table>

</div>

</br>

#### 5. Create a new connection in RealVNC Viewer using the IP address of your Raspberry Pi as shown below, and connect to it.

</br>

<div align="center">

<table>
  <tr>
    <td style="border: 200px solid black; padding: 5px;">
      <img width="367" height="261" alt="image" src="https://github.com/user-attachments/assets/8ae7badd-ccbd-4945-aae0-88d5395c49dd" />
    </td>
  </tr>
</table>

</div>

</br>

#### 6. Go into LX Terminal and type/paste in the following code.

### Step 1: Install System Dependencies
```
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
  build-essential cmake git wget curl gnupg lsb-release \
  libpython3-dev python3-colcon-common-extensions \
  python3-flake8 python3-pip python3-pytest-cov python3-rosdep \
  python3-setuptools python3-vcstool python3-numpy \
  libasio-dev libtinyxml2-dev libcunit1-dev unzip libyaml-dev
sudo apt install -y libfastcdr-dev libfastrtps-dev
sudo apt install -y python3-dev python3-numpy
```

### Step2: Initialize Rosdep
```
sudo rosdep init || true
rosdep update
```
### Step3: Create ROS2 Workspace and Clone Minimal ROS2 Sources
```
mkdir -p ~/fe_ws/src
cd ~/fe_ws
wget https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos
vcs import src < ros2.repos
```

### Step 4: Install ROS 2 Source Dependencies
```
cd ~/fe_ws
rosdep install --from-paths src --ignore-src -y --rosdistro humble \
--skip-keys="fastcdr fastrtps rti-connext-dds-6.0.1 urdfdom_headers"
```
### Step 5: Install Missing Custom Message 'ros_robot_controller_msgs/msg/ButtonState'
```
- use the command "docker ps -a" to find the container ID of 'MentorPi' 
- run "docker cp adb8457c2eec:/home/ubuntu/ros2_ws/src/driver/ros_robot_controller_msgs ~/fe_ws/src/"  
  where adb8457c2eec should be replaced by the container ID of 'MentorPi' you got from the previous step
```

### Step 6: Build The Workspace
```
colcon build --symlink-install --packages-skip-regex '^rviz' 
```

### Step 7: Switch to Bash and Reboot
```
chsh -s /bin/bash 
sudo reboot
```

### Step 8: Source Your Environment
```
echo "source ~/fe_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 9: Test with the Commands Below. The First Command Should List All the ROS2 Topics. The Second Command Will Wait for you to Press Button 1 or 2
```
ros2 topic list
ros2 topic echo /ros_robot_controller/button
```


### Additional Step: Go Back to AP Mode With DHCP Issue Prevented
- update hiwonder-toolbox/wifi_conf.py to switch from STA mode to AP mode
```
sudo nano /etc/dnsmasq.d/wifi_ap.conf
- Paste the content below and save it
interface=wlan0
dhcp-range=192.168.149.10,192.168.149.100,12h
bind-interfaces

sudo systemctl restart dnsmasq
sudo reboot
```

Congratulations! You have built a ROS2 environment under a Raspberry Pi Host and created a ROS2
workspace “fe_ws”. You can now create ROS2 packages and nodes in this workspace.

Suppose you combine both Hardware and Software, hooray! You have built our entire car!



