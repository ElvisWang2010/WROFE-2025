# Building Instructions

## Motor Mechanism

### Step 1
Put the belt and 12 mm ball bearings onto the shaft.  
<img width="600" alt="Step 1" src="https://github.com/user-attachments/assets/a2987109-9992-453f-a72f-bddd081fd541" />

---

### Step 2
Insert 4 of the 6 mm ball bearings into the holes, then place the shaft mechanism down on top of the 12 mm ball bearing holder.  
<img width="600" alt="Step 2" src="https://github.com/user-attachments/assets/755b32c9-1db5-476a-bc96-4c083b12c792" />

---

### Step 3
Place the top half of the motor mechanism on top of the bottom half.  
<img width="600" alt="Step 3" src="https://github.com/user-attachments/assets/437a7066-cc08-4258-9962-17ff6d62dabf" />

---

### Step 4
Screw the top to the bottom (upside down).  
<img width="600" alt="Step 4" src="https://github.com/user-attachments/assets/067696be-cac3-4ae4-b99e-6b38ead02792" />

---

### Step 5
Screw the ACXWA motor mount into the two high holes.  
<img width="600" alt="Step 5" src="https://github.com/user-attachments/assets/fa540000-e1f0-466f-8508-706494be5705" />

---

## Turning Mechanism

### Step 1
Put 6 ball head screws onto the caster block.  
<img width="600" alt="Step 1" src="https://github.com/user-attachments/assets/d87e028d-0c1c-433c-b1f8-9778088a66d0" />

---

### Step 2
Connect the caster block to the suspension arm.  
<img width="600" alt="Step 2" src="https://github.com/user-attachments/assets/f1c13142-cf7f-45d5-987f-d48232a1a89f" />

---

### Step 3
Connect the toe link to the outer ball head screws.  
<img width="600" alt="Step 3" src="https://github.com/user-attachments/assets/d81d0363-0046-4be2-8719-2df912aac021" />

---

### Step 4
Screw in ball screws to the servo horn.  
<img width="600" alt="Step 4" src="https://github.com/user-attachments/assets/bac2505d-1dc4-43a9-ac13-41494a90e503" />

---

### Step 5
Connect the toe link to the ball screw attached to the servo horn, then screw the servo horn onto the servo.  
<img width="600" alt="Step 5" src="https://github.com/user-attachments/assets/3170d68a-c79d-4c4a-930c-f361121de986" />

---

### Step 6
Attach four 6 mm ball bearings to the caster block, insert the axle through, then attach the wheel and screw it in.  
<img width="600" alt="Step 6" src="https://github.com/user-attachments/assets/7b6f6226-9883-40d0-a468-56d8355eff5f" />

---

# Building the Entire Robot

### Step 1
Screw in the turning mechanism and place the servo into its mount.  
<img width="600" alt="Step 1a" src="https://github.com/user-attachments/assets/fcac0f32-5a25-4d98-9a94-a2eec8b31c01" />  
<img width="600" alt="Step 1b" src="https://github.com/user-attachments/assets/0dd37268-4dc7-40fd-ae3e-15047285d242" />

---

### Step 2
Attach the moving mechanism to the back.  
<img width="600" alt="Step 2a" src="https://github.com/user-attachments/assets/9a6e2113-e1b1-4480-a47b-5dd69d9b059e" />  
<img width="600" alt="Step 2b" src="https://github.com/user-attachments/assets/4eec024c-64c4-4afd-8b11-8593ff490422" />

---

### Step 3
Install M2 screws and M2 × 3 mm male brass standoffs on top.  
Afterwards, place the Raspberry Pi 5 onto the standoffs and secure it using M2 standoffs of your choice.  
<img width="600" alt="Step 3" src="https://github.com/user-attachments/assets/5334fd1d-4e08-42f1-8c47-afe4e493c830" />

---

### Step 4
Screw an M3 × 15 mm male brass standoff into the highlighted holes.  
<img width="600" alt="Step 4a" src="https://github.com/user-attachments/assets/b8910cd1-ca6a-4693-bab1-ad1d922c5a00" />  

It should look something like this:  
<img width="600" alt="Step 4b" src="https://github.com/user-attachments/assets/2f4b6e7b-2467-475b-87c9-bf8fafb6d179" />

---

### Step 5
Mount the Hiwonder controller/expansion board on top of the M3 × 15 mm and 10 mm male brass standoffs.  
Secure it to the M3 × 15 mm standoffs. Note: the standoff closest in the picture uses a smaller female brass standoff.  
<img width="600" alt="Step 5" src="https://github.com/user-attachments/assets/04eaf78f-ef2f-49f8-8908-6bf4bd5a2d2e" />

---

### Step 6
Insert 3 mm screws from the bottom upwards, then fasten them into the three brass standoffs above (M3 × 15 mm and 10 mm).  
<img width="600" alt="Step 6a" src="https://github.com/user-attachments/assets/696874f9-d8da-4aee-afd9-bedd4abecb65" />  

It should look like this:  
<img width="600" alt="Step 6b" src="https://github.com/user-attachments/assets/dcf29feb-1cf0-4c43-bab8-03e3e6434c14" />

---

### Step 7
Place the ESC into its holder.  
<img width="600" alt="Step 7" src="https://github.com/user-attachments/assets/e4baf873-70fe-408b-9037-bb3a526479a9" />

Now, you've finished the hardware!

---

</br>

# Software Instructions

Since we use a CSI camera, we need to use picamera2. The problem with this is that we use ROS2 and picamera2 cannot be installed in ROS2 docker container.

There are 3 possible solutions:

**Option1:** Use USB camera instead of CSI camera. USB camera can be accessed without picamera2.


**Option2:** Split the project into two parts:
- Create a ROS2 node that runs in Raspberry pi host (instead of inside docker container). This node
publishes camera related data
- Create the main ROS2 project in the docker container. It subscribes the topic that publishes
camera data. It contains the main logic of the project.


**Option3:** Create the whole ROS2 project in Raspberry pi host. It can access camera via picamera2. It
also subscribes topics published by third-party ROS nodes in ROS2 container

*Note: Both option 2 and 3 require us to build a ROS2 environment from source in Raspberry Pi OS.*

We did not want to use a USB camera, instead we opted to build a ROS2 environment from source.

## Building the Environment


#### 1. Build a ROS2 environment from source in Raspberry Pi OS

</br>
   
#### 2. Switch from AP mode to STA mode so that raspberry pi has access to the internet
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
*The IP range to scan should be the same range of your computer’s IP address. For example,
my computer’s IP address is 172.16.0.223. The scan range is 172.16.0.1~172.16.0.254 (or 255)
After scan is complete, Uncheck “View-&gt;Show Dead”. Check “View-&gt;Show Details Pane” so that you
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

#### 5. Create a new connection in RealVNC Viewer using the IP address of your raspberry Pi as shown below and connect to it.

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
- use command "docker ps -a" to find the container ID of 'MentorPi' 
- run "docker cp adb8457c2eec:/home/ubuntu/ros2_ws/src/driver/ros_robot_controller_msgs ~/fe_ws/src/"  
  where adb8457c2eec should be replace by the container ID of 'MentorPi' you got from previous step
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

### Step 9: Test with Commands Below. The First Command Should List All the ROS2 Topics. The Second Command Will Wait for you to Press Button 1 or 2
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
Congratulations! You have built ROS2 environment under Raspberry Pi Host and created a ROS2
workspace “fe_ws”. Now you can create ROS2 packages and ROS2 nodes in this workspace.


