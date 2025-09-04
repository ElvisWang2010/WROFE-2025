<div align="center">


# Engineering Documentation – Team Buzzy Bee


*WRO Future Engineers 2025 Entry*


---
</div>


<div align="center">

## Table of Contents


1&#46; <a href="#team-members">Team Members</a><br>
2&#46; <a href="#complete-construction-manual">Complete Construction Manual</a><br>
3&#46; <a href="#machinery">Machinery</a><br>
4&#46; <a href="#robot-design-overview">Robot Design Overview</a><br>
5&#46; <a href="#assembly-gallery">Assembly Gallery</a><br>
6&#46; <a href="#power-management">Power Management</a><br>
&#8226; <a href="#battery">Battery</a><br>
7&#46; <a href="#mobility-management">Mobility Management</a><br>
&#8226; <a href="#chassis">Chassis</a><br>
&#8226; <a href="#motor">Motor</a><br>
&#8226; <a href="#electronic-speed-controller-esc">Electronic Speed Controller (ESC)</a><br>
&#8226; <a href="#steering">Steering</a><br>
&#8226; <a href="#servo-motor">Servo Motor</a><br>
8&#46; <a href="#software">Software</a><br>
&#8226; <a href="#python-30">Python 3.0</a><br>
&#8226; <a href="#ros2-humble">ROS2 Humble</a><br>
&#8226; <a href="#picamera2">Picamera2</a><br>
&#8226; <a href="#opencv">OpenCV</a><br>
&#8226; <a href="#cv_bridge">Cv_bridge</a><br>
&#8226; <a href="#time">Time</a><br>
&#8226; <a href="#numpy">Numpy</a><br>
&#8226; <a href="#ros_robot_controller_sdk">ROS_ROBOT_CONTROLLER_SDK</a><br>
9&#46; <a href="#open-challenge">Open Challenge</a><br>
10&#46; <a href="#obstacle-challenge">Obstacle Challenge</a><br>
11&#46; <a href="#building-instructions">Building Instructions</a><br>
</div>



</div>




## Team Members

- Ryan Rao - 14
- Elvis Wang - 14
- Harry Xiao - 15

## Complete Construction Manual


The WRO Future Engineers is the competition of the self-driven vehicles. Students of the age group from 15 till 19 need to design a model of a car, equip it with electromechanical components and program it as so it will be able to autonomously drive on the track avoiding objections.

More details about the competition can be found on [the official site of WRO Association](https://wro-association.org/competition/new-competition-formats/future-engineers).

<img src="docs/img/fe-map.png" width="640">

The materials are intended to be build with the [Mkdocs](https://www.mkdocs.org/) site generator.

The example of the site is accessible by https://world-robot-olympiad-association.github.io/future-engineers-gs/.

If you have any suggestions or found some bugs/inconsistencies please report them in form of [the GitHub issue](https://github.com/World-Robot-Olympiad-Association/future-engineers-gs/issues/new)._

## Machinery 
### Components
| Name | Product | Price (CAD)|
| ----------- | ----------- | ----------- |
| RC Car | [`ACXWA CD`](https://www.aliexpress.com/item/1005007495175639.html?src=bing&aff_short_key=UneMJZVf&aff_platform=true&isdl=y&albch=shopping&acnt=135095331&isdl=y&albcp=555018171&albag=1299623888131540&slnk=&trgt=pla-4584826057944442&plac=&crea=81226548307861&netw=o&device=c&mtctp=e&utm_source=Bing&utm_medium=shopping&utm_campaign=PA_Bing_CA_PLA_PC_Hot-Sale_MaxValue_20240715&utm_content=Hot%20sale&utm_term=1%2F28%20car%20chassie&msclkid=b27792305716194d30fcf5a6c0fef479)  | $27 |
| RC Car Battery | [`Gens Ace 1300mAh Battery`](https://www.adrenalinehobby.c1om/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug) | $35 | 
| Drive Motor | [`Furitek Micro Komodo 1212 3450KV Brushless Motor`](https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118) | $35 |
| Servo Motor | [`HS-5055MG 11.9g Metal Gear Digital Micro Servo`](https://hitecrcd.com/hs-5055mg-economy-metal-gear-feather-servo/?srsltid=AfmBOooq_9U4Nehv90Y-tGWqZeo6_1c0_7imuMD9W_dBJmYS1m0sd2Y_) | $25 |
| ESC | [`Furitek Lizard Pro 30A/50A ESC`](https://furitek.com/products/combo-of-furitek-lizard-pro-30a-50a-brushed-brushless-esc-for-axial-scx24-with-bluetooth). | $80 |
| Turning Motor | [`Hitec HS-5055MG Servo Motor`](https://ca.robotshop.com/products/hs-5055mg-metal-gear-micro-servo-motor?srsltid=AfmBOopv8Z7LoCVOEqe16w05ZV-R78dNmy7dappldIxZiQzCJroxcssFc2Y) | $38 |
| CSI Camera | [`SainSmart Wide Angle Fish-Eye Camera`](https://www.amazon.ca/SainSmart-Fish-Eye-Camera-Raspberry-Arduino/dp/B00N1YJKFS/ref=sr_1_5) | $19 |
| Raspberry Pi 5 | [`Raspberry Pi 5`](https://www.amazon.ca/Vemico-Raspberry-Kit-Heatsinks-Screwdriver/dp/B09WXRCYL4/ref=sr_1_3) | $180 |
| Expansion Board | [`RRC Lite Controller`](https://www.hiwonder.com/products/rrc-lite?srsltid=AfmBOoqZuQkdiCruulYju-KXoSowMik5Ov_Vs3-_8TA4Bm_luvoK6Oxn). | $45 |
| Lidar | [`LDROBOT D500 lidar kit TOF laser Radar Lidar Scanner `](https://www.aliexpress.com/item/1005003012681021.html?spm=a2g0o.order_list.order_list_main.11.7a3b18028WK12R). | $82 |



##  Robot Design Overview 
A majority of the robot was 3D printed or replaced.
<table>
  <tr>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/08dde4bb-38ca-4fac-9316-b60eb3069f0e" width="320" alt="Camera, ESC, Raspberry Pi 5 Holder"><br>
      <b>1. Camera, ESC & Raspberry Pi 5 Holder</b><br>
      Mounts the Raspberry Pi 5, ESC, and camera.
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/012318fc-d7e4-40a1-8cab-0527ac1ed8a0" width="320" alt="Servo and Controller Holder"><br>
      <b>2. Servo & Controller Holder</b><br>
      Holds the servo and its controller in alignment.
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/bf2e3ba0-369d-4ea8-bbc9-abe4e0329b85" width="320" alt="Steering Mechanism"><br>
      <b>3. Steering Mechanism</b><br>
      Combines the turning linkage and servo mount to control wheel direction.
    </td>
  </tr>
  <tr>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/d691f26d-0096-4c3b-9ac7-1ecb6748c1cb" width="320" alt="Motor Mechanism"><br>
      <b>4. Motor Mechanism</b><br>
      Houses the drive motor and transmits torque to the wheels/tracks.
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/c26f6fdb-64e1-464d-859f-822a5be7695b" width="320" alt="Lidar Mount"><br>
      <b>5. Lidar Mount</b><br>
      Holds lidar above components to detect pillars and walls.
    </td>
    <td align="center" width="33%">
      <img src="https://github.com/user-attachments/assets/92ab470a-5c97-48ca-aca0-c640ddbe2343" width="320" alt="Full Assembly"><br>
      <b>6. Full Assembly</b><br>
      Complete integrated build with all modules.
    </td>
  </tr>
</table>

---

# Assembly Gallery

<table>
  <!-- Row 1 -->
  <tr>
    <td align="center" style="border:1px solid #ddd; padding:15px;">
      <img src="https://placehold.co/600x400/eee/999999/png?text=Front+View" width="100%" alt="Front view of the complete robot" /><br/>
      <em><strong>Front View</strong><br>Shows the overall profile and front-facing components.</em>
    </td>
    <td align="center" style="border:1px solid #ddd; padding:15px;">
      <img src="https://placehold.co/600x400/eee/999999/png?text=Rear+View" width="100%" alt="Rear view of the complete robot" /><br/>
      <em><strong>Rear View</strong><br>Shows the drive wheels and rear assembly.</em>
    </td>
    <td align="center" style="border:1px solid #ddd; padding:15px;">
      <img src="https://placehold.co/600x400/eee/999999/png?text=Top+View" width="100%" alt="Top down view of the robot" /><br/>
      <em><strong>Top View</strong><br>Shows the layout of all major components on the upper chassis.</em>
    </td>
  </tr>
  <!-- Row 2 -->
  <tr>
    <td align="center" style="border:1px solid #ddd; padding:15px;">
      <img src="https://placehold.co/600x400/eee/999999/png?text=Left+View" width="100%" alt="Left side view of the robot" /><br/>
      <em><strong>Left Side View</strong><br>Shows the side profile and left-side components.</em>
    </td>
    <td align="center" style="border:1px solid #ddd; padding:15px;">
      <img src="https://placehold.co/600x400/eee/999999/png?text=Right+View" width="100%" alt="Right side view of the robot" /><br/>
      <em><strong>Right Side View</strong><br>Shows the side profile and right-side components.</em>
    </td>
    <td align="center" style="border:1px solid #ddd; padding:15px;">
      <img src="https://placehold.co/600x400/eee/999999/png?text=Bottom+View" width="100%" alt="Bottom view of the robot" /><br/>
      <em><strong>Bottom View</strong><br>Shows the underside, wheel mounting, and lower chassis.</em>
    </td>
  </tr>
</table>


# Power Management

## Battery

<table border="2" width="100%">
  <tr>
    <td width="40%" align="center">
      <img src="https://github.com/user-attachments/assets/cf44cd09-a0a1-47c7-8620-c4acd393ab89" width="250" alt="Gens Ace 1300mAh 2S LiPo Battery" />
    </td>
    <td width="60%" valign="top">
      <h2>Gens Ace 1300mAh 2S 7.4V 25C LiPo Battery</h2>
      <ul>
        <li><strong>Voltage:</strong> 7.4V (2S)</li>
        <li><strong>Capacity:</strong> 1300mAh</li>
        <li><strong>Maximum Charge Rate:</strong> 5C (6.5A)</li>
        <li><strong>Discharge Rate:</strong> 45C Continuous / 90C Peak</li>
        <li><strong>Cell Configuration:</strong> 2S1P</li>
        <li><strong>Watt Hours:</strong> 9.62Wh</li>
        <li><strong>Connector:</strong> T-Style (Battery), G-Tech Smart (Balance)</li>
        <li><strong>Dimensions (LxWxH):</strong> 70.87 × 35.24 × 14.5 mm</li>
        <li><strong>Weight:</strong> 90 g</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td colspan="2">
      <h3>Why We Chose This Battery</h3>
      <p>
       This battery was selected for its high discharge 45C continuous discharge rate, doubling to 90C at its peak, easily meeting our power requirements. The high discharge rate enables for quicker, consistent bursts of energy for acceration, preventing sag that could impact performance. Its light 90g design permitted for a quicker, more agile car while placing less strain on our chassis. Its 7.4V voltage is a perfect suit, jumping into a 11.1V battery may provide more speed, but also wears down components quicker, eventually destroying the part completly; longevity would be a great concern.
      </p>
      <h3>Real-World Notes</h3>
      <p>
        In practice, the battery delivered stable voltage under load without noticeable sag during acceleration. Its compact dimensions and light weight allowed for easy installation while simultaneously providing a significant supply of energy, giving us quicker and more decisive runs. However, its limited survivability proved drawback. In the process of downloading ROS2, the battery would frequently die over the 3 hour download time, so connecting the car to an outlet was the only solution. We feel that we could have sacrificed a bit of weight for some more power time. Charging was also an issue. The process to charge took a lengthy time, roughly 3 hours.
      </p>
    </td>
  </tr>
</table>

---

# Mobility Management

## Chassis 


For our build, we selected the **ACXWA CD chassis**, a non-prebuilt frame originally crafted with a modular design to provide adjustable lengths between 1/28 and 1/24 scale. During development, we redesigned the component connecting the front and rear sections to improve structural stability. This modification locked the chassis into a fixed 1/24 scale (16.2 cm). While adjustable lengths provide flexibility, they introduced structural weakness. Our fixed build removes this weak point, creating a rigid, non-sliding frame.

<p><strong>Where to Buy:</strong> <a href="https://www.aliexpress.com/item/1005007495175639.html?spm=a2g0o.order_list.order_list_main.11.48a11802NKINMb" target="_blank">Click Here</a></p>

#### Notable Features / Design Advantages

- **Cost-effective:** At only $27 CAD, the chassis allowed us to remain within budget, compared to typical pre-built chassis costing between $50 and $100+.
- **Motor stand included:** This reduced assembly time and ensured accurate motor alignment, minimizing drivetrain losses and gear misalignment.
- **High steering range:**  Front wheels pivoted significantly, enabling tight turns with less steering input.
- **Compact form factor:** The fixed 1/24 scale size aided a streamlined electronics layout (ESC, battery), reducing clutter and improving airflow for cooling.

####  Chassis Limitations  

Despite the benefits, the chassis presented several challenges that impacted performance and assembly:  

- **Tire traction:** The stock plastic tires offered almost no grip, which we noticed immediately during testing. On smooth surfaces like the game map, this resulted in uncontrolled wheelspin, unstable acceleration, and extended braking distances.
- **Steering imbalance:** We also observed that the front wheels did not pivot evenly, creating unbalanced turning angles. The turning was unpredicable and when navigating precise maneuvers, uneven steering made the car to control consistantly.

####  How We Overcame These Limitations  

- **Improved traction:** We replaced stock plastic tires with rubber-coated wheels. This simple change was crucial, acceleration was more controlled and the robot futhered the reliability of the car.
- **Balanced steering:** Designed and 3D-printed a custom steering system, succeeding our previous unequal one.


## Motor

A compact, high-RPM brushless motor chosen for its exceptional power-to-weight ratio, providing rapid acceleration and high top speed for competitive performance.

<table border="1" width="100%">
  <tr>
    <td width="40%" align="center">
      <img src="https://github.com/user-attachments/assets/d0dc1c9c-15fc-4828-bf2a-c64bc5550389" width="250" alt="Furitek Micro Komodo Motor" />
    </td>
    <td width="60%" valign="top">
      <h2>Furitek Micro Komodo 1212 3450KV Brushless Motor</h2>
      <ul>
        <li><strong>KV (rpm/V):</strong> 3450</li>
        <li><strong>No-load Current @10V:</strong> 0.7 A</li>
        <li><strong>Power:</strong> 120 W</li>
        <li><strong>Battery:</strong> 2–3S LiPo</li>
        <li><strong>Resistance:</strong> 0.16 Ω</li>
        <li><strong>Max Current:</strong> 10 A</li>
        <li><strong>Slot/Pole:</strong> 12</li>
        <li><strong>Motor Size:</strong> 15.5 × 20.6 mm</li>
        <li><strong>Shaft:</strong> 1.5 × 6 mm</li>
        <li><strong>Weight:</strong> 17.5 g</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td colspan="2">
      <h3>Why We Chose This Motor</h3>
      <p>
Since this was our first time in the competition, we wanted to focus on learning and keep costs low. Rather than buying a new motor, we decided to reuse an existing one, turning it into a learning opportunity. At the same time, we chose to redesign the drivetrain, not because the motor didn’t fit, but to personalize the robot and make it our own. The motor wasn’t the fastest nor was it the most powerful, but its 17.5 g weight kept the robot light, and 120 W of power drove the system reliably. Its 3450 KV rating provided enough speed, and the low internal resistance helped maintain efficiency. The compact size and 1.5 mm shaft line up with our gears, and the 12-slot stator with multipole rotor delivered smooth torque for better control. It wasn’t the “best” motor available, but reusing it let us learn, save money, and create a robot that reflected our ideas in our first competition.
      </p>
      <h3>Real-World Notes</h3>
      <p>
        Testing showed smooth acceleration with minimal heat buildup. Its compact design fit tightly in the chassis, reducing drivetrain losses and maintaining reliability through repeated runs. Although not our ideal pick, it was definitely not much of a hinderance. 
      </p>
    </td>
  </tr>
</table>

<p><strong>Where to Buy:</strong> <a href="https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118" target="_blank">Click Here</a></p>

## Electronic Speed Controller (ESC)  
The ESC is a key component that regulates how the motor receives power. It:  
- Controls the motor's speed  
- Manages direction  
- Ensures safe delivery of voltage and current to the motor  

<table border="1" width="100%" style="font-size:20px; text-align:left;">
  <tr> 
    <td width="50%" align="center" style="vertical-align:top;"> 
      <img src="https://github.com/user-attachments/assets/e42ffea8-167e-4d05-9c82-ca2e05dc2562" width="500" alt="Furitek Lizard Pro ESC" /> 
    </td> 
    <td width="50%" valign="top" style="vertical-align:top; font-size:20px;"> 
      <h2 style="font-size:28px;">Furitek Lizard Pro 30A/50A ESC</h2> 
      <ul>
        <li><strong>Battery Support:</strong> 2S–3S LiPo</li> 
        <li><strong>BEC:</strong> 5V or 6.5V, 2.5A</li> 
        <li><strong>Constant Current:</strong> 30A</li> 
        <li><strong>Burst Current:</strong> 50A</li> 
        <li><strong>Built-in Power Switch</strong></li> 
        <li><strong>Dimensions:</strong> 28 × 15.5 mm</li> 
        <li><strong>Weight:</strong> 3.7 g</li> 
      </ul> 
    </td> 
  </tr> 
</table>



---

## Steering
Our first prototype used a commercial chassis. However, we quickly identified that the steering system was inefficient and turned asymmetrically. To overcome this, we engineered a completely new steering system from the ground up, 3D-printing all components to achieve the symmetric and precise control we needed.

The design features a mounting frame that secures the system to the chassis, a servo linkage arm that connects directly to the servo motor, and steering knuckles that translate the servo’s rotation into precise, symmetrical wheel movement. This layout ensures consistent handling and greatly improves control compared to the original design.

<table>
  <tr>
    <td style="border: 200px solid black; padding: 5px;">
      <img width="1200" height="570" alt="Untitled design" 
           src="https://github.com/user-attachments/assets/839592d3-8859-4111-a7a7-856aeb2863e8" />
    </td>
  </tr>
</table>


#### Potential Improvements:
- Used a different material, PLA is brittle and PETG, ABS, or Nylon would've given more toughness and flexibility.
- Added more fillets on corners, reducing stress concentration and prevents cracks.
- Implemented brass threaded inserts for screw holes rather than relying on bare plastic.
- Could have made knuckles thicker to prevent wear from constant movement.

## Servo Motor

<table border="1" width="100%">
  <tr>
    <td width="40%" align="center">
      <img src="https://github.com/user-attachments/assets/56dbecd9-28f9-4358-8cb5-949740f3ea51" width="250" alt="HS-5055MG Digital Micro Servo" />
    </td>
    <td width="60%" valign="top">
      <h2>HS-5055MG 11.9g Metal Gear Digital Micro Servo</h2>
      <ul>
        <li><strong>Operating Voltage:</strong> 4.8V – 6.0V DC</li>
        <li><strong>Max Torque:</strong> 22 oz/in (1.6 kg/cm)</li>
        <li><strong>Speed:</strong> 0.17s/60° @ 6.0V</li>
        <li><strong>Stall Current:</strong> 700 mA</li>
        <li><strong>Gear Material:</strong> Metal</li>
        <li><strong>Weight:</strong> 9.5 g</li>
        <li><strong>Circuit Type:</strong> G1 Programmable Digital</li>
      </ul>
    </td>
  </tr>
  <tr>
    <td colspan="2">
      <h3>Why We Chose This Servo</h3>
      <p>
        The HS-5055MG was chosen for its strength, precision, and compact size. Its metal gears provide 
        durability for steering, and its programmable digital circuit ensures accurate control.
      </p>
      <h3>Real-World Notes</h3>
      <p>
        In practice, the servo delivered consistent steering with minimal backlash. It handled loads 
        even during sharp turns at speed, making it dependable for precision navigation.
      </p>
    </td>
  </tr>
</table>


#### How the Car Moves
When the battery supplies power, the ESC controls the amount of electricity funneled to the motor. The motor then converts this regulated flow of electricity into rotational motion, thereby spinning the wheels of the car. By adjusting the voltage and current through the ESC, we can control the car's speed and direction (forwards and backwards), allowing it to accelerate, slow down, or reverse in motion.

In parallel, the servo motor is responsible for steering. It adjusts the angle of the front wheels through the steering system, enabling the car to turn left or right with precision.

Together, the motor, ESC, servo motor, and chassis form the core of our car's mobility system, while the battery holds the energy.

# Sense Managment

# Software
## Frameworks
### Python 3.0
Python 3 is the core programming language used to build and run the autonomous car logic. Its simplicity, readability, and massive library pool make it ideal for controlling hardware like the Raspberry Pi. For robotics, Python makes it easy to interface with sensors and actuators, process images, and manage logic. However, Python is slower than compiled languages like C++, and because it’s dynamically typed, certain bugs may only appear at runtime if not carefully tested.

### ROS2 Humble 
ROS 2 (Robot Operating System 2) Humble is a modern, open-source robotics middleware designed to enable communication between nodes in robotic systems. It provides tools for message passing to aid with maximizing robot software efficiency. Although efficient, it is difficult and time consuming to incorporate a working ROS2 enviorment into your code. It is for this reason that only the obstacle challenge uses ROS 2 while the open challenge is done with a standalone python file.
<img width="955" height="443" alt="image" src="https://github.com/user-attachments/assets/541fb372-0a35-41a8-9da3-7a418b4bc60a" />


## Libraries
### Picamera2
Picamera2 is a library used to interface with the Raspberry Pi's camera module. It allows the robot to capture real-time photos and videos, which are critical for analyzing the robot environment. It provides full control over resolution, frame rate, and pixel format to effectively work in tandem with OpenCV.

### OpenCV
OpenCV (Open Source Computer Vision Library) is used to process camera frames, apply thresholding, and detect contours within specified regions of interests (ROIs). This data is used to determine the robots orientation and distance relative to walls and pillars to guide its movement throughout the course.

### Cv_bridge
cv_bridge is a ROS 2 library that converts between ROS Image messages and OpenCV images, enabling image processing with OpenCV in ROS-based applications.

### Time
The built-in Python time module is used for time-based operations, such as delays (sleep) or measuring time elapsed since an event.

### Numpy
NumPy is a fundamental Python library for numerical computation. In robotics, it's often used for handling arrays, matrices, and mathematical operations efficiently. In our code it is use for creating and handling arrays for color thresholding in HSV space

### ROS_ROBOT_CONTROLLER_SDK
This custom Python SDK (Software Development Kit) provided by HiWonder is responsible for controlling robot hardware, including steering servos, throttle motors, and RGB LEDs. It turns low-level control into simple methods, allowing the main code to remain clean.

## Open Challenge


## Obstacle Challenge
### Package structure

```
~/fe_ws/src/obstacle_challenge/
├── obstacle_challenge/
│   ├── __init__.py
│   ├── camera_node.py
│   ├── imu_node.py
│   ├── navigator_node.py
│   └── ros_robot_controller_sdk.py
├── launch/
│   └── obstacle_challenge.launch.py
├── package.xml
└── setup.py
```


<img width="1920" height="1080" alt="NAVIGATOR_NODE py" src="https://github.com/user-attachments/assets/adf65227-4afb-4aa7-91dd-885e6386cb29" />

### NODES/TOPICS


# Building Instructions:

## Motor Mechanism
### Step 1:
   Put belt and 12 mm ball berings onto the shaft
   ![Screenshot 2025-08-29 144137-fotor-20250829144459](https://github.com/user-attachments/assets/a2987109-9992-453f-a72f-bddd081fd541)

### Step 2:
   Insert 4 of the 6mm ball berings into holes, put shaft mechanism down ontop of 12mm ball bering holder
   <img width="1138" height="825" alt="image" src="https://github.com/user-attachments/assets/755b32c9-1db5-476a-bc96-4c083b12c792" />

### Step 3: 
   Put top half of motor mechanism on top of the bottem half
   <img width="867" height="629" alt="image" src="https://github.com/user-attachments/assets/437a7066-cc08-4258-9962-17ff6d62dabf" />

### Step 4:
   Screw in the top to the bottem upside down
   <img width="939" height="605" alt="Screenshot 2025-08-29 145919-fotor-202508291510" src="https://github.com/user-attachments/assets/067696be-cac3-4ae4-b99e-6b38ead02792" />
### Step 5: 
   Screw in acxwa motor mount to 2 high holes
   <img width="807" height="579" alt="image" src="https://github.com/user-attachments/assets/fa540000-e1f0-466f-8508-706494be5705" />

   






   

