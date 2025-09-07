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
4&#46; <a href="#robotcar-design-overview">Robot/Car Design Overview</a><br>
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

</div>



</div>




## Team Members

- Ryan Rao - 14
- Elvis Wang - 14
- Harry Xiao - 15

## Complete Construction Manual


The WRO Future Engineers is the competition of the self-driven vehicles. Students of the age group from 15 till 19 need to design a model of a car, equip it with electromechanical components and program it as so it will be able to autonomously drive on the track avoiding objections.

More details about the competition can be found on [the official site of WRO Association](https://wro-association.org/competition/2025-season/#rules).

<img width="700" alt="image" src="https://github.com/user-attachments/assets/51e5db22-617e-4d90-a60a-5c4dbe9ec88f" />


The materials are intended to be build with the [Mkdocs](https://www.mkdocs.org/) site generator.

The example of the site is accessible by https://world-robot-olympiad-association.github.io/future-engineers-gs/.



## Machinery 
### Components
| Name | Product | Price (CAD)|
| ----------- | ----------- | ----------- |
| RC Car | [`ACXWA CD`](https://www.aliexpress.com/item/1005007495175639.html?src=bing&aff_short_key=UneMJZVf&aff_platform=true&isdl=y&albch=shopping&acnt=135095331&isdl=y&albcp=555018171&albag=1299623888131540&slnk=&trgt=pla-4584826057944442&plac=&crea=81226548307861&netw=o&device=c&mtctp=e&utm_source=Bing&utm_medium=shopping&utm_campaign=PA_Bing_CA_PLA_PC_Hot-Sale_MaxValue_20240715&utm_content=Hot%20sale&utm_term=1%2F28%20car%20chassie&msclkid=b27792305716194d30fcf5a6c0fef479)  | $27 |
| RC Car Battery | [`Gens Ace 1300mAh Battery`](https://www.adrenalinehobby.c1om/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug) | $35 | 
| Drive Motor | [`Furitek Micro Komodo 1212 3450KV Brushless Motor`](https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118) | $35 |
| Servo Motor | [`HS-5055MG 11.9g Metal Gear Digital Micro Servo`](https://hitecrcd.com/hs-5055mg-economy-metal-gear-feather-servo/?srsltid=AfmBOooq_9U4Nehv90Y-tGWqZeo6_1c0_7imuMD9W_dBJmYS1m0sd2Y_) | $25 |
| ESC | [`Furitek Lizard Pro 30A/50A ESC`](https://furitek.com/products/combo-of-furitek-lizard-pro-30a-50a-brushed-brushless-esc-for-axial-scx24-with-bluetooth). | $80 |
| Camera | [`5MP 1080P HD Camera with OV5647 Sensor`](https://www.amazon.ca/dp/B0D324RKRZ?ref=ppx_yo2ov_dt_b_fed_asin_title) | $35 |
| Raspberry Pi 5 | [`Raspberry Pi 5`](https://www.amazon.ca/Vemico-Raspberry-Kit-Heatsinks-Screwdriver/dp/B09WXRCYL4/ref=sr_1_3) | $180 |
| Expansion Board | [`RRC Lite Controller`](https://www.hiwonder.com/products/rrc-lite?srsltid=AfmBOoqZuQkdiCruulYju-KXoSowMik5Ov_Vs3-_8TA4Bm_luvoK6Oxn). | $40 |
| Lidar | [`LDROBOT D500 lidar kit TOF laser Radar Lidar Scanner `](https://www.aliexpress.com/item/1005003012681021.html?spm=a2g0o.order_list.order_list_main.11.7a3b18028WK12R). | $82 |

Total : $546


##  Robot/Car Design Overview 
A majority of the car was 3D printed or replaced.
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

<p><strong>Where to Buy:</strong> <a href="https://www.aliexpress.com/item/1005007495175639.html?spm=a2g0o.order_list.order_list_main.11.48a11802NKINMb" target="_blank">Click Here</a></p>

#### Potential Improvements
- Faster charge rate to reduce downtime.
- Longer survivabilty. Sacrifice some weight for a higher mAh, equating to a longer lasting battery.
- Enhanced safety features such as overcharge. One of our batteries broke due to overcharge.
  
We added a velcro strip to both our battery and the base of our chassis, allowing our battery to sit securely on the bottom of our car.

### Power Ratings Table
| Component | Voltage | Normal Current Draw | Max Current Draw | Normal Power | Max Power |
|-|-|-|-|-|-|
| 5MP 1080P HD Camera         | 5 V     | 0.16 A   | 0.20 A | 0.80 W  | 1.00 W  |
| RRC Lite Controller         | 5 V     | 0.07 A   | 0.50 A | 0.35 W  | 2.50 W  |
| Furitek Lizard Pro ESC      | 7.4 V   | 0.007 A  | 0.10 A | 0.05 W  | 0.74 W  |
| Furitek Micro Komodo Motor  | 7.4 V   | 0.95 A   | 10.0 A | 7.00 W  | 74.0 W  |
| HS-5055MG Servo Motor       | 5 V     | 0.10 A   | 0.70 A | 0.50 W  | 3.50 W  |
| LDROBOT D500 Lidar          | 5 V     | 0.12 A   | 0.35 A | 0.60 W  | 1.75 W  |
| MicroSD, LEDs, Speakers     | 5 V     | 0.12 A   | 0.30 A | 0.60 W  | 1.50 W  |
| Raspberry Pi 5              | 5 V     | 0.55 A   | 2.00 A | 2.75 W  | 10.0 W  |
| Expansion Board             | 5 V     | 0.12 A   | 0.50 A | 0.60 W  | 2.50 W  |
| Totals                      | —       | —        | —      | ~13 W   | ~97 W   |


The **Gens Ace 1300 mAh 2S 7.4 V LiPo Battery** offers plenty of power for our car, as shown in the Power Ratings Table. Normally, the car uses about 13 W, which is about 1.7 A from the battery. The car typically draws around 4 W (≈0.5 A) and in short bursts can reach 97 W (≈13 A). The 7.4 V 1300 mAh LiPo has a 45C continuous and 90C peak rating, meaning it can safely supply up to 58.5 A continuously and 117 A in short bursts, far more than the car will ever require. This extra capacity allows the battery to provide stable voltage for the Raspberry Pi, controller, sensors, and peripherals.

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

- **Improved traction:** We replaced stock plastic tires with rubber-coated wheels. This simple change was crucial, acceleration was more controlled and futhered the reliability of the car.
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
Since this was our first time in the competition, we wanted to focus on learning and keep costs low. Rather than buying a new motor, we decided to reuse an existing one, turning it into a learning opportunity. At the same time, we chose to redesign the drivetrain, not because the motor didn’t fit, but to personalize the car and make it our own. The motor wasn’t the fastest nor was it the most powerful, but its 17.5 g weight kept the car light, and 120 W of power drove the system reliably. Its 3450 KV rating provided enough speed, and the low internal resistance helped maintain efficiency. The compact size and the 12-slot stator with multipole rotor delivered smooth torque for better control. It wasn’t the “best” motor available, but reusing it let us learn, save money, and create a car that reflected our ideas in our first competition.
      </p>
      <h3>Real-World Notes</h3>
      <p>
        Testing showed smooth acceleration with minimal heat buildup. Its compact design fit tightly in the chassis, reducing drivetrain losses and maintaining reliability through repeated runs. Although not our ideal pick, it was definitely not much of a hinderance. 
      </p>
    </td>
  </tr>
</table>

<p><strong>Where to Buy:</strong> <a href="https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118" target="_blank">Click Here</a></p>

#### Potential Improvements
- Although the 3450 KV rating is quite fast, this speed sacrifices smooth, controlled rotation at a lower RPM. When we reduce RPM for tight turns, the motor’s choppy rotation slows us down and reduces effectiveness.
- Improve quality. Under repeated high-load or high-speed operation, the motors components will degrade faster than higher quality ones.
  

## Electronic Speed Controller (ESC)  
The ESC is a key component that regulates how the motor receives power. It:  
- Controls the motor's speed  
- Manages direction  
- Ensures safe delivery of voltage and current to the motor

This ESC was chosen for its high current capacity, lightweight design, and reliable built-in BEC, making it well-suited for our motor and servo needs.

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
This process put us against tough challenges. In early versions, we found it difficult to make the system truly balanced. Even small errors in 3D design or flaws in the printing process led to slight misalignments. These small mistakes resulted in noticeable steering inconsistencies. We had to do multiple redesigns and reprints. By iterating, tightening tolerances, and refining our 3D models, we gradually removed these errors and created a system that was both precise and reliable.

The new design features a mounting frame that secures the system to the chassis, a servo linkage arm that connects directly to the servo motor, and steering knuckles that translate the servo’s rotation into precise, symmetrical wheel movement. This layout ensures consistent handling and greatly improves control compared to the original design.

<table>
  <tr>
    <td style="border: 200px solid black; padding: 5px;">
      <img width="1200" height="570" alt="Untitled design" 
           src="https://github.com/user-attachments/assets/839592d3-8859-4111-a7a7-856aeb2863e8" />
    </td>
  </tr>
</table>

### Key Improvements Over Stock Design
1. **Symmetry in Motion** – Both left and right wheels now turn at equal angles, reducing understeer/oversteer imbalance.  
2. **Reduced Backlash** – By using strong linkages and tightly toleranced 3D-printed parts, uncontrolled movement is minimized.  
3. **Strength & Durability** – The mounting frame distributes servo torque evenly, reducing wear and preventing chassis flex.  
4. **Precision Control** – The redesigned geometry allows finer servo adjustments to directly translate into steering corrections.  

### Manufacturing & Assembly
- All parts were modeled in CAD and 3D-printed using high-strength PLA fillament.  
- Stainless steel screws and ball bearings were used in the knuckles to reduce friction and extend part life.  
- The servo linkage was reinforced with a metal rod to prevent bending under load.  

#### Potential Improvements:
- Use a different material, PLA is brittle and PETG, ABS, or Nylon would've given more toughness and flexibility.
- Adde more fillets on corners, reducing stress concentration and prevents cracks.
- Implement brass threaded inserts for screw holes rather than relying on bare plastic.


## Servo Motor

This servo motor was chosen for its compact size, strong torque, and durable metal gears, making it great for precise steering control.

<table border="1" width="100%" style="font-size:20px;">
  <tr>
    <td width="50%" align="center" style="vertical-align:top;">
      <img src="https://github.com/user-attachments/assets/56dbecd9-28f9-4358-8cb5-949740f3ea51" width="400" alt="HS-5055MG Digital Micro Servo" />
    </td>
    <td width="50%" valign="top" style="vertical-align:top; font-size:20px;">
      <h2 style="font-size:26px;">HS-5055MG 11.9g Metal Gear Digital Micro Servo</h2>
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
</table>

#### Potential Improvements
- Improve response time. This servo motor is not as quick as higher-end micro servos, which reduces steering precision during sharp turns.
- Upgrade to a higher-torque digital micro servo for more reliable steering.
  
#### How the Car Moves
When the battery supplies power, the ESC controls the amount of electricity funneled to the motor. The motor then converts this regulated flow of electricity into rotational motion, thereby spinning the wheels of the car. By adjusting the voltage and current through the ESC, we can control the car's speed and direction (forwards and backwards), allowing it to accelerate, slow down, or reverse in motion.

In parallel, the servo motor is responsible for steering. It adjusts the angle of the front wheels through the steering system, enabling the car to turn left or right with precision.

Together, the motor, ESC, servo motor, and chassis form the core of our car's mobility system, while the battery holds the energy.

# Sense Management

The car relies on various sensors to understand its surroundings and interact safely with the environment. Sense management refers to how these inputs are coordinated, processed, and used for decision-making. Instead of treating each sensor independently, we designed a system that combines all data into a single model of the world. 

At the heart of sense management is the idea of prioritization. Different sensors have different strengths, some are better at detecting precise distances, while others at identifying shapes or movement. By assigning specific roles to each sensor and merging their data, the car maintains a reliable awareness of its environment, even as conditions change.

## Camera
This camera was chosen for its wide 175° field of view, compact size, and 5MP resolution, making it ideal for real-time vision processing and object detection on our car.  

<table border="1" width="100%" style="font-size:20px; text-align:left;">
  <tr> 
    <td width="50%" align="center" style="vertical-align:top;"> 
      <img width="500 alt="image" src="https://github.com/user-attachments/assets/f60b01c2-a8af-488a-9bf0-2b0dda0ea1aa" />
    </td> 
    <td width="50%" valign="top" style="vertical-align:top; font-size:20px;"> 
      <h2 style="font-size:28px;">5/Zero Camera Module (OV5647 Sensor)</h2> 
      <ul>
        <li><strong>Lens Pixel:</strong> 5 MP</li> 
        <li><strong>Resolution:</strong> 2592 × 1944</li> 
        <li><strong>Lens Angle:</strong> 175° Wide Angle</li> 
        <li><strong>Lens Focal Length:</strong> 3.6 mm</li> 
        <li><strong>Focus Mode:</strong> Manual</li> 
        <li><strong>CMOS Size:</strong> 1/2.5 inch</li> 
        <li><strong>Material:</strong> ABS + Optical Glass</li> 
        <li><strong>Screw Model:</strong> M2 × 6</li> 
        <li><strong>Cable:</strong> 15 cm Ribbon Cable</li> 
      </ul> 
    </td> 
  </tr> 
</table>

The wide-angle camera serves as the car’s primary tool for visual detection. With its 175° lens, the camera captures almost the entire forward field of view, reducing blind spots and enabling the system to track multiple objects simultaneously. This feature proves particularly useful for tasks like pillar and wall detection, identifying obstacles ahead, and observing changes in the environment.

We process the camera feed on the Raspberry Pi using computer programs. This setup allows the car to not only notice when something is in its path but also to identify what it is, such as open passages, furniture, people, or in this case, walls and pillars. This understanding gives the car a significant edge over systems that depend solely on distance sensors.

#### Potential Improvements for Camera 

While the current Pi Camera and LiDAR provide a good amount sensing for our obstacle challenge, several improvements could improve performance.

**Camera (5MP Pi Camera w/ OV5647)**
- Upgrade to a higher resolution or global-shutter sensor (e.g., HQ Camera IMX477) for sharper images and less motion blur.  
- Add auto exposure and white balance adjustments to handle changing lighting conditions.    
- Explore infrared capability with an IR-sensitive camera and IR light source for consistent performance regardless of ambient lighting.  
- Apply filtering in software (e.g., erosion/dilation in OpenCV) or use a moving average/Kalman filter to stabilize pillar tracking.


## Inertial Measurement Unit (IMU)

While the camera can simultaneously identify different items and surroundings, understanding the car’s own motion and orientation is equally crucial. This is where the IMU comes in. The Inertial Measurement Unit tracks the car’s acceleration, angular velocity, and orientation in real-time, allowing the car to know exactly how it is moving through space. 

<table border="1" width="100%" style="font-size:20px;">
  <tr>
    <td width="50%" align="center" style="vertical-align:top;">
      <img width="600" alt="image" src="https://github.com/user-attachments/assets/b6fa3cf6-a8aa-4aaa-ab9d-8c2424c89b00" />
    </td>
    <td width="50%" valign="top" style="vertical-align:top; font-size:20px;">
      <h2 style="font-size:26px;">MPU-6050 6-Axis IMU</h2>
      <ul>
        <li><strong>Sensor Type:</strong> 3-axis Accelerometer + 3-axis Gyroscope</li>
        <li><strong>Accelerometer Range:</strong> ±16g</li>
        <li><strong>Gyroscope Range:</strong>  ±2000°/s</li>
        <li><strong>Resolution:</strong> 16-bit</li>
        <li><strong>Interface:</strong> I²C (up to 400 kHz)</li>
        <li><strong>Supply Voltage:</strong> 3.3V – 5V</li>
        <li><strong>Features:</strong> On-chip DMP, auxiliary I²C bus, built-in temperature sensor, shock tolerant</li>
        <li><strong>Package:</strong> 4×4×0.9 mm QFN</li>
      </ul>
    </td>
  </tr>
</table>

By combining the IMU with the camera and LiDAR, the car can merge multiple data sources to create a clear view of its surroundings. The IMU tracks motion between LiDAR scans and camera frames, which improves mapping and object tracking. For instance, if the car’s camera spots a wall or pillar, the IMU makes sure the system knows if the car is moving forward, turning, or tilting as it nears the obstacle.

In short, the IMU serves as the car’s sense of balance and movement. It works alongside the external perception from the camera and LiDAR. This teamwork allows the autonomous system to make better decisions, navigate safely, and maintain accurate control at all times.

## Sensor Consideration/Potential Improvements
### Lidar 

This LiDAR is good for its long-range accuracy, compact design, and robust scanning capability, making it ideal for mapping and obstacle detection on our car.  

<table border="1" width="100%" style="font-size:20px; text-align:left;">
  <tr> 
    <td width="50%" align="center" style="vertical-align:top;"> 
      <img src="https://github.com/user-attachments/assets/9657e574-9164-4301-acbb-571c49af60e9" width="500" alt="LDROBOT D500 LiDAR" /> 
    </td> 
    <td width="50%" valign="top" style="vertical-align:top; font-size:20px;"> 
      <h2 style="font-size:28px;">LDROBOT D500 LiDAR</h2> 
      <ul>
        <li><strong>Ranging Distance:</strong> 0.03 – 12 m</li> 
        <li><strong>Accuracy:</strong> ±10 mm (0.3–0.5 m), ±20 mm (0.5–2 m), ±30 mm (2–12 m)</li> 
        <li><strong>Scanning Angle:</strong> 360°</li> 
        <li><strong>Scanning Frequency:</strong> 6 – 13 Hz (Typ. 10 Hz)</li> 
        <li><strong>Ranging Frequency:</strong> 5000 Hz</li> 
        <li><strong>Wavelength:</strong> 895 – 915 nm (Typ. 905 nm)</li> 
        <li><strong>Interface:</strong> UART @ 230400 baud</li> 
        <li><strong>Ambient Light Tolerance:</strong> up to 60K Lux</li> 
        <li><strong>Power Supply:</strong> 5 V</li> 
        <li><strong>Power Consumption:</strong> 1.45 W (290 mA)</li> 
        <li><strong>Operating Temperature:</strong> -10 ~ 45 °C</li> 
        <li><strong>Dimensions:</strong> 54 × 46.3 × 35 mm</li> 
        <li><strong>Weight:</strong> 45 g</li> 
      </ul> 
    </td> 
  </tr> 
</table>

The LiDAR is the car’s most reliable tool for precise distance measurement and mapping. Unlike a camera, which depends on good lighting and struggles to estimate exact distances, the LiDAR provides accurate range data in every direction, up to 12 meters away. This gives the car a real-time 2D map of obstacles and open space. 

You can use this data for path planning and collision avoidance. As the LiDAR spins, it continuously creates a profile of the environment. The car can then identify safe routes, avoid collisions, and even build lasting maps of the areas it explores. This is essential for autonomous navigation because it ensures the car always knows how much space it has to move, no matter the lighting or background conditions.

Another benefit of LiDAR is its stability in changing environments. If a person or another car moves into its path, the LiDAR quickly detects the change and updates the map. This makes the system very responsive and safe for real-world use. By relying on geometry rather than appearance, LiDAR supports the camera and offers a reliability that vision alone cannot achieve.

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
### Overview
The Open Challenge is designed to test a robot’s ability to autonomously navigate a closed-loop course using only the black walls as guidance. The robot must detect walls, align itself, make smooth turns at corners, and complete 3 laps without manual intervention in under 3 minutes. The main focus is on path-following and code accuracy.

### Difficulties
In the Open Challenge, the course can be set up in either a wide(100cm) or a narrow(60cm) configuration, and each comes with its own difficulties for vision and navigation. 
In the wide setup, the black walls are spaced much farther apart. This makes the robot’s region of interest (ROI) readings weaker because the walls occupy a smaller portion of the camera frame.
The narrow configuration creates the opposite problem. With the walls placed close together, the black regions fill a large portion of the ROIs, making the system very sensitive.

To be successful, the code must be consistent enough to be able to navigate through both scenarios, which each present conflicting issues. Fixing an error in the wide setup may break something in the narrow setup and vice versa.

### Our solution
The primary means of navigation in the open challenge lies in the camera. The camera captures frames multiple times a second and performs wall following logic with information inside the ROIs. ROIs or region of interests are small rectangles placed strategically in areas of interest, to perform wall detection. The open challenge includes 3 of these ROIs initialized at the top

The left and right ROIs are each placed on the edge of their respective sides. They are essential for the detecton of differences in wall size to do PD steering and detect turn segments. 
The orange ROI is a thin rectangular box, centered near the bottom of the frame. It is is used to detect the orange line on the ground, detecting turn areas for lap counting only.

In order to begin camera detection we must define our ROIs, initialize the camera and define the hsv range for orange:

```
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
time.sleep(1)

lower_orange = np.array([5, 100, 100])
upper_orange = np.array([20, 255, 255])

# ---- Define ROIs ----
left_roi = (0, 200, 180, 180) # x, y, w, l
right_roi = (460, 200, 180, 180)
orange_roi = (100, 360, 440, 40) 
```   

Now, we can apply masks, crop ROIs, and count pixels

```
# ---- Get camera frame ----
frame = picam2.capture_array()
x, y, w, h = orange_roi
roi_crop = frame[y:y+h, x:x+w]

# Convert to HSV for color detection
hsv = cv2.cvtColor(roi_crop, cv2.COLOR_BGR2HSV)
mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)

# Count orange pixels
orange_pixel_count = cv2.countNonZero(mask_orange)

if orange_pixel_count >= 500:  
    current_time = time.time()
    if current_time - last_orange_time > orange_cooldown:
        turns += 1

# Thresholding
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
_, thresh = cv2.threshold(gray, 110, 255, cv2.THRESH_BINARY_INV)

mask = np.zeros_like(thresh)
cv2.rectangle(mask, (left_roi[0], left_roi[1]), (left_roi[0]+left_roi[2], left_roi[1]+left_roi[3]), 255, -1)
cv2.rectangle(mask, (right_roi[0], right_roi[1]), (right_roi[0]+right_roi[2], right_roi[1]+right_roi[3]), 255, -1)
masked = cv2.bitwise_and(thresh, mask)

contours, _ = cv2.findContours(masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Crop ROIs
left_crop = thresh[left_roi[1]:left_roi[1]+left_roi[3], left_roi[0]:left_roi[0]+left_roi[2]]
right_crop = thresh[right_roi[1]:right_roi[1]+right_roi[3], right_roi[0]:right_roi[0]+right_roi[2]]

# Count black pixels
left_area = cv2.countNonZero(left_crop)
right_area = cv2.countNonZero(right_crop)
```

The purpose of this code is the generate us 3 pieces of information, amount of black pixels in the left roi, the amount of black pixels in the right roi, and the amount of orange pixels in the orange roi. We can then use this information to steer our robot.

Most of the time, the robot will be in a straight section where it will use PD steering to avoid walls. PD steering is a system used to correct the robots movements so that it is centered between the two walls. 

The proportional term(P) is simply how far off you are from the center.
You compute an error with:
```
error = right_area - left_area
```
If the robot sees more black on the right, error is positive steer left.
If it sees more black on the left, error is negative steer right.

Then you apply:
```
steering = straight_pwm + kp * error
```
where kp is a constant gain. This makes the robot steer proportionally to how far it’s off-center.

the derivative term looks at how fast the error is changing.
If the error is quickly swinging, it means the robot is wobbling. The derivative dampens this by applying correction against sudden changes:
```
derivative = error - prev_error
steering += kd * derivative
```

where kd is another constant.
This helps smooth out steering and prevents oscillation (zig-zagging).

Our code utilizes PD steering like this
```
area_diff = right_area - left_area
angle_pwm = int(STRAIGHT_PWM + area_diff * KP + (area_diff - prev_diff) * KD)
```
Here:
area_diff = error.
straight_pwm = neutral steering.
kp = how much to react to a error.
kd = how much to stabilize a change in turning angle
(area_diff - prev_diff) is the derivative.

When a substantial part of one wall goes missing out of nowhere we detect a turn.
Initially we had 2 variables control turning
```
TURN_THRESHOLD = 3000
EXIT_THRESHOLD = 9500
```

If the pixel area on one side dropped below the turn threshold it would initialize a turn. 

```
if left_area <= TURN_THRESHOLD and not right_turn:
    left_turn = True
elif right_area <= TURN_THRESHOLD and not left_turn:
    right_turn = True
```
Then to detect a turn exit:
```
if left_turn or right_turn:
    if (right_area >= EXIT_THRESHOLD and right_turn) or (left_area >= EXIT_THRESHOLD and left_turn):
        current_time = time.time()
        if current_time - last_turn_time >= 1.4:
            left_turn = right_turn = False
            board.set_rgb([[1, 0, 255, 0]])
            board.set_rgb([[2, 0, 255, 0]])
            last_turn_time = current_time
            prev_diff = 0
            print(f"Turn complete. Segments = {turns}")
```
However, the difference in wall distance scenarios, caused inconsistencies in turn exits.

To combat this we got rid of the exit threshold, instead opting for a proportional turn exit.
```
if right_area > left_area * 1.8:  # Right has 80% more black than left
            current_time = time.time()
            if current_time - last_turn_time >= 1.2:
                left_turn = False          
elif right_turn:
    if left_area > right_area * 1.8:  # Left has 80% more black than right
        current_time = time.time()
        if current_time - last_turn_time >= 1.2:
            right_turn = False
```
Finally, we turn:
```
elif left_turn:
    angle_pwm = min(max(angle_pwm, STRAIGHT_PWM + TURN_DEV), MAX_LEFT)
elif right_turn:
    angle_pwm = max(min(angle_pwm, STRAIGHT_PWM - TURN_DEV), MAX_RIGHT)
else: # Clamp
  angle_pwm = max(min(angle_pwm, MAX_LEFT), MAX_RIGHT)
```
We need to apply the clamp to make sure the steering angle does not surpass the predefined maximums.

The car will continue running all this code in a while loop until it detects 12 oranges lines:

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
│   └── obstacle_challenge_launch.py
├── package.xml
└── setup.py
```

<img width="1920" height="1080" alt="NAVIGATOR_NODE py" src="https://github.com/user-attachments/assets/94e4d3b1-762c-4baa-82bd-70e4aa4d126b" />

### NODES/TOPICS
#### navigator_node.py
- Core decision making node
- Subscribes to /image_raw from camera_node.py
- Subscribes to /imu_angle from imu_node.py
- Subscribes to /lap_status from imu_node.py
- Subscribes to /ros_robot_controller/button
- Publishes to /state to communicate to other nodes
  Combines camera and IMU data to output driving decisions (turning, avoiding, lap counting, parking).

#### camera_node.py
- Captures frames from Raspberry Pi's camera
- Subscribes to /state to know what to do
- Publishes to /image_raw to transmit camera information
  Provides visual input for pillar and wall detection


#### imu_node.py
- Reads IMU sensor value and transmits important information
- Subscribes to /imu/rpy/filtered for filtered IMU data
- Publishes to /imu_angle to transmit current IMU angle
- Publishes to /lap_status to transmit lap information
  Provides orientational information for lap counting and navigation



   






   

