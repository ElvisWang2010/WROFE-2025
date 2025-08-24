# Engineering Documentation

This repository documents Team Buzzy Bee's building and programming process.


## Team Members

- Ryan Rao - 14
- Elvis Wang - 14
- Harry Xiao - 15

## Machinery 
### Components
| Name | Product | Price (CAD)|
| ----------- | ----------- | ----------- |
| RC Car | [`ACXWA CD`](https://www.aliexpress.com/item/1005007495175639.html?spm=a2g0o.order_list.order_list_main.11.48a11802NKINMb)  | $27 |
| RC Car Battery | [`Gens Ace 1300mAh Battery`](https://www.adrenalinehobby.c1om/products/gens-ace-g-tech-1300mah-2s-7-4v-25c-lipo-deans-plug) | $35 | 
| Drive Motor | [`Furitek Micro Komodo 1212 3450KV Brushless Motor`](https://furitek.com/products/furitek-micro-komodo-1212-3456kv-brushless-motor-with-15t-steel-pinion-for-fury-wagon-fx118) | $39 |
| ESC | [`Furitek Lizard Pro 30A/50A ESC`](https://furitek.com/products/combo-of-furitek-lizard-pro-30a-50a-brushed-brushless-esc-for-axial-scx24-with-bluetooth). | $90 |
| Turning Motor | [`Hitec HS-5055MG Servo Motor`](https://ca.robotshop.com/products/hs-5055mg-metal-gear-micro-servo-motor?srsltid=AfmBOopv8Z7LoCVOEqe16w05ZV-R78dNmy7dappldIxZiQzCJroxcssFc2Y) | $38 |
| CSI Camera | [`SainSmart Wide Angle Fish-Eye Camera`](https://www.amazon.ca/SainSmart-Fish-Eye-Camera-Raspberry-Arduino/dp/B00N1YJKFS/ref=sr_1_5) | $19 |
| Raspberry Pi 5 | [`Raspberry Pi 5`](https://www.amazon.ca/Vemico-Raspberry-Kit-Heatsinks-Screwdriver/dp/B09WXRCYL4/ref=sr_1_3) | $180 |
| Expansion Board | [`RRC Lite Controller`](https://www.hiwonder.com/products/rrc-lite?srsltid=AfmBOoqZuQkdiCruulYju-KXoSowMik5Ov_Vs3-_8TA4Bm_luvoK6Oxn). | $45 |


# Component Structure

## Motor


<table>
  <tr>
    <td width="500" valign="top" style="border:1px solid #ddd; padding:10px;">
      <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/d0dc1c9c-15fc-4828-bf2a-c64bc5550389" />
    </td>
    <td width="500" valign="top" style="border:1px solid #ddd; padding:10px;">
      <h2>Specifications:</h2>
      <h3>
      • KV (rpm/V): 3450<br>
      • No-load current @10 V: 0.7 A<br>
      • Power: 120 W<br>
      • Battery: 2–3S<br>
      • Resistance: 0.16 Ω<br>
      • Max current: 10 A<br>
      • Slot/Pole: 12<br>
      • Motor size (mm): 15.5 × 20.6<br>
      • Shaft (mm): 1.5 × 6<br>
      • Weight: 17.5 g<br>
      </h3>
    </td>
  </tr>
</table><table>
  <tr>
    <td width="500" valign="top" style="border:1px solid #ddd; padding:10px;">
      <img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/d0dc1c9c-15fc-4828-bf2a-c64bc5550389" />
    </td>
    <td width="500" valign="top" style="border:1px solid #ddd; padding:10px;">
      <h2>Specifications:</h2>
      <h3>
      • Voltage: 10 V <br>
      • Watt <br>
      • Speed: 3450 rpm <br>
      • Resistance: 0.16 Ω <br>
      • Weight: 17.5 g
      </h3>
    </td>
  </tr>
</table>

### Chassis 


For our build, we selected the **ACXWA CD chassis**, a non-prebuilt frame originally crafted with a modular design to provide adjustable lengths between 1/28 and 1/24 scale. During development, we redesigned the component connecting the front and rear sections to improve structural stability. This modification locked the chassis into a fixed 1/24 scale (16.2 cm). While adjustable lengths provide flexibility, they introduce structural weakness. Our fixed build removes this weak point, creating a rigid, non-sliding frame.

### Notable Features / Design Advantages

- **Cost-effective:** At only **$27 CAD**, the chassis allowed us to remain within budget, compared to typical pre-built chassis costing between **$50 and $100+**.
- **Motor stand included:** This reduced assembly time and ensured accurate motor alignment, minimizing drivetrain losses and gear misalignment.
- **High steering range:** Tested front wheels pivoted significantly, enabling tight turns with less steering input—ideal for technical track layouts.
- **Compact form factor:** The fixed 1/24 scale size facilitated a streamlined electronics layout (ESC, battery), reducing clutter and improving airflow for cooling.

### Chassis Limitations

⚠️ Despite these benefits, the chassis presented several challenges that impacted performance and assembly:

- **Tire traction:** The frictionless plastic tires led to frequent wheelspin and poor grip, especially on smooth surfaces like the game map. This caused unstable acceleration and longer braking distances, reducing control at higher speeds.
- **Steering imbalance:** Asymmetric turning angles made consistent driving difficult and required software compensation.
- **Material flex:** The plastic frame flexed under pressure at screw joints, potentially affecting long-term durability.
- **Assembly complexity:** Non-prebuilt design required manual hole adjustments, and instructions in a foreign language slowed assembly and increased the chance of errors.

Additionally, the steering system displayed uneven turning angles: the car turned more sharply in one direction than the other. This imbalance made it difficult to drive in straight lines or execute consistent turns, and could not be easily corrected due to the limited adjustability of the steering system. The overall material quality of the chassis also presented concerns. The plastic frame flexed slightly under pressure, particularly at the screw joints, which could compromise long-term durability. Furthermore, although the chassis came with a motor stand, the non-prebuilt nature of the chassis made the assembly process slow and unintuitive. Certain holes required manual widening to fit screws properly, and the instructions were in a different language, which increased the chance of incorrect assembly. As a result, while the chassis served its purpose, it demanded extra effort and compensations to overcome its inherent flaws.

## Design
Most of the car was 3D printed or replaced, for example, the turning mechanism and motor mechanism from either of poor quality or not durable. The board underneath the car was not 3D printed, as it was made from carbon fiber and is more durable and lighter compared to printed part.

# 📷 Robot Design Overview  

## 1. Camera, ESC & Raspberry Pi 5 Holder  
![Camera, ESC, Raspberry Pi 5 Holder](https://github.com/user-attachments/assets/f4a32aea-952e-464d-8d2d-4e37fc32044b)  
This module securely mounts the Raspberry Pi 5, Electronic Speed Controller (ESC), and the camera, keeping wiring compact and accessible.  

---

## 2. Servo & Controller Holder  
![Servo and Controller Holder](https://github.com/user-attachments/assets/a5852082-f2c6-43ce-b2e2-41a676bc3e9a)  
Holds the servo and its controller in alignment, ensuring stable mounting and reliable operation.  

---

## 3. Turning Mechanism  
![Turning Mechanism](https://github.com/user-attachments/assets/1c6ecc1b-0e13-4adc-8c61-39562bed50f1)  
Mechanical system for steering/rotation, linked to the servo mount.  

---

## 4. Motor Mechanism  
![Motor Mechanism](https://github.com/user-attachments/assets/d691f26d-0096-4c3b-9ac7-1ecb6748c1cb)  
Houses the drive motor and transmits torque to the wheels/tracks.  

---

## 5. Servo Mount  
![Servo Mount](https://github.com/user-attachments/assets/b30877e6-f7ff-4926-92d1-63a098a3b12e)  
Dedicated mount for the servo, ensuring correct angle alignment and stability under load.  

---

## 6. Full Assembly  
![All Together](https://github.com/user-attachments/assets/5bd7f71e-6008-450d-a71e-d0bd98a7e613)  
Final integrated system showing all modules combined into the complete build.  


## Motor


## Power Resource Management
### Battery
Our RC car gets its power from a single `Gens Ace 1300mAh 7.4V Battery`. We chose this battery primarily for one reason: its high discharge rate. The 45C discharge rate allows for quick bursts of power, making it suitable for demanding applications despite the battery's being compact and lightweight. This powerful battery easily supplies the power demands of all of the components of our car, requiring a voltage regulator that resides in our Pi HAT to power our `Raspberry Pi 4B`, running on 5V.

## Software
### Python 3.0
Python 3 is the core programming language used to build and run the autonomous car logic. Its simplicity, readability, and massive library pool make it ideal for controlling hardware like the Raspberry Pi. For robotics, Python makes it easy to interface with sensors and actuators, process images, and manage logic. However, Python is slower than compiled languages like C++, and because it’s dynamically typed, certain bugs may only appear at runtime if not carefully tested.

### Picamera2
Picamera2 is a library used to interface with the Raspberry Pi's camera module. It allows the robot to capture real-time photos and videos, which are critical for analyzing the robot environment. It provides full control over resolution, frame rate, and pixel format to effectively work in tandem with OpenCV.

### OpenCV
OpenCV (Open Source Computer Vision Library) is used to process camera frames, apply thresholding, and detect contours within specified regions of interests (ROIs). This data is used to determine the robots orientation and distance relative to walls to guide its movement throughout the course.

### ROS_ROBOT_CONTROLLER_SDK
This custom Python SDK (Software Development Kit) provided by HiWonder is responsible for controlling robot hardware, including steering servos, throttle motors, and RGB LEDs. It turns low-level control into simple methods, allowing the main code to remain clean.

### RPI.GPIO
The RPi.GPIO library is used to read inputs from the Raspberry Pi’s GPIO pins, particularly for detecting button presses.

## Complete Construction Manual


The WRO Future Engineers is the competition of the self-driven vehicles. Students of the age group from 15 till 19 need to design a model of a car, equip it with electromechanical components and program it as so it will be able to autonomously drive on the track avoiding objections.

More details about the competition can be found on [the official site of WRO Association](https://wro-association.org/competition/new-competition-formats/future-engineers).

<img src="docs/img/fe-map.png" width="640">

The materials are intended to be build with the [Mkdocs](https://www.mkdocs.org/) site generator.

The example of the site is accessible by https://world-robot-olympiad-association.github.io/future-engineers-gs/.

_If you have any suggestions or found some bugs/inconsistencies please report them in form of [the GitHub issue](https://github.com/World-Robot-Olympiad-Association/future-engineers-gs/issues/new)._
