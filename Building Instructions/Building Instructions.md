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

1. Build a ROS2 environment from source in Raspberry Pi OS
   
2. Switch from AP mode to STA mode so that raspberry pi has access to the internet
- Run command “nano hiwonder-toolbox/wifi_conf.py”, edit the file with your wifi and password.
Save the changes and reboot via command “sudo reboot”

<img width="556" height="351" alt="image" src="https://github.com/user-attachments/assets/98e79421-b81a-4495-a92f-75dfc845175b" />



