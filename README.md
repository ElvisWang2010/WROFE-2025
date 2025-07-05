# Engineering Documentation

This repository documents Team Skib's building and programming process.


## Team Members

- Ryan Rao, 14
- Elvis Wang, 14
- Harry Xiao, 15

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
| Raspberry Pi HAT | [`HiWonder TurboPi HAT`](https://www.hiwonder.com/collections/raspberrypi-bionic-robot/products/turbopi?variant=40112905388119) | $61 |
| Raspberry Pi | [`Vemico Raspberry Pi 4`](https://www.amazon.ca/Vemico-Raspberry-Kit-Heatsinks-Screwdriver/dp/B09WXRCYL4/ref=sr_1_3) | $176 |
| Raspberry Pi Fan | [`GeeekPi Fan`](https://www.amazon.ca/dp/B07C9H9LJN?psc=1&ref=ppx_yo2ov_dt_b_product_details) | $19 |
| ON OFF Switch | [`DaierTek Switch`]() | $13 |

## Component Structure

### Chassis

For our build, we selected the **ACXWA CD chassis**, a non-prebuilt frame originally crafted with a modular design to provide adjustable lengths between 1/28 and 1/24 scale. During development, we redesigned the component connecting the front and rear sections to improve structural stability. This modification locked the chassis into a fixed 1/24 scale (16.2 cm). While adjustable lengths provide flexibility, they introduce structural weakness. Our fixed build removes this weak point, creating a rigid, non-sliding frame.

### Notable Features / Design Advantages

- **Cost-effective:** At only **$27 CAD**, the chassis allowed us to remain within budget, compared to typical prebuilt chassis costing between **$50–$100+**.
- **Motor stand included:** This reduced assembly time and ensured accurate motor alignment, minimizing drivetrain losses and gear misalignment.
- **High steering range:** Tested front wheels pivoted significantly, enabling tight turns with less steering input—ideal for technical track layouts.
- **Compact form factor:** The fixed 1/24 scale size facilitated a streamlined electronics layout (ESC, battery), reducing clutter and improving airflow for cooling.

### Chassis Limitations

⚠️ Despite these benefits, the chassis presented several challenges that impacted performance and assembly:

- **Tire traction:** The frictionless plastic tires led to frequent wheelspin and poor grip, especially on smooth surfaces like the game map. This caused unstable acceleration and longer braking distances, reducing control at higher speeds.
- **Steering imbalance:** Asymmetric turning angles made consistent driving difficult and required software compensation.
- **Material flex:** The plastic frame flexed under pressure at screw joints, potentially affecting long-term durability.
- **Assembly complexity:** Non-prebuilt design required manual hole adjustments, and instructions in a foreign language slowed assembly and increased the chance of errors.

Additionally, the steering system exhibited asymmetric turning angles: the car turned more sharply in one direction than the other. This imbalance made it difficult to drive in straight lines or execute consistent turns, and could not be easily corrected due to the limited adjustability of the stock steering linkage. The overall material quality of the chassis also presented concerns. The plastic frame flexed slightly under pressure, particularly at the screw joints, which could compromise long-term durability. Furthermore, although the chassis came with a motor stand, the non-prebuilt nature of the chassis made the assembly process slow and unintuitive. Certain holes required manual widening to fit screws properly, and the instructions were in a different language, which increased the chance of incorrect assembly. As a result, while the chassis served its purpose, it demanded extra effort and compensations to overcome its inherent flaws.

## Design
## Motor


## Power Resource Management
### Battery
Our rc car gets its power from a single `Gens Ace 1300mAh 7.4V Battery`. We chose this battery primarily for one reason: its high discharge rate. The 45C discharge rate allows for quick bursts of power, making it suitable for demanding applications despite the battery from being compact and lightweight. This powerful battery easily supplies the power demands of all of the components of our car, requiring a voltage regulator that resides in our Pi HAT to power our `Raspberry Pi 4B`, running on 5V.

## Software

## Complete Construction Manual


The WRO Future Engineers is the competition of the self-driven vehicles. Students of the age group from 15 till 19 need to design a model of a car, equip it with electromechanical components and program it as so it will be able to autonomously drive on the track avoiding objections.

More details about the competition can be found on [the official site of WRO Association](https://wro-association.org/competition/new-competition-formats/future-engineers).

<img src="docs/img/fe-map.png" width="640">

The materials are intended to be build with the [Mkdocs](https://www.mkdocs.org/) site generator.

The example of the site is accessible by https://world-robot-olympiad-association.github.io/future-engineers-gs/.

_If you have any suggestions or found some bugs/inconsistencies please report them in form of [the GitHub issue](https://github.com/World-Robot-Olympiad-Association/future-engineers-gs/issues/new)._
