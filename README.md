# Mini Atom

## Introduction
Mini Atom is a custom two-wheeled differential drive robot designed for autonomous navigation and teleoperation. It bridges low-level hardware control on an ESP32 S3 microcontroller with high-level navigation logic running on ROS 2 (Ubuntu 24.04).

## Demo

https://github.com/user-attachments/assets/228bb11c-85bc-4ee5-88f6-8ebe2e608b24

## Hardware
- [ESP32 S3 Devkit (ESP32 S3 WROOM 1 N16R8 | 16MB Flash + 8MB PSRAM)](https://robu.in/product/esp32-s3-devkit-esp32-s3-wroom-1-n16r8/)
- [N20 6V 100RPM Micro Metal Gear Motor With Encoder](https://robu.in/product/n20-6v-100rpm-micro-metal-gear-motor-with-encoder/)
- [Battery 18650](https://makerbazar.in/products/18650-3-7v-lithium-ion-rechargeable-cell-good-quality?pr_prod_strat=e5_desc&pr_rec_id=57ca5b87e&pr_rec_pid=8807406928112&pr_ref_pid=9984551583984&pr_seq=uniform&variant=47533301629168)
- [Battery Holder](https://makerbazar.in/products/18650-battery-single-cell-holder-case?variant=48251184054512)
- [DRV8833 Motor Driver](https://makerbazar.in/products/drv8833-2-channel-1-5a-dc-motor-driver?variant=48341841346800)
- [Jumper Wires](https://makerbazar.in/products/jumper-cable-male-female?variant=49704067694832&country=IN&currency=INR&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&gad_source=1&gad_campaignid=17426677322&gbraid=0AAAAACLxaAYZRzGruuzvPDvgYJLc_LG4k&gclid=CjwKCAjwgajSBhBEEiwASicJU4mxGk7dmW_OVx_leIf7TlC30MYFkrN3UzSZUkkukZ1jz08AU3cYjhoCv60QAvD_BwE)
- [Lipo Battery (Generic 523450 3.7V 1800mAh)](https://makerbazar.in/products/generic-523450-3-7v-1800mah-lipo-battery-single-cell-lithium-polymer-battery?pr_prod_strat=e5_desc&pr_rec_id=90c2eb343&pr_rec_pid=8334609875184&pr_ref_pid=7493557223664&pr_seq=uniform)
- [Caster Wheel](https://makerbazar.in/products/mini-3pi-car-n20-caster-robot-ball-wheel?variant=48251083358448)
- [Wheels (43mm Rubber Wheel Tyre for N20)](https://makerbazar.in/products/43mm-rubber-wheel-tyre-for-n20-gear-motor)
- [Mounting Bracket](https://makerbazar.in/products/mounting-bracket-for-n20-metal-gear-motors?variant=42616145772784)
- [DIP Slide Switch 2.54mm Straight](https://makerbazar.in/products/dip-slide-switch-2-54mm?variant=45382567821552)

## Software Requirements
- **Ubuntu 24.04**
- **ROS 2** (Jazzy/Humble)
- **Arduino CLI** (or Arduino IDE)

### ESP32 Library Installation
To compile the ESP32 code for the ESP32 S3 Dev Kit, install the ESP32 board manager in Arduino CLI:
1. Open your terminal.
2. Add the ESP32 board URL to Arduino CLI:
   `arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json`
3. Install the ESP32 core:
   `arduino-cli core install esp32:esp32 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json`

## Configurations (Network & IPs)

### 1. Turn on Laptop Hotspot
1. Go to your Ubuntu WiFi settings.
2. Turn on the "Wi-Fi Hotspot" feature to create a local network for the ESP32 to connect to.
3. Note the Hotspot SSID and Password you set.

### 2. Find Laptop IP Address
1. Open a terminal.
2. Run the command: `ip a`
3. Look for the interface associated with your hotspot (often `wlan0` or similar) and find the `inet` address (e.g., `10.42.0.1`).

### 3. Update ESP32 Configuration
1. Open the file `mini_atom_esp32/.env` in the repository.
2. Change `ENV_SSID` to your Hotspot Name.
3. Change `ENV_PASSWORD` to your Hotspot Password.
4. Change `ENV_LAPTOP_IP` to the IP address you found in Step 2.

### 4. Find ESP32 IP Address
1. Flash the code to your ESP32.
2. Install Angry IP Scanner on Ubuntu: `sudo apt install angryipscanner` (or download the `.deb` from their website).
3. Open Angry IP Scanner and scan your hotspot subnet (e.g., `10.42.0.0` to `10.42.0.255`).
4. Look for the active IP that corresponds to the ESP32 on the network.

### 5. Update ROS 2 Navigation Configuration
1. Open the file `mini_atom_ws/src/mini_atom_navigation/.env`.
2. Change `ESP32_IP` to the ESP32 IP address you found in Step 4.

## How to Run the Robot

### 1. Build the ROS 2 Workspace
1. Open a terminal.
2. Navigate to your workspace directory: `cd mini_atom/mini_atom_ws`
3. Build the packages: `colcon build`

### 2. Source the Workspace
1. Run: `source install/setup.bash`

### 3. Start the Hardware Control & Navigation Node
1. Run: `ros2 run mini_atom_navigation navigate_to_pose`
2. You should now be able to use keyboard teleop (w,a,s,d) to move the robot.

### 4. Optional: Start RViz for Autonomous Navigation
1. Open a new terminal and source the workspace: `source install/setup.bash`
2. Launch RViz with your navigation config (if available), or just `rviz2`.
3. Use the "2D Goal Pose" tool in RViz to set a destination. The robot will autonomously navigate to the goal marker.
