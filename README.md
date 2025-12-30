# 🏭 ESP32 Automatic Color Sorting Conveyor

![Language](https://img.shields.io/badge/Language-C-blue?style=for-the-badge&logo=c)
![Hardware](https://img.shields.io/badge/Hardware-ESP32-red?style=for-the-badge&logo=espressif)
![Status](https://img.shields.io/badge/Status-Completed-success?style=for-the-badge)

> An embedded system project utilizing the **ESP32-WROOM-32D** to automate product classification based on color sensing.

---

## 👥 Project Team

| Student Name | Student ID | Role |
| :--- | :--- | :--- |
| **Bui Quoc Bao** | `23520092` | Leader |
| **Trinh Nguyen Thanh Binh** | `23520162` | Member |
| **Le Tran Huynh Phong** | `23521164` | Member |
| **Diep Thanh Phong** | `23521160` | Member |

---

## 📖 Overview
<img width="837" height="622" alt="image" src="https://github.com/user-attachments/assets/f754aae3-74cb-4726-a5d7-94b54afd6761" />
</t> *Overview image of the finished system*

This project implements a miniature automated conveyor belt system capable of sorting products by color. The system uses an **RGB Color Sensor** to detect the object's color and controls a **Servo Motor** to route the product to the correct destination.

### Key Features
* **Conveyor Control:** DC Motor speed regulation via PWM.
* **Color Detection:** Real-time RGB value analysis using I2C communication.
* **Sorting Mechanism:** Precision servo arm movement.
* **Product Counting:** Obstacle avoidance sensor (IR) using Interrupts to count sorted items.

---

## 🛠️ Hardware & Tech Stack

### Components
* **Microcontroller:** ESP32-WROOM-32D Development Board.
* **Actuators:**
    * DC Motor (Conveyor Belt).
    * Servo Motor (Sorting Arm).
* **Sensors:**
    * RGB Color Sensor (e.g., TCS34725).
    * IR Obstacle Avoidance Sensor.

### Technical Concepts
To achieve system stability, we utilized the following embedded protocols and techniques:
* **GPIO:** Digital Input/Output control.
* **I2C Protocol:** Communication with the RGB Sensor.
* **PWM (Pulse Width Modulation):** Controlling Motor speed and Servo angle.
* **Interrupts (ISR):** efficient handling of product counting sensors without blocking the CPU.

---

## 📂 Project Structure

```text
├── main/
│   ├── main.c              # Main application logic
│   └── ...
├── components/
│   ├── motor_speed_control/         # Library for DC Motor
│   ├── tcs34725/                    # Library for RGBSensors
│   └── ...
├── CMakeLists.txt          # Build configuration
└── README.md               # Project documentation
