
# 🎛️ STM32F401 Signal Generator

## 📘 Overview

This project implements a **Signal Generator** using the **STM32F401** microcontroller. The design is split into two main modules:

- **User Interface Module**: Allows the user to set the signal type and frequency using a keypad and a potentiometer.
- **Signal Generation Module**: Responsible for generating the signals, and it communicates with the user interface via **SPI**.

  ![image](https://github.com/user-attachments/assets/b2eb47c6-6040-43be-8755-b254830ddbaa)

  ![image](https://github.com/user-attachments/assets/e5e019c1-ad20-4b9b-b126-08e188a46fe0)



## ✨ Key Features

- **fixed point Arithmetic**: Utilizes fixed point calculations(taylor series) for efficiency instead of floating point, enhancing performance.
  ![image](https://github.com/user-attachments/assets/47991f9c-5b5c-4a16-be05-c4c2bf9c2707)

- **Timer Utilization**: The microcontroller's timer is used to precisely control signal generation.
- **Zero Crossing Detection**: Incorporates zero crossing for accurate signal shaping.
  ![image](https://github.com/user-attachments/assets/06b42d87-d118-4c43-97e1-9b4b094ce553)


## 🛠️ How it Works

1. The user selects the **signal type** and sets the **frequency** via the **keypad** and **potentiometer**.
2. The **User Interface Module** sends data to the **Signal Generation Module** using **SPI**.
3. The **Signal Generation Module** generates the signal, controlled by the internal timer and ensuring proper **zero crossing** for smooth transitions.
