# Smart 3D Printer Safety and Readiness System

## Overview
The **Smart 3D Printer Safety and Readiness System** ensures secure and reliable 3D printing by monitoring **temperature** and **vibrations** using an **STM32F446ZE microcontroller**. The system prevents printing if the temperature is below **Threshold** and alerts users to excessive vibrations, reducing errors and protecting hardware. 

### **Features:**
- **Real-time temperature monitoring** to ensure optimal printing conditions.
- **Vibration detection** to prevent print misalignment and hardware damage.
- **LCD display for live status updates**.
- **Buzzer alerts** for safety notifications.
- **STM32CubeIDE firmware** for real-time processing and control.

---
## Components Required
- **STM32F446ZE Microcontroller**
- **I2C LCD (16x2) Display**
- **DHT11 Temperature Sensor**
- **SW-18010 Vibration Sensor**
- **HW-508 Buzzer**
- **Jumper Wires and Breadboard**

---
## Connection Details

| **Device**          | **Pin Name** | **Connection** |
|---------------------|-------------|---------------|
| **I2C LCD**        | VCC         | 5V            |
|                     | GND         | GND           |
|                     | SDA         | Pin D14       |
|                     | SCL         | Pin D15       |
| **DHT11 Sensor**    | VCC         | 5V            |
|                     | GND         | GND           |
|                     | OUT         | Pin A0       |
| **Vibration Sensor**| VCC         | 3.3V          |
|                     | GND         | GND           |
|                     | D0          | Pin C0        |
| **Buzzer**         | VCC         | 5V            |
|                     | GND         | GND           |
|                     | S           | Pin A3        |

---
## Working Principle

### **1. Initialization**
- Upon powering up, the **STM32 microcontroller** initializes the **I2C LCD, Temperature Sensor, Vibration Sensor,** and **Buzzer**.
- The **LCD displays a default message**.

### **2. Temperature Monitoring**
- The **DHT11 temperature sensor** measures the ambient temperature.
- If the **temperature is greater than the Threshold Temperature**, the system displays **"Ready for Printing"** and enables printing.
- If the **temperature is below the Threshold Temperature**, the LCD displays **"Not Ready for Printing"**, and printing remains disabled.

### **3. Vibration Detection**
- The **SW-18010 vibration sensor** detects excessive vibrations during printing.
- If vibrations exceed a **predefined threshold**, the **buzzer activates**, and the LCD displays **"Vibration Alert"**.

### **4. Real-time Monitoring and Control**
- The firmware, developed using **STM32CubeIDE**, utilizes **GPIO, ADC, and timers** for real-time data processing and hardware control.
- The system continuously **monitors and updates the status** based on sensor inputs, ensuring **safe and stable 3D printing**.

---
## Applications

### **1. Enhanced 3D Printing Safety**
- Prevents printing under **inadequate temperature conditions**, reducing material waste and print failures.
- Detects **vibrations** during operation, preventing **misalignments and hardware damage**.

### **2. Industrial & Manufacturing Use**
- Ensures **consistent print quality** in automated production environments.
- Reduces **downtime** by alerting users to **operational instabilities**.

---
## How to Use
1. **Connect** the components as shown in the **Connection Details** section.
2. **Upload the firmware** to the **STM32F446ZE microcontroller** using **STM32CubeIDE**.
3. **Power on the system**:
   - The **LCD will show a default message**.
4. **Temperature Monitoring**:
   - The **DHT11 sensor** measures the temperature and displays real-time values on the **LCD screen**.
   - If the temperature is too low, printing remains **disabled**.
5. **Vibration Detection**:
   - If the **printer vibrates excessively**, the **buzzer sounds an alarm**.
   - The **LCD displays a vibration warning** to notify the user.

---
