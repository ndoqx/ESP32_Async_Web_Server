# ESP32 Web-Controlled Robot

This project allows you to **control a robot powered by an ESP32** via a web interface hosted on the ESP32 itself. The robot’s movement can be directed using buttons for **FORWARD, BACKWARD, LEFT, RIGHT**, with speed settings (**SLOW, NORMAL, FAST**) and a **custom slider** for precision control. The interface is styled using a separate CSS file.

### Features

* Web server hosted on ESP32 using **SPIFFS**
* **Asynchronous web control** using `ESPAsyncWebServer`
* Separate HTML & CSS files for modular design
* Controls two DC motors via **motor driver pins**
* **Speed control** via button presets or slider input
* Visual feedback using four colored LEDs
* Responsive layout for browser-based control

---

## Project Structure

```
project/
│
├── data/
│   ├── index.html      # Web interface for controlling the robot
│   └── style.css       # Styling for the HTML interface
│
├── src/
│   └── main.ino        # Arduino sketch (ESP32 robot logic)
│
├── README.md
```

---

## ⚙Hardware Requirements

* ESP32 development board
* Dual H-Bridge motor driver (e.g., L298N)
* 2x DC motors
* 4 LEDs (Red, Green, Blue, Yellow)
* Power supply
* Jumper wires, Breadboard

---

## Software & Libraries

* **Arduino IDE**
* ESP32 board package
* Install these libraries via the Library Manager:

  * `ESPAsyncWebServer`
  * `AsyncTCP`
  * `FS` (for SPIFFS file system)

---

## Getting Started

### 1. Upload Web Files to SPIFFS

Use the [ESP32 filesystem uploader tool](https://randomnerdtutorials.com/install-esp32-filesystem-uploader-arduino-ide/) to upload the `index.html` and `style.css` into the ESP32:

* Place `index.html` and `style.css` into the `data/` folder
* Upload to SPIFFS via the IDE

### 2. Upload Arduino Code

* Connect your ESP32 to your computer
* Open `main.ino` in Arduino IDE
* Replace Wi-Fi credentials:

  ```cpp
  const char* ssid = "YOUR_SSID";
  const char* password = "YOUR_PASSWORD";
  ```
* Upload the sketch

### 3. Access Web Interface

* Open the Serial Monitor (baud 115200)
* Get the ESP32’s IP address
* Open it in your browser (e.g., `http://192.168.1.x`)

---

## Web Controls

Each button sends an HTTP GET request:

| Button           | Action              |
| ---------------- | ------------------- |
| FORWARD          | Move robot forward  |
| BACKWARD         | Move robot backward |
| LEFT             | Turn left           |
| RIGHT            | Turn right          |
| STOP             | Stop motors         |
| FAST/NORMAL/SLOW | Speed presets       |
| Slider           | Set custom speed    |

The ESP32 updates GPIO states and controls PWM accordingly.

---

## UI Preview

Styled with soft pastel themes, large buttons for touch-friendliness, and a slider for fine-grained control.

---

## Credits

Developed based on [Rui Santos’ ESP32 web server tutorials](https://randomnerdtutorials.com).

---

## Future Enhancements

* Add camera streaming for live robot vision
* Implement obstacle detection
* Deploy over Wi-Fi hotspot (Access Point mode)
* Mobile-first responsive design

---
