# Robot Controller Interface

A Python-based interface for controlling a robot with GPS navigation capabilities. This project includes both the controller interface software and the Arduino firmware for the robot.

## Features

- Real-time controller input monitoring and visualization
- GPS navigation with waypoint support
- Interactive map display with click-to-add waypoints
- Route tracking and waypoint management
- Serial communication with Arduino controller
- Support for multiple input devices

## Requirements

### Python Dependencies
- nicegui
- pyserial
- inputs
- folium
- asyncio

### Arduino Dependencies
- AccelStepper
- Servo
- Wire
- Adafruit_ADXL345_U
- TinyGPS++

## Installation

1. Clone the repository
2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Upload the Arduino sketch (`controller_interface.ino`) to your Arduino board

## Usage

1. Run the Python interface:
   ```bash
   python src/main.py
   ```
2. Connect your controller
3. Select the appropriate serial port for the Arduino
4. Use the interface to monitor controller inputs and manage GPS waypoints

## GPS Features

- Real-time GPS position tracking
- Interactive map display
- Click-to-add waypoints
- Waypoint completion tracking
- Route visualization
- Distance and bearing calculations

## License

This project is licensed under the MIT License - see the LICENSE file for details. 