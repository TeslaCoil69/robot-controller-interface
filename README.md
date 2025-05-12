# Robot Controller Interface

A Python-based interface for controlling a robot with GPS navigation capabilities. This project includes both the controller interface software and the Arduino firmware for the robot.

## Features

- Real-time controller input monitoring and visualization
- GPS navigation with waypoint support
- Interactive map display with click-to-add waypoints
- Route tracking and waypoint management
- Return to Home functionality
- Compass heading with BNO055 sensor
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
- Adafruit_BNO055
- Adafruit_Sensor
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

### Position Tracking
- Real-time GPS position tracking
- Interactive map display
- Position history tracking

### Waypoint Management
- Click-to-add waypoints on the map
- Waypoint table with name, coordinates, and status
- Automatic waypoint completion when within range
- Customizable completion radius

### Return to Home (RTH)
The Return to Home feature allows your robot to automatically navigate back to its starting position:

- **How it works**: The system automatically saves the first valid GPS position after startup as the "home" location
- **Activation**: Click the "Return to Home" button in the GPS section
- **Behavior**: Creates a new waypoint at the home position that the robot can navigate to
- **Requirements**: Requires a valid GPS fix on startup to set the home position

### Compass Navigation
The BNO055 compass sensor provides accurate heading information for navigation:

- **Heading**: Displays the current direction in degrees (0-360°)
- **Pitch and Roll**: Shows the orientation of the vehicle in 3D space
- **Auto-Calibration**: The BNO055 features self-calibration capabilities
- **Calibration Status**: Visual indicators display calibration levels for system, gyroscope, accelerometer, and magnetometer
- **Integration**: Compass data is available simultaneously with GPS data, even without GPS signal
- **Real-time Updates**: Heading updates twice per second for responsive navigation

### Route Navigation
- Route visualization with lines between waypoints
- Distance and bearing calculations to next waypoint
- Route progress tracking
- Current route highlighted on map

## License

This project is licensed under the MIT License - see the LICENSE file for details. 