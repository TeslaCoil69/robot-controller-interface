from nicegui import ui, app
from inputs import get_gamepad
import threading
import time
from collections import deque
import asyncio
import serial
import serial.tools.list_ports
import folium
from folium import plugins
import json
from datetime import datetime
import os
from dataclasses import dataclass
from typing import List, Optional, Tuple
import math
import numpy as np
from shapely.geometry import Point, Polygon

@dataclass
class Waypoint:
    latitude: float
    longitude: float
    name: str
    timestamp: datetime
    description: Optional[str] = None
    completed: bool = False

    def distance_to(self, lat: float, lon: float) -> float:
        """Calculate distance to another point in meters using Haversine formula"""
        R = 6371000  # Earth's radius in meters
        lat1, lon1 = math.radians(self.latitude), math.radians(self.longitude)
        lat2, lon2 = math.radians(lat), math.radians(lon)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def bearing_to(self, lat: float, lon: float) -> float:
        """Calculate bearing to another point in degrees"""
        lat1, lon1 = math.radians(self.latitude), math.radians(self.longitude)
        lat2, lon2 = math.radians(lat), math.radians(lon)
        
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360

class GPSData:
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed = None
        self.satellites = None
        self.hdop = None
        self.last_update = None
        self.history = deque(maxlen=100)  # Keep last 100 positions
        self.has_fix = False
        self.waypoints: List[Waypoint] = []
        self.next_waypoint_id = 1
        self.completion_radius = 5.0  # meters
        self.active_waypoint_index = 0
        self.home_position = None  # (lat, lon)
        self.home_set = False
        # Add compass data
        self.compass_heading = None
        self.compass_pitch = None
        self.compass_roll = None
        self.compass_ready = False
        self.compass_last_update = None
        # Add calibration status for BNO055
        self.compass_system_cal = 0
        self.compass_gyro_cal = 0
        self.compass_accel_cal = 0
        self.compass_mag_cal = 0
        # Add autopilot data
        self.autopilot_active = False
        self.autopilot_target_heading = None
        self.autopilot_cross_track_error = 0.0
        self.autopilot_distance_to_waypoint = 0.0
        self.autopilot_arrival_radius = 5.0  # meters, when we consider a waypoint "reached"
        self.autopilot_lookahead_distance = 10.0  # meters, for pure pursuit algorithm
        self.autopilot_last_target_point = None
        self.autopilot_steering_angle = 0
        self.autopilot_throttle = 0
        self.autopilot_speed_limit = 70  # % of max throttle
        # Add geofencing data
        self.geofence_active = False
        self.geofence_points = []  # List of (lat, lon) tuples defining the fence boundary
        self.geofence_polygon = None  # Shapely polygon for containment checks
        self.geofence_name = "Default Fence"
        self.geofence_violation = False
        self.last_geofence_check = None
        self.inside_geofence = True  # Assume inside until first check
        self.geofence_file = 'geofences.json'
        self.available_geofences = {}  # name -> points mapping
        self.load_geofences()  # Load saved geofences

    def update(self, lat, lon, alt, speed, sats, hdop):
        self.latitude = lat
        self.longitude = lon
        self.altitude = alt
        self.speed = speed
        self.satellites = sats
        self.hdop = hdop
        self.last_update = datetime.now()
        self.has_fix = True
        self.history.append({
            'lat': lat,
            'lon': lon,
            'time': self.last_update.strftime('%H:%M:%S')
        })
        # Set home position if not already set
        if not self.home_set:
            self.home_position = (lat, lon)
            self.home_set = True
        # Check for waypoint completion
        self.check_waypoint_completion(lat, lon)
        
        # Check geofence compliance if active
        if self.geofence_active and self.geofence_polygon:
            inside = self.check_geofence(lat, lon)
            # If we detect a violation, notify immediately
            if not inside and not self.inside_geofence:  # Just crossed outside
                self.geofence_violation = True
                print(f"GEOFENCE VIOLATION at position {lat:.6f}, {lon:.6f}!")
                # The actual disarm and stop commands are handled in the controller's _send_serial_data method

    def no_fix(self):
        self.has_fix = False
        self.last_update = datetime.now()

    def add_waypoint(self, lat: float, lon: float, description: Optional[str] = None) -> Waypoint:
        """Add a new waypoint to the list"""
        waypoint = Waypoint(
            latitude=lat,
            longitude=lon,
            name=f"WP{self.next_waypoint_id}",
            timestamp=datetime.now(),
            description=description
        )
        self.waypoints.append(waypoint)
        self.next_waypoint_id += 1
        return waypoint

    def remove_waypoint(self, waypoint: Waypoint) -> None:
        """Remove a waypoint from the list"""
        if waypoint in self.waypoints:
            self.waypoints.remove(waypoint)

    def check_waypoint_completion(self, lat: float, lon: float):
        """Check if current position is within range of any waypoint"""
        if not self.waypoints:
            return

        # Only check the next uncompleted waypoint
        for i, wp in enumerate(self.waypoints):
            if not wp.completed:
                distance = wp.distance_to(lat, lon)
                if distance <= self.completion_radius:
                    wp.completed = True
                    self.active_waypoint_index = min(i + 1, len(self.waypoints) - 1)
                break

    def get_next_waypoint(self) -> Optional[Waypoint]:
        """Get the next uncompleted waypoint"""
        for wp in self.waypoints:
            if not wp.completed:
                return wp
        return None

    def get_route_info(self, current_lat: float, current_lon: float) -> Tuple[float, float, float]:
        """Get distance, bearing, and total route progress"""
        if not self.waypoints:
            return 0.0, 0.0, 0.0

        next_wp = self.get_next_waypoint()
        if not next_wp:
            return 0.0, 0.0, 100.0  # All waypoints completed

        # Calculate distance and bearing to next waypoint
        distance = next_wp.distance_to(current_lat, current_lon)
        bearing = next_wp.bearing_to(current_lat, current_lon)

        # Calculate total route progress
        completed = sum(1 for wp in self.waypoints if wp.completed)
        total = len(self.waypoints)
        progress = (completed / total) * 100 if total > 0 else 0

        return distance, bearing, progress

    def set_home(self, lat, lon):
        self.home_position = (lat, lon)
        self.home_set = True

    def get_home(self):
        return self.home_position

    def trigger_return_to_home(self):
        """Add a temporary waypoint at the home position if set."""
        if self.home_set and self.home_position:
            lat, lon = self.home_position
            # Add a special RTH waypoint
            rth_wp = Waypoint(
                latitude=lat,
                longitude=lon,
                name="RTH",
                timestamp=datetime.now(),
                description="Return to Home (auto-added)"
            )
            self.waypoints.append(rth_wp)
            return rth_wp
        return None

    def calculate_desired_heading(self, current_lat, current_lon):
        """Calculate the desired heading to the next waypoint."""
        next_wp = self.get_next_waypoint()
        if not next_wp:
            return None
            
        # Calculate bearing to waypoint
        bearing = next_wp.bearing_to(current_lat, current_lon)
        self.autopilot_target_heading = bearing
        return bearing
        
    def calculate_cross_track_error(self, current_lat, current_lon):
        """Calculate cross-track error (deviation from ideal path)."""
        if len(self.waypoints) < 2 or not self.autopilot_active:
            self.autopilot_cross_track_error = 0.0
            return 0.0
            
        # Get current and next waypoint
        next_wp = None
        for i, wp in enumerate(self.waypoints):
            if not wp.completed:
                if i > 0:  # We have a previous waypoint to form a path
                    prev_wp = self.waypoints[i-1]
                    next_wp = wp
                    break
                else:
                    # First waypoint, no previous path
                    self.autopilot_cross_track_error = 0.0
                    return 0.0
        
        if not next_wp:
            self.autopilot_cross_track_error = 0.0
            return 0.0
            
        # Simplified cross-track error calculation
        # Convert coordinates to a local Cartesian system
        EARTH_RADIUS = 6371000  # Earth radius in meters
        
        # Convert all coordinates to radians
        lat1, lon1 = math.radians(prev_wp.latitude), math.radians(prev_wp.longitude)
        lat2, lon2 = math.radians(next_wp.latitude), math.radians(next_wp.longitude)
        lat3, lon3 = math.radians(current_lat), math.radians(current_lon)
        
        # Calculate the perpendicular distance to the great circle path
        x1 = EARTH_RADIUS * math.cos(lat1) * math.cos(lon1)
        y1 = EARTH_RADIUS * math.cos(lat1) * math.sin(lon1)
        z1 = EARTH_RADIUS * math.sin(lat1)
        
        x2 = EARTH_RADIUS * math.cos(lat2) * math.cos(lon2)
        y2 = EARTH_RADIUS * math.cos(lat2) * math.sin(lon2)
        z2 = EARTH_RADIUS * math.sin(lat2)
        
        x3 = EARTH_RADIUS * math.cos(lat3) * math.cos(lon3)
        y3 = EARTH_RADIUS * math.cos(lat3) * math.sin(lon3)
        z3 = EARTH_RADIUS * math.sin(lat3)
        
        # Vector cross product to get perpendicular vector to path
        cx = (y1*z2 - z1*y2)
        cy = (z1*x2 - x1*z2)
        cz = (x1*y2 - y1*x2)
        
        # Normalize
        norm = math.sqrt(cx*cx + cy*cy + cz*cz)
        if norm < 1e-10:  # Too small, avoid division by zero
            self.autopilot_cross_track_error = 0.0
            return 0.0
            
        cx, cy, cz = cx/norm, cy/norm, cz/norm
        
        # Dot product with current position to get distance
        cross_track = abs(cx*x3 + cy*y3 + cz*z3)
        
        # Sign of the cross-track error (left or right of path)
        # Use the determinant to determine which side
        det = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)
        cross_track = cross_track if det > 0 else -cross_track
        
        self.autopilot_cross_track_error = cross_track
        return cross_track
    
    def calculate_pure_pursuit_steering(self, current_lat, current_lon, current_heading):
        """Calculate steering angle using pure pursuit algorithm."""
        if not self.autopilot_active:
            return 0
            
        next_wp = self.get_next_waypoint()
        if not next_wp:
            return 0
            
        # Calculate distance to waypoint
        distance = next_wp.distance_to(current_lat, current_lon)
        self.autopilot_distance_to_waypoint = distance
        
        # Check if we've reached the waypoint
        if distance <= self.autopilot_arrival_radius:
            next_wp.completed = True
            self.active_waypoint_index = min(self.active_waypoint_index + 1, len(self.waypoints) - 1)
            return 0
            
        # Calculate lookahead point along the path
        target_lat, target_lon = next_wp.latitude, next_wp.longitude
        
        # Calculate heading error (difference between current heading and desired heading)
        desired_heading = self.calculate_desired_heading(current_lat, current_lon)
        if desired_heading is None:
            return 0
            
        heading_error = desired_heading - current_heading
        # Normalize to -180 to 180
        heading_error = (heading_error + 180) % 360 - 180
        
        # Calculate cross-track error for path correction
        cross_track_error = self.calculate_cross_track_error(current_lat, current_lon)
        
        # Proportional control for steering
        # Combine heading error and cross-track error
        kp_heading = 1.0
        kp_cross_track = 0.01  # Gain for cross-track error (adjust based on testing)
        
        # Calculate steering angle (-100 to 100 for controller emulation)
        steering_angle = kp_heading * heading_error + kp_cross_track * cross_track_error
        
        # Limit steering angle to valid range
        steering_angle = max(-100, min(100, steering_angle))
        
        # Set the calculated steering angle
        self.autopilot_steering_angle = int(steering_angle)
        
        # Calculate throttle based on distance and steering angle
        # Reduce speed when turning sharply or close to waypoint
        base_throttle = self.autopilot_speed_limit
        turn_factor = 1.0 - (abs(steering_angle) / 100.0) * 0.5  # Reduce speed up to 50% when turning hard
        distance_factor = min(1.0, distance / 20.0)  # Reduce speed when approaching waypoint
        
        throttle = base_throttle * turn_factor * distance_factor
        self.autopilot_throttle = int(throttle)
        
        return steering_angle

    def load_geofences(self):
        """Load saved geofences from file."""
        try:
            if os.path.exists(self.geofence_file):
                with open(self.geofence_file, 'r') as f:
                    self.available_geofences = json.load(f)
        except Exception as e:
            print(f"Error loading geofences: {e}")
            self.available_geofences = {}
    
    def save_geofences(self):
        """Save current geofences to file."""
        try:
            with open(self.geofence_file, 'w') as f:
                json.dump(self.available_geofences, f)
        except Exception as e:
            print(f"Error saving geofences: {e}")
    
    def set_geofence(self, points, name="Default Fence"):
        """Set geofence from a list of (lat, lon) points."""
        if len(points) < 3:
            return False  # Need at least 3 points for a polygon
        
        self.geofence_points = points
        self.geofence_name = name
        
        # Create shapely polygon for containment checks
        if points:
            self.geofence_polygon = Polygon(points)
            # Save to available geofences
            self.available_geofences[name] = points
            self.save_geofences()
            return True
        return False
    
    def activate_geofence(self, name=None):
        """Activate geofence by name or current geofence."""
        if name and name in self.available_geofences:
            self.set_geofence(self.available_geofences[name], name)
        
        if self.geofence_polygon:
            self.geofence_active = True
            self.geofence_violation = False
            self.inside_geofence = True  # Reset state
            return True
        return False
    
    def deactivate_geofence(self):
        """Deactivate geofence."""
        self.geofence_active = False
        self.geofence_violation = False
        return True
    
    def delete_geofence(self, name):
        """Delete a saved geofence."""
        if name in self.available_geofences:
            del self.available_geofences[name]
            self.save_geofences()
            # If current geofence was deleted, reset it
            if name == self.geofence_name:
                self.geofence_points = []
                self.geofence_polygon = None
                self.geofence_active = False
            return True
        return False
    
    def check_geofence(self, lat, lon):
        """Check if position is inside the geofence."""
        if not self.geofence_active or not self.geofence_polygon:
            return True  # No geofence active, so considered "inside"
        
        position = Point(lon, lat)  # Note: Shapely uses (x, y) = (lon, lat)
        inside = self.geofence_polygon.contains(position)
        self.inside_geofence = inside
        self.last_geofence_check = datetime.now()
        
        # Check for violation (if was inside and now outside)
        if not inside and not self.geofence_violation:
            self.geofence_violation = True
            return False
        
        return inside

class SerialConnection:
    def __init__(self):
        self.serial = None
        self.connected = False
        self.available_ports = []
        self.update_ports()
        self.gps_data = GPSData()
        self.map_file = 'gps_map.html'
        self.map_html = ''  # Store the map HTML content
        self.map_click_callback = None

    def update_ports(self):
        """Update list of available serial ports"""
        self.available_ports = [port.device for port in serial.tools.list_ports.comports()]

    def connect(self, port):
        """Connect to the selected serial port"""
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            
            self.serial = serial.Serial(port, 57600, timeout=1)
            self.connected = True
            return True
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")
            self.connected = False
            return False

    def disconnect(self):
        """Disconnect from the current serial port"""
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False

    def write(self, data):
        """Write data to the serial port"""
        if self.connected and self.serial and self.serial.is_open:
            try:
                self.serial.write(data.encode())
            except Exception as e:
                print(f"Failed to write to serial port: {e}")
                self.connected = False

    def read_serial_data(self):
        """Read and parse data from serial port (GPS and compass)"""
        if not self.connected or not self.serial or not self.serial.is_open:
            return None

        try:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8').strip()
                if line.startswith('GPS,'):
                    # Handle GPS data
                    parts = line.split(',')
                    if len(parts) == 7 and parts[1] != 'NO_FIX':
                        try:
                            self.gps_data.update(
                                float(parts[1]),  # latitude
                                float(parts[2]),  # longitude
                                float(parts[3]),  # altitude
                                float(parts[4]),  # speed
                                int(parts[5]),    # satellites
                                float(parts[6])   # hdop
                            )
                            self.update_map()
                            return self.gps_data
                        except (ValueError, IndexError) as e:
                            print(f"Error parsing GPS data: {e}")
                    elif len(parts) == 2 and parts[1] == 'NO_FIX':
                        self.gps_data.no_fix()
                        return self.gps_data
                elif line.startswith('COMPASS,'):
                    # Handle compass data
                    parts = line.split(',')
                    if len(parts) == 8 and parts[1] != 'NOT_READY':  # Updated for BNO055 with calibration data
                        try:
                            self.gps_data.compass_heading = float(parts[1])
                            self.gps_data.compass_pitch = float(parts[2])
                            self.gps_data.compass_roll = float(parts[3])
                            self.gps_data.compass_system_cal = int(parts[4])
                            self.gps_data.compass_gyro_cal = int(parts[5])
                            self.gps_data.compass_accel_cal = int(parts[6])
                            self.gps_data.compass_mag_cal = int(parts[7])
                            self.gps_data.compass_ready = True
                            self.gps_data.compass_last_update = datetime.now()
                            return self.gps_data
                        except (ValueError, IndexError) as e:
                            print(f"Error parsing compass data: {e}")
                    elif len(parts) == 2 and parts[1] == 'NOT_READY':
                        self.gps_data.compass_ready = False
                        return self.gps_data
        except Exception as e:
            print(f"Error reading serial data: {e}")
            self.connected = False
        return None

    def update_map(self):
        """Update the map with current GPS position, waypoints, and route"""
        if not self.gps_data.has_fix or not self.gps_data.latitude or not self.gps_data.longitude:
            return

        # Create map centered on current position
        m = folium.Map(
            location=[self.gps_data.latitude, self.gps_data.longitude],
            zoom_start=15,
            tiles='OpenStreetMap'
        )

        # Add current position marker
        folium.Marker(
            [self.gps_data.latitude, self.gps_data.longitude],
            popup=f'Current Position<br>Speed: {self.gps_data.speed:.1f} km/h<br>Alt: {self.gps_data.altitude:.1f}m',
            icon=folium.Icon(color='red', icon='info-sign')
        ).add_to(m)

        # Add position history as a line
        if len(self.gps_data.history) > 1:
            positions = [(p['lat'], p['lon']) for p in self.gps_data.history]
            folium.PolyLine(
                positions,
                color='blue',
                weight=2,
                opacity=0.8
            ).add_to(m)

        # Add waypoint markers and route lines
        if self.gps_data.waypoints:
            # Draw route lines between waypoints
            route_positions = []
            for i, wp in enumerate(self.gps_data.waypoints):
                route_positions.append([wp.latitude, wp.longitude])
                
                # Add waypoint marker with different colors for completed/uncompleted
                icon_color = 'green' if wp.completed else 'orange'
                folium.Marker(
                    [wp.latitude, wp.longitude],
                    popup=f'{wp.name}<br>{wp.description or ""}<br>{wp.timestamp.strftime("%H:%M:%S")}',
                    icon=folium.Icon(color=icon_color, icon='flag', prefix='fa')
                ).add_to(m)

            # Draw route line
            folium.PolyLine(
                route_positions,
                color='green',
                weight=3,
                opacity=0.8,
                dash_array='5, 10'  # Dashed line
            ).add_to(m)

            # Draw line from current position to next waypoint
            next_wp = self.gps_data.get_next_waypoint()
            if next_wp:
                folium.PolyLine(
                    [[self.gps_data.latitude, self.gps_data.longitude],
                     [next_wp.latitude, next_wp.longitude]],
                    color='red',
                    weight=2,
                    opacity=0.8
                ).add_to(m)

        # Add geofence visualization if active or being drawn
        if self.gps_data.geofence_active and self.gps_data.geofence_points:
            # Draw geofence polygon
            folium.Polygon(
                locations=self.gps_data.geofence_points,
                color='red' if self.gps_data.geofence_violation else 'green',
                weight=3,
                fill=True,
                fill_opacity=0.1,
                popup=f'Geofence: {self.gps_data.geofence_name}'
            ).add_to(m)
        
        # Draw click handler for waypoints or geofence
        m.get_root().html.add_child(folium.Element("""
            <script>
                document.addEventListener('DOMContentLoaded', function() {
                    var map = document.querySelector('.leaflet-map-pane');
                    if (map) {
                        map.parentElement.addEventListener('click', function(e) {
                            if (e.target && e.target.classList.contains('leaflet-container')) {
                                var rect = e.target.getBoundingClientRect();
                                var x = e.clientX - rect.left;
                                var y = e.clientY - rect.top;
                                var point = L.point(x, y);
                                var latLng = e.target._leaflet_map.containerPointToLatLng(point);
                                window.sendEvent('map_click', {lat: latLng.lat, lng: latLng.lng});
                            }
                        });
                    }
                });
            </script>
        """))

        # Save the map and store its HTML content
        m.save(self.map_file)
        with open(self.map_file, 'r') as f:
            self.map_html = f.read()

    def send_disarm_command(self):
        """Send a disarm command to the robot."""
        if not self.connected or not self.serial or not self.serial.is_open:
            return
        
        try:
            # Create a special disarm message
            disarm_data = "DISARM\n"
            self.serial.write(disarm_data.encode())
            print("Geofence violation! Sent disarm command.")
        except Exception as e:
            print(f"Error sending disarm command: {e}")
            
    def send_stop_command(self):
        """Send a stop command to the robot (emergency stop without disarming)."""
        if not self.connected or not self.serial or not self.serial.is_open:
            return
        
        try:
            # Create a special stop message
            stop_data = "STOP\n"
            self.serial.write(stop_data.encode())
            print("Sent emergency stop command.")
        except Exception as e:
            print(f"Error sending stop command: {e}")

class ControllerMonitor:
    def __init__(self):
        self.running = True
        self.events_history = deque(maxlen=100)  # Keep last 100 events
        self.deadzone = 10  # Deadzone threshold (percentage)
        self.serial = SerialConnection()
        self.last_message = ""  # Store last sent message
        self.button_states = {
            'BTN_SOUTH': 0,  # A button
            'BTN_EAST': 0,   # B button
            'BTN_NORTH': 0,  # Y button
            'BTN_WEST': 0,   # X button
            'BTN_TL': 0,     # Left bumper
            'BTN_TR': 0,     # Right bumper
            'BTN_SELECT': 0, # Select button
            'BTN_START': 0,  # Start button
            'BTN_DPAD_UP': 0,
            'BTN_DPAD_DOWN': 0,
            'BTN_DPAD_LEFT': 0,
            'BTN_DPAD_RIGHT': 0,
            'ABS_HAT0X': 0,  # D-pad X axis
            'ABS_HAT0Y': 0,  # D-pad Y axis
        }
        self.axis_states = {
            'ABS_X': 0, 'ABS_Y': 0,  # Left stick
            'ABS_RX': 0, 'ABS_RY': 0,  # Right stick
            'ABS_Z': 0, 'ABS_RZ': 0,   # Triggers
        }
        self.lock = threading.Lock()
        self.autopilot_override = False
        # Add timeout for autopilot override
        self.last_manual_control_time = time.time()
        self.manual_control_timeout = 5.0  # seconds

    def scale_joystick(self, value):
        """Scale joystick value from -32768/32767 to -100/100 with deadzone"""
        scaled = (value / 32767) * 100
        # Apply deadzone
        if abs(scaled) < self.deadzone:
            return 0
        # Rescale the remaining range to maintain full range after deadzone
        if scaled > 0:
            return round(((scaled - self.deadzone) / (100 - self.deadzone)) * 100)
        else:
            return round(((scaled + self.deadzone) / (100 - self.deadzone)) * -100)

    def start_monitoring(self):
        # Start controller monitoring thread
        self.monitor_thread = threading.Thread(target=self._monitor_controller)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        
        # Start serial transmission thread
        self.serial_thread = threading.Thread(target=self._serial_transmission_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()

    def _monitor_controller(self):
        while self.running:
            try:
                events = get_gamepad()
                with self.lock:
                    for event in events:
                        if event.ev_type == 'Key':
                            self.button_states[event.code] = event.state
                        elif event.ev_type == 'Absolute':
                            # Scale joystick values
                            if event.code in ['ABS_X', 'ABS_Y', 'ABS_RX', 'ABS_RY']:
                                self.axis_states[event.code] = self.scale_joystick(event.state)
                            else:
                                self.axis_states[event.code] = event.state
                            if event.code in ['ABS_HAT0X', 'ABS_HAT0Y']:
                                self.button_states[event.code] = event.state
                        
                        # Add event to history with timestamp
                        self.events_history.append({
                            'time': time.strftime('%H:%M:%S'),
                            'type': event.ev_type,
                            'code': event.code,
                            'state': event.state
                        })

            except Exception as e:
                print(f"Error reading gamepad: {e}")
                time.sleep(1)

    def _serial_transmission_loop(self):
        """Separate thread for sending serial data at fixed intervals"""
        while self.running:
            try:
                self._send_serial_data()
                time.sleep(0.2)  # Send every 200ms
            except Exception as e:
                print(f"Error in serial transmission: {e}")
                time.sleep(1)

    def _send_serial_data(self):
        """Send controller data over serial in comma-delimited format"""
        if not self.serial.connected:
            return

        # Check for geofence violation first
        gps = self.serial.gps_data
        if gps and gps.geofence_active and gps.geofence_violation:
            # If there's a geofence violation, send disarm command
            self.serial.send_disarm_command()
            
            # Also send stop command as a backup in case disarm fails
            self.serial.send_stop_command()
            
            # Also send neutral commands to ensure no movement
            data = [
                0, 0,          # Left stick X & Y (camera gimbal) - set to neutral
                0, 0,          # Right stick X & Y (steering, throttle) - set to neutral/stop
                0, 0,          # Left & right triggers - neutral
                0, 0, 0, 0,    # A, B, X, Y buttons - off
                0, 0,          # Left & right bumpers - off
                0, 0,          # Select & start buttons - off
                0, 0, 0, 0     # D-pad - off
            ]
            
            # Convert to comma-separated string and add newline
            data_str = ','.join(map(str, data)) + '\n'
            self.serial.write(data_str)
            self.last_message = data_str.strip()  # Store last message without newline
            return

        # Format: LX,LY,RX,RY,LT,RT,A,B,X,Y,LB,RB,SELECT,START,DPAD_UP,DPAD_DOWN,DPAD_LEFT,DPAD_RIGHT
        # Check if autopilot should override controls
        if gps and gps.autopilot_active and not self.autopilot_override:
            # Override right stick X (steering) and right stick Y (throttle)
            # Left stick (camera gimbal) remains user controlled
            steering_angle = gps.autopilot_steering_angle
            throttle = gps.autopilot_throttle
            
            # Check if user is trying to override autopilot
            if abs(self.axis_states['ABS_RX']) > 20 or abs(self.axis_states['ABS_RY']) > 20:
                # User is moving right stick, temporarily override autopilot
                self.autopilot_override = True
                self.last_manual_control_time = time.time()
            else:
                # Use autopilot values for right stick
                data = [
                    self.axis_states['ABS_X'],      # Left stick X - user controlled
                    self.axis_states['ABS_Y'],      # Left stick Y - user controlled
                    steering_angle,                 # Right stick X - autopilot steering
                    -throttle,                      # Right stick Y - autopilot throttle (negative for forward)
                    self.axis_states['ABS_Z'],      # Left trigger
                    self.axis_states['ABS_RZ'],     # Right trigger
                    self.button_states['BTN_SOUTH'], # A button
                    self.button_states['BTN_EAST'],  # B button
                    self.button_states['BTN_WEST'],  # X button
                    self.button_states['BTN_NORTH'], # Y button
                    self.button_states['BTN_TL'],    # Left bumper
                    self.button_states['BTN_TR'],    # Right bumper
                    self.button_states['BTN_SELECT'],# Select button
                    self.button_states['BTN_START'], # Start button
                    1 if self.button_states['ABS_HAT0Y'] == -1 else 0,  # D-pad Up
                    1 if self.button_states['ABS_HAT0Y'] == 1 else 0,   # D-pad Down
                    1 if self.button_states['ABS_HAT0X'] == -1 else 0,  # D-pad Left
                    1 if self.button_states['ABS_HAT0X'] == 1 else 0    # D-pad Right
                ]
                
                # Convert to comma-separated string and add newline
                data_str = ','.join(map(str, data)) + '\n'
                self.serial.write(data_str)
                self.last_message = data_str.strip()  # Store last message without newline
                return
        else:
            # Check if manual control timeout has expired
            if self.autopilot_override and time.time() - self.last_manual_control_time > self.manual_control_timeout:
                self.autopilot_override = False

        # Normal control (no autopilot or user override active)
        data = [
            self.axis_states['ABS_X'],      # Left stick X
            self.axis_states['ABS_Y'],      # Left stick Y
            self.axis_states['ABS_RX'],     # Right stick X
            self.axis_states['ABS_RY'],     # Right stick Y
            self.axis_states['ABS_Z'],      # Left trigger
            self.axis_states['ABS_RZ'],     # Right trigger
            self.button_states['BTN_SOUTH'], # A button
            self.button_states['BTN_EAST'],  # B button
            self.button_states['BTN_WEST'],  # X button
            self.button_states['BTN_NORTH'], # Y button
            self.button_states['BTN_TL'],    # Left bumper
            self.button_states['BTN_TR'],    # Right bumper
            self.button_states['BTN_SELECT'],# Select button
            self.button_states['BTN_START'], # Start button
            1 if self.button_states['ABS_HAT0Y'] == -1 else 0,  # D-pad Up
            1 if self.button_states['ABS_HAT0Y'] == 1 else 0,   # D-pad Down
            1 if self.button_states['ABS_HAT0X'] == -1 else 0,  # D-pad Left
            1 if self.button_states['ABS_HAT0X'] == 1 else 0    # D-pad Right
        ]
        
        # Convert to comma-separated string and add newline
        data_str = ','.join(map(str, data)) + '\n'
        self.serial.write(data_str)
        self.last_message = data_str.strip()  # Store last message without newline

    def stop_monitoring(self):
        self.running = False

# Initialize controller monitor
controller = ControllerMonitor()
controller.start_monitoring()

# Set theme colors for a professional look
ui.colors(
    primary='#1976D2',      # Deep blue
    secondary='#455A64',    # Blue grey
    accent='#00BCD4',       # Cyan
    dark='#263238',         # Dark blue grey
    positive='#4CAF50',     # Green
    negative='#F44336',     # Red
    warning='#FFC107',      # Amber
    info='#2196F3'          # Light blue
)

# Main container with professional styling
with ui.card().classes('w-full q-pa-lg bg-grey-1'):
    # Header with gradient background
    with ui.card().classes('w-full q-mb-lg bg-gradient-to-r from-primary to-secondary text-white'):
        with ui.row().classes('items-center justify-between q-pa-md'):
            ui.label('Robot Controller Interface').classes('text-h4 text-weight-bold')
            ui.label('v1.0').classes('text-subtitle1 text-grey-3')

    # Status Bar
    with ui.card().classes('w-full q-mb-lg bg-dark text-white'):
        with ui.row().classes('items-center justify-between q-pa-sm'):
            with ui.row().classes('items-center gap-4'):
                ui.icon('memory').classes('text-accent')
                ui.label('System Status:').classes('text-weight-medium')
                status_label = ui.label('Disconnected').classes('text-negative')
            with ui.row().classes('items-center gap-4'):
                ui.icon('schedule').classes('text-accent')
                ui.label('Last Update:').classes('text-weight-medium')
                last_update = ui.label('--:--:--').classes('font-mono')

    # Serial Port Selection with improved styling
    with ui.card().classes('w-full q-mb-lg bg-white'):
        with ui.row().classes('items-center justify-between q-pa-md'):
            with ui.row().classes('items-center gap-4'):
                ui.icon('settings_input_component').classes('text-primary text-h5')
                ui.label('Serial Connection').classes('text-h6 text-weight-medium')
            with ui.row().classes('items-center gap-4'):
                port_select = ui.select(
                    options=controller.serial.available_ports,
                    value=None,
                    with_input=True,
                    on_change=lambda e: controller.serial.connect(e.value) if e.value else None
                ).classes('w-64 bg-grey-1')
                
                refresh_btn = ui.button('Refresh', icon='refresh', on_click=lambda: controller.serial.update_ports()).classes('bg-primary text-white')
                
                def update_port_status():
                    status_label.set_text('Connected' if controller.serial.connected else 'Disconnected')
                    status_label.classes('text-positive' if controller.serial.connected else 'text-negative')
                    last_update.set_text(time.strftime('%H:%M:%S'))
                
                port_select.on('update:model-value', update_port_status)

    # Main content area with improved grid layout
    with ui.grid(columns=12).classes('w-full gap-4'):
        # GPS and Map section - Now at the top and more prominent
        with ui.card().classes('col-span-12 bg-white q-mb-lg'):
            with ui.card().classes('q-pa-lg'):
                # Define handlers first
                def handle_map_click(e):
                    """Handle map click events to add waypoints"""
                    # Check if we have GPS signal
                    if not controller.serial.gps_data or not controller.serial.gps_data.has_fix:
                        ui.notify('No GPS signal available. Please wait for GPS fix.', type='warning')
                        return

                    try:
                        # Extract coordinates from the click event
                        # The coordinates are in the event's data attribute
                        lat = float(e.args.get('lat', 0))
                        lon = float(e.args.get('lng', 0))
                        
                        # Validate coordinates
                        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                            ui.notify('Invalid coordinates received', type='negative')
                            return

                        # Create waypoint dialog
                        dialog = ui.dialog()
                        with dialog, ui.card().classes('w-96'):
                            ui.label('Add Waypoint').classes('text-h6 q-mb-md')
                            description = ui.input('Description (optional)').classes('w-full')
                            with ui.row().classes('w-full justify-end gap-2'):
                                ui.button('Cancel', on_click=dialog.close).classes('bg-grey-2')
                                ui.button('Add', on_click=lambda: add_waypoint(lat, lon, description.value, dialog)).classes('bg-primary text-white')
                    except (ValueError, TypeError, AttributeError) as e:
                        print(f"Error handling map click: {e}")
                        ui.notify('Error processing map click', type='negative')

                def add_waypoint(lat: float, lon: float, description: str, dialog: ui.dialog):
                    """Add a new waypoint and update the display"""
                    try:
                        waypoint = controller.serial.gps_data.add_waypoint(lat, lon, description)
                        dialog.close()
                        update_waypoints_table()
                        controller.serial.update_map()
                        update_gps_display()
                        ui.notify(f'Waypoint {waypoint.name} added successfully', type='positive')
                    except Exception as e:
                        print(f"Error adding waypoint: {e}")
                        ui.notify('Error adding waypoint', type='negative')

                def remove_waypoint(waypoint: Waypoint):
                    """Remove a waypoint and update the display"""
                    controller.serial.gps_data.remove_waypoint(waypoint)
                    update_waypoints_table()
                    controller.serial.update_map()
                    update_gps_display()

                def clear_waypoints():
                    """Clear all waypoints"""
                    controller.serial.gps_data.waypoints.clear()
                    controller.serial.gps_data.next_waypoint_id = 1
                    update_waypoints_table()
                    controller.serial.update_map()
                    update_gps_display()

                def update_waypoints_table():
                    """Update the waypoints table with current waypoints"""
                    rows = []
                    for wp in controller.serial.gps_data.waypoints:
                        status_color = 'text-positive' if wp.completed else 'text-warning'
                        rows.append({
                            'name': wp.name,
                            'lat': f'{wp.latitude:.6f}°',
                            'lon': f'{wp.longitude:.6f}°',
                            'time': wp.timestamp.strftime('%H:%M:%S'),
                            'description': wp.description or '',
                            'status': ui.label('Completed' if wp.completed else 'Pending').classes(status_color),
                            'actions': ui.button('Delete', icon='delete', on_click=lambda w=wp: remove_waypoint(w)).classes('bg-negative text-white')
                        })
                    waypoints_table.rows = rows

                with ui.row().classes('items-center justify-between q-mb-md'):
                    with ui.row().classes('items-center gap-2'):
                        ui.icon('gps_fixed').classes('text-primary text-h4')
                        ui.label('GPS Navigation').classes('text-h4 text-weight-bold text-primary')
                    gps_status = ui.label('No GPS Fix').classes('text-h6 text-negative q-pa-sm rounded')
                    with ui.row().classes('items-center gap-2 q-mb-md'):
                        ui.button('Return to Home', icon='home', on_click=lambda: return_to_home()).classes('bg-primary text-white')
                
                # GPS Status Display with improved layout
                with ui.grid(columns=12).classes('gap-4'):
                    # GPS Data Panel - Left side
                    with ui.card().classes('col-span-4 bg-grey-1 q-pa-md'):
                        with ui.column().classes('gap-3'):
                            ui.label('GPS Data').classes('text-h6 text-weight-medium q-mb-sm')
                            with ui.grid(columns=2).classes('gap-2'):
                                gps_lat = ui.label('Latitude: --').classes('font-mono text-subtitle1')
                                gps_lon = ui.label('Longitude: --').classes('font-mono text-subtitle1')
                                gps_alt = ui.label('Altitude: --').classes('font-mono text-subtitle1')
                                gps_speed = ui.label('Speed: --').classes('font-mono text-subtitle1')
                                gps_sats = ui.label('Satellites: --').classes('font-mono text-subtitle1')
                                gps_hdop = ui.label('HDOP: --').classes('font-mono text-subtitle1')
                                gps_time = ui.label('Last Update: --').classes('font-mono text-subtitle1')
                                
                                # Add compass information
                                ui.separator()
                                ui.label('Compass Data').classes('text-subtitle1 text-weight-medium')
                                compass_status = ui.label('Compass: --').classes('font-mono text-subtitle1')
                                compass_heading = ui.label('Heading: --').classes('font-mono text-subtitle1')
                                compass_pitch = ui.label('Pitch: --').classes('font-mono text-subtitle1')
                                compass_roll = ui.label('Roll: --').classes('font-mono text-subtitle1')
                                compass_time = ui.label('Last Update: --').classes('font-mono text-subtitle1')
                                
                                # Add calibration status indicators
                                ui.label('Calibration Status').classes('text-subtitle2 text-weight-medium q-mt-sm')
                                with ui.row().classes('items-center justify-start gap-2'):
                                    ui.label('System:').classes('text-caption')
                                    system_cal_indicator = ui.linear_progress(value=0).classes('w-16')
                                    ui.label('Gyro:').classes('text-caption')
                                    gyro_cal_indicator = ui.linear_progress(value=0).classes('w-16')
                                with ui.row().classes('items-center justify-start gap-2'):
                                    ui.label('Accel:').classes('text-caption')
                                    accel_cal_indicator = ui.linear_progress(value=0).classes('w-16')
                                    ui.label('Mag:').classes('text-caption')
                                    mag_cal_indicator = ui.linear_progress(value=0).classes('w-16')
                                
                                # Add route information
                                route_distance = ui.label('Distance to next: --').classes('font-mono text-subtitle1')
                                route_bearing = ui.label('Bearing to next: --').classes('font-mono text-subtitle1')
                                route_progress = ui.label('Route progress: --').classes('font-mono text-subtitle1')
                                
                                # Add completion radius setting
                                with ui.row().classes('items-center gap-2'):
                                    ui.label('Completion radius:').classes('text-subtitle2')
                                    radius_input = ui.number(
                                        value=controller.serial.gps_data.completion_radius,
                                        min=1.0,
                                        max=50.0,
                                        step=1.0,
                                        on_change=lambda e: set_completion_radius(e.value)
                                    ).classes('w-24')

                    def set_completion_radius(value: float):
                        """Update the waypoint completion radius"""
                        controller.serial.gps_data.completion_radius = value

                    # Map Panel - Right side (larger)
                    with ui.card().classes('col-span-8 bg-grey-1 q-pa-md'):
                        with ui.column().classes('gap-2'):
                            with ui.row().classes('items-center justify-between'):
                                ui.label('Live Position Map').classes('text-h6 text-weight-medium')
                                with ui.row().classes('items-center gap-2'):
                                    ui.icon('my_location').classes('text-primary')
                                    ui.label('Click to add waypoints').classes('text-caption text-grey-7')
                            with ui.card().classes('w-full h-[500px] bg-white') as map_container:
                                map_display = ui.html('').style('width: 100%; height: 100%; overflow: hidden;')
                                # Add click handler with proper event handling
                                map_display.on('click', handle_map_click)

                    # Waypoints Panel - Below map
                    with ui.card().classes('col-span-12 bg-grey-1 q-pa-md'):
                        with ui.column().classes('gap-2'):
                            with ui.row().classes('items-center justify-between'):
                                ui.label('Waypoints').classes('text-h6 text-weight-medium')
                                with ui.row().classes('items-center gap-2'):
                                    ui.button('Clear All', icon='clear_all', on_click=lambda: clear_waypoints()).classes('bg-negative text-white')
                            waypoints_table = ui.table(
                                columns=[
                                    {'name': 'name', 'label': 'Name', 'field': 'name', 'align': 'left'},
                                    {'name': 'lat', 'label': 'Latitude', 'field': 'lat', 'align': 'left'},
                                    {'name': 'lon', 'label': 'Longitude', 'field': 'lon', 'align': 'left'},
                                    {'name': 'time', 'label': 'Time', 'field': 'time', 'align': 'left'},
                                    {'name': 'description', 'label': 'Description', 'field': 'description', 'align': 'left'},
                                    {'name': 'status', 'label': 'Status', 'field': 'status', 'align': 'left'},
                                    {'name': 'actions', 'label': 'Actions', 'field': 'actions', 'align': 'right'}
                                ],
                                rows=[],
                                row_key='name',
                                pagination={'rowsPerPage': 5}
                            ).classes('w-full')

        # Autopilot Section
        with ui.card().classes('col-span-12 bg-white q-mb-lg'):
            with ui.card().classes('q-pa-lg'):
                with ui.row().classes('items-center justify-between q-mb-md'):
                    with ui.row().classes('items-center gap-2'):
                        ui.icon('auto_mode').classes('text-primary text-h4')
                        ui.label('Autopilot').classes('text-h4 text-weight-bold text-primary')
                    autopilot_status = ui.label('Autopilot: Disabled').classes('text-h6 text-negative q-pa-sm rounded')
                
                with ui.row().classes('q-mb-md gap-4'):
                    autopilot_toggle = ui.switch('Enable Autopilot', on_change=lambda e: toggle_autopilot(e.value))
                    ui.button('Follow Waypoints', icon='route', on_click=lambda: start_follow_waypoints()).classes('bg-primary text-white')
                    ui.button('Stop Autopilot', icon='stop_circle', on_click=lambda: stop_autopilot()).classes('bg-negative text-white')
                    
                # Autopilot status display
                with ui.card().classes('col-span-12 bg-grey-1 q-pa-md'):
                    with ui.column().classes('gap-2'):
                        ui.label('Autopilot Status').classes('text-subtitle1 text-weight-medium')
                        autopilot_target = ui.label('Target Heading: --').classes('font-mono')
                        autopilot_error = ui.label('Heading Error: --').classes('font-mono')
                        autopilot_xte = ui.label('Cross Track Error: --').classes('font-mono')
                        autopilot_distance = ui.label('Distance to Waypoint: --').classes('font-mono')
                        autopilot_steering = ui.label('Steering Command: --').classes('font-mono')
                        autopilot_throttle = ui.label('Throttle Command: --').classes('font-mono')
                        
                        with ui.row().classes('items-center gap-2 q-mt-sm'):
                            ui.label('Manual Override:').classes('text-caption')
                            override_indicator = ui.badge('INACTIVE', color='grey')
                            ui.label('Speed Limit:').classes('text-caption q-ml-md')
                            speed_limit = ui.slider(min=0, max=100, value=70, on_change=lambda e: set_autopilot_speed(e.value)).classes('w-32')
                            speed_display = ui.label('70%').classes('text-caption w-14')

        # Geofencing Section
        with ui.card().classes('col-span-12 bg-white q-mb-lg'):
            with ui.card().classes('q-pa-lg'):
                with ui.row().classes('items-center justify-between q-mb-md'):
                    with ui.row().classes('items-center gap-2'):
                        ui.icon('fence').classes('text-primary text-h4')
                        ui.label('Geofencing').classes('text-h4 text-weight-bold text-primary')
                    geofence_status = ui.label('Geofence: Inactive').classes('text-h6 text-negative q-pa-sm rounded')
                
                with ui.row().classes('q-mb-md gap-4'):
                    geofence_toggle = ui.switch('Enable Geofence', on_change=lambda e: toggle_geofence(e.value))
                    ui.button('Draw on Map', icon='edit', on_click=lambda: start_geofence_drawing()).classes('bg-primary text-white')
                    ui.button('Clear Geofence', icon='delete', on_click=lambda: clear_geofence()).classes('bg-negative text-white')
                
                # Geofence management
                with ui.card().classes('col-span-12 bg-grey-1 q-pa-md'):
                    with ui.row().classes('items-center justify-between q-mb-sm'):
                        ui.label('Geofence Settings').classes('text-subtitle1 text-weight-medium')
                    
                    with ui.row().classes('items-center gap-2 q-mb-md'):
                        ui.label('Current Fence:').classes('text-caption')
                        current_fence_name = ui.label('None').classes('font-mono text-subtitle1')
                        
                    with ui.row().classes('items-center gap-2 q-mb-md'):
                        ui.label('Save Current Fence:').classes('text-caption')
                        fence_name_input = ui.input(placeholder='Fence name').classes('w-48')
                        ui.button('Save', icon='save', on_click=lambda: save_current_geofence()).classes('bg-positive text-white')
                    
                    with ui.row().classes('items-center gap-2'):
                        ui.label('Load Fence:').classes('text-caption')
                        fence_selector = ui.select(
                            options=[],
                            value=None,
                            on_change=lambda e: load_geofence(e.value) if e.value else None
                        ).classes('w-48')
                        ui.button('Delete', icon='delete', on_click=lambda: delete_selected_geofence()).classes('bg-negative text-white')
                    
                    with ui.row().classes('items-center gap-2 q-mt-md'):
                        ui.label('Status:').classes('text-caption')
                        fence_position_status = ui.label('Unknown').classes('font-mono text-subtitle1')
                    
                    # Hidden dialog for geofence drawing instructions
                    geofence_dialog = ui.dialog()
                    with geofence_dialog, ui.card().classes('w-96'):
                        ui.label('Draw Geofence').classes('text-h6 q-mb-md')
                        ui.label('Click on the map to add points to your geofence. Add at least 3 points to create a valid boundary. Click "Done" when finished.').classes('q-mb-md')
                        with ui.row().classes('w-full justify-end gap-2'):
                            ui.button('Cancel', on_click=geofence_dialog.close).classes('bg-grey-2')
                            ui.button('Done', on_click=lambda: finish_geofence_drawing()).classes('bg-primary text-white')

        # Left column - Analog inputs with visual indicators
        with ui.card().classes('col-span-4 bg-white'):
            with ui.card().classes('q-pa-md'):
                ui.label('Analog Control').classes('text-h6 text-weight-medium q-mb-md')
                
                # Left stick with visual indicator
                with ui.card().classes('q-pa-md q-mb-md bg-grey-1'):
                    ui.label('Left Stick').classes('text-subtitle1 text-weight-medium q-mb-sm')
                    with ui.row().classes('items-center justify-between'):
                        with ui.column().classes('items-center'):
                            ui.label('X').classes('text-caption text-grey-7')
                            ui.label().bind_text_from(controller.axis_states, 'ABS_X',
                                lambda x: f'{x:+d}').classes('text-h6 font-mono')
                        with ui.column().classes('items-center'):
                            ui.label('Y').classes('text-caption text-grey-7')
                            ui.label().bind_text_from(controller.axis_states, 'ABS_Y',
                                lambda x: f'{x:+d}').classes('text-h6 font-mono')
                    # Visual joystick indicator
                    with ui.card().classes('w-full h-32 bg-grey-2 q-mt-sm relative'):
                        joystick_indicator = ui.card().classes('absolute w-8 h-8 rounded-full bg-primary')
                        def update_joystick_pos():
                            x = (controller.axis_states['ABS_X'] + 100) / 200
                            y = (controller.axis_states['ABS_Y'] + 100) / 200
                            joystick_indicator.style(f'transform: translate({x * 100}%, {y * 100}%)')
                        ui.timer(0.1, update_joystick_pos)

                # Right stick with visual indicator
                with ui.card().classes('q-pa-md q-mb-md bg-grey-1'):
                    ui.label('Right Stick').classes('text-subtitle1 text-weight-medium q-mb-sm')
                    with ui.row().classes('items-center justify-between'):
                        with ui.column().classes('items-center'):
                            ui.label('X').classes('text-caption text-grey-7')
                            ui.label().bind_text_from(controller.axis_states, 'ABS_RX',
                                lambda x: f'{x:+d}').classes('text-h6 font-mono')
                        with ui.column().classes('items-center'):
                            ui.label('Y').classes('text-caption text-grey-7')
                            ui.label().bind_text_from(controller.axis_states, 'ABS_RY',
                                lambda x: f'{x:+d}').classes('text-h6 font-mono')
                    # Visual joystick indicator
                    with ui.card().classes('w-full h-32 bg-grey-2 q-mt-sm relative'):
                        joystick_indicator_r = ui.card().classes('absolute w-8 h-8 rounded-full bg-secondary')
                        def update_joystick_pos_r():
                            x = (controller.axis_states['ABS_RX'] + 100) / 200
                            y = (controller.axis_states['ABS_RY'] + 100) / 200
                            joystick_indicator_r.style(f'transform: translate({x * 100}%, {y * 100}%)')
                        ui.timer(0.1, update_joystick_pos_r)

                # Triggers with progress bars
                with ui.card().classes('q-pa-md bg-grey-1'):
                    ui.label('Triggers').classes('text-subtitle1 text-weight-medium q-mb-sm')
                    with ui.column().classes('gap-2'):
                        with ui.row().classes('items-center gap-2'):
                            ui.label('Left').classes('text-caption text-grey-7 w-16')
                            trigger_l = ui.linear_progress().classes('flex-grow')
                            trigger_l_label = ui.label().classes('text-caption w-12 text-right')
                        with ui.row().classes('items-center gap-2'):
                            ui.label('Right').classes('text-caption text-grey-7 w-16')
                            trigger_r = ui.linear_progress().classes('flex-grow')
                            trigger_r_label = ui.label().classes('text-caption w-12 text-right')
                        
                        def update_triggers():
                            l_val = controller.axis_states['ABS_Z']
                            r_val = controller.axis_states['ABS_RZ']
                            trigger_l.set_value(l_val)
                            trigger_r.set_value(r_val)
                            trigger_l_label.set_text(f'{l_val}%')
                            trigger_r_label.set_text(f'{r_val}%')
                        ui.timer(0.1, update_triggers)

        # Middle column - Buttons with improved styling
        with ui.card().classes('col-span-4 bg-white'):
            with ui.card().classes('q-pa-md'):
                ui.label('Digital Control').classes('text-h6 text-weight-medium q-mb-md')
                
                # Face buttons with modern styling
                with ui.card().classes('q-pa-md q-mb-md bg-grey-1'):
                    ui.label('Face Buttons').classes('text-subtitle1 text-weight-medium q-mb-sm')
                    with ui.grid(columns=2).classes('gap-4'):
                        for btn, code in [('A', 'BTN_SOUTH'), ('B', 'BTN_EAST'), 
                                        ('X', 'BTN_WEST'), ('Y', 'BTN_NORTH')]:
                            with ui.card().classes('q-pa-sm'):
                                ui.label(btn).classes('text-weight-medium text-center')
                                btn_indicator = ui.label().bind_text_from(controller.button_states, code,
                                    lambda x: 'Pressed' if x else 'Released')
                                btn_indicator.classes('text-center text-caption')

                # Bumpers with modern styling
                with ui.card().classes('q-pa-md q-mb-md bg-grey-1'):
                    ui.label('Bumpers').classes('text-subtitle1 text-weight-medium q-mb-sm')
                    with ui.grid(columns=2).classes('gap-4'):
                        for btn, code in [('Left', 'BTN_TL'), ('Right', 'BTN_TR')]:
                            with ui.card().classes('q-pa-sm'):
                                ui.label(btn).classes('text-weight-medium text-center')
                                btn_indicator = ui.label().bind_text_from(controller.button_states, code,
                                    lambda x: 'Pressed' if x else 'Released')
                                btn_indicator.classes('text-center text-caption')

                # D-pad with modern styling
                with ui.card().classes('q-pa-md q-mb-md bg-grey-1'):
                    ui.label('D-Pad').classes('text-subtitle1 text-weight-medium q-mb-sm')
                    with ui.grid(columns=2).classes('gap-4'):
                        for btn, code, val in [('Up', 'ABS_HAT0Y', -1), ('Down', 'ABS_HAT0Y', 1),
                                             ('Left', 'ABS_HAT0X', -1), ('Right', 'ABS_HAT0X', 1)]:
                            with ui.card().classes('q-pa-sm'):
                                ui.label(btn).classes('text-weight-medium text-center')
                                btn_indicator = ui.label().bind_text_from(controller.button_states, code,
                                    lambda x, v=val: 'Pressed' if x == v else 'Released')
                                btn_indicator.classes('text-center text-caption')

                # Menu buttons with modern styling
                with ui.card().classes('q-pa-md bg-grey-1'):
                    ui.label('Menu Buttons').classes('text-subtitle1 text-weight-medium q-mb-sm')
                    with ui.grid(columns=2).classes('gap-4'):
                        for btn, code in [('Select', 'BTN_SELECT'), ('Start', 'BTN_START')]:
                            with ui.card().classes('q-pa-sm'):
                                ui.label(btn).classes('text-weight-medium text-center')
                                btn_indicator = ui.label().bind_text_from(controller.button_states, code,
                                    lambda x: 'Pressed' if x else 'Released')
                                btn_indicator.classes('text-center text-caption')

        # Right column - Event history with improved styling
        with ui.card().classes('col-span-4 bg-white'):
            with ui.card().classes('q-pa-md'):
                with ui.row().classes('items-center justify-between q-mb-md'):
                    ui.label('Event Log').classes('text-h6 text-weight-medium')
                    ui.button('Clear', icon='clear_all', on_click=lambda: setattr(controller, 'events_history', deque(maxlen=100))).classes('bg-grey-2')
                
                event_table = ui.table(
                    columns=[
                        {'name': 'time', 'label': 'Time', 'field': 'time', 'align': 'left', 'sortable': True},
                        {'name': 'type', 'label': 'Type', 'field': 'type', 'align': 'left', 'sortable': True},
                        {'name': 'code', 'label': 'Code', 'field': 'code', 'align': 'left', 'sortable': True},
                        {'name': 'state', 'label': 'State', 'field': 'state', 'align': 'left', 'sortable': True}
                    ],
                    rows=[],
                    row_key='time',
                    pagination={'rowsPerPage': 10},
                    selection='none'
                ).classes('w-full')

                # Last message display with improved styling
                with ui.card().classes('q-mt-md q-pa-sm bg-grey-1'):
                    ui.label('Last Serial Message').classes('text-subtitle2 text-weight-medium q-mb-xs')
                    last_message = ui.label('No messages sent').classes('font-mono text-caption bg-grey-2 q-pa-sm rounded')

        # Add GPS and Map column
        with ui.card().classes('col-span-12 bg-white'):
            with ui.card().classes('q-pa-md'):
                ui.label('GPS Status').classes('text-h6 text-weight-medium q-mb-md')
                
                # GPS Status Display
                with ui.grid(columns=2).classes('gap-4'):
                    with ui.card().classes('col-span-1 bg-grey-1'):
                        with ui.column().classes('gap-2'):
                            ui.label('GPS Data').classes('text-subtitle1 text-weight-medium')
                            gps_status = ui.label('No GPS Fix').classes('text-negative')
                            gps_lat = ui.label('Latitude: --').classes('font-mono')
                            gps_lon = ui.label('Longitude: --').classes('font-mono')
                            gps_alt = ui.label('Altitude: --').classes('font-mono')
                            gps_speed = ui.label('Speed: --').classes('font-mono')
                            gps_sats = ui.label('Satellites: --').classes('font-mono')
                            gps_hdop = ui.label('HDOP: --').classes('font-mono text-subtitle1')
                            gps_time = ui.label('Last Update: --').classes('font-mono text-subtitle1')

                    with ui.card().classes('col-span-1 bg-grey-1'):
                        ui.label('Map').classes('text-subtitle1 text-weight-medium')
                        with ui.card().classes('w-full h-96 bg-white') as map_container:
                            map_display = ui.html('').style('width: 100%; height: 100%; overflow: hidden;')

                def update_gps_display():
                    gps = controller.serial.gps_data
                    if gps and gps.has_fix:  # Check if gps exists and has fix
                        gps_status.set_text('GPS Fix Active')
                        gps_status.classes('text-positive bg-positive-1')
                        gps_lat.set_text(f'Latitude: {gps.latitude:.6f}°')
                        gps_lon.set_text(f'Longitude: {gps.longitude:.6f}°')
                        gps_alt.set_text(f'Altitude: {gps.altitude:.1f}m')
                        gps_speed.set_text(f'Speed: {gps.speed:.1f} km/h')
                        gps_sats.set_text(f'Satellites: {gps.satellites}')
                        gps_hdop.set_text(f'HDOP: {gps.hdop:.1f}')
                        if gps.last_update:
                            gps_time.set_text(f'Last Update: {gps.last_update.strftime("%H:%M:%S")}')
                        
                        # Update compass data if available
                        if gps.compass_ready:
                            compass_status.set_text('Compass: Active')
                            compass_status.classes('text-positive')
                            compass_heading.set_text(f'Heading: {gps.compass_heading:.1f}°')
                            compass_pitch.set_text(f'Pitch: {gps.compass_pitch:.1f}°')
                            compass_roll.set_text(f'Roll: {gps.compass_roll:.1f}°')
                            if gps.compass_last_update:
                                compass_time.set_text(f'Last Update: {gps.compass_last_update.strftime("%H:%M:%S")}')
                            
                            # Update calibration status indicators (values are 0-3, convert to 0-1 for progress bars)
                            system_cal_indicator.set_value(gps.compass_system_cal / 3.0)
                            system_cal_indicator.classes(f'bg-{get_cal_color(gps.compass_system_cal)}')
                            
                            gyro_cal_indicator.set_value(gps.compass_gyro_cal / 3.0)
                            gyro_cal_indicator.classes(f'bg-{get_cal_color(gps.compass_gyro_cal)}')
                            
                            accel_cal_indicator.set_value(gps.compass_accel_cal / 3.0)
                            accel_cal_indicator.classes(f'bg-{get_cal_color(gps.compass_accel_cal)}')
                            
                            mag_cal_indicator.set_value(gps.compass_mag_cal / 3.0)
                            mag_cal_indicator.classes(f'bg-{get_cal_color(gps.compass_mag_cal)}')
                        else:
                            compass_status.set_text('Compass: Not Ready')
                            compass_status.classes('text-negative')
                            compass_heading.set_text('Heading: --')
                            compass_pitch.set_text('Pitch: --')
                            compass_roll.set_text('Roll: --')
                            
                            # Reset calibration indicators
                            system_cal_indicator.set_value(0)
                            gyro_cal_indicator.set_value(0)
                            accel_cal_indicator.set_value(0)
                            mag_cal_indicator.set_value(0)

                        try:
                            if controller.serial.map_html:  # Check if map_html exists
                                map_display.set_content(controller.serial.map_html)
                            else:
                                map_display.set_content(
                                    '<div style="display: flex; justify-content: center; align-items: center; '
                                    'height: 100%; background-color: #f5f5f5; color: #666; font-size: 1.2em;">'
                                    'Map data not available</div>'
                                )
                        except Exception as e:
                            print(f"Error updating map: {e}")
                            map_display.set_content(
                                '<div style="display: flex; justify-content: center; align-items: center; '
                                'height: 100%; background-color: #f5f5f5; color: #F44336; font-size: 1.2em;">'
                                'Error displaying map</div>'
                            )

                        # Update route information
                        distance, bearing, progress = gps.get_route_info(gps.latitude, gps.longitude)
                        route_distance.set_text(f'Distance to next: {distance:.1f}m')
                        route_bearing.set_text(f'Bearing to next: {bearing:.1f}°')
                        route_progress.set_text(f'Route progress: {progress:.1f}%')

                        # Update waypoints table
                        update_waypoints_table()

                    else:
                        gps_status.set_text('No GPS Fix')
                        gps_status.classes('text-negative bg-negative-1')
                        gps_lat.set_text('Latitude: --')
                        gps_lon.set_text('Longitude: --')
                        gps_alt.set_text('Altitude: --')
                        gps_speed.set_text('Speed: --')
                        gps_sats.set_text('Satellites: --')
                        gps_hdop.set_text('HDOP: --')
                        if gps and gps.last_update:  # Check if gps exists
                            gps_time.set_text(f'Last Update: {gps.last_update.strftime("%H:%M:%S")}')
                        else:
                            gps_time.set_text('Last Update: --')
                        
                        # Also update compass section even if no GPS fix
                        if gps and gps.compass_ready:
                            compass_status.set_text('Compass: Active')
                            compass_status.classes('text-positive')
                            compass_heading.set_text(f'Heading: {gps.compass_heading:.1f}°')
                            compass_pitch.set_text(f'Pitch: {gps.compass_pitch:.1f}°')
                            compass_roll.set_text(f'Roll: {gps.compass_roll:.1f}°')
                            if gps.compass_last_update:
                                compass_time.set_text(f'Last Update: {gps.compass_last_update.strftime("%H:%M:%S")}')
                            
                            # Update calibration status indicators
                            system_cal_indicator.set_value(gps.compass_system_cal / 3.0)
                            system_cal_indicator.classes(f'bg-{get_cal_color(gps.compass_system_cal)}')
                            
                            gyro_cal_indicator.set_value(gps.compass_gyro_cal / 3.0)
                            gyro_cal_indicator.classes(f'bg-{get_cal_color(gps.compass_gyro_cal)}')
                            
                            accel_cal_indicator.set_value(gps.compass_accel_cal / 3.0)
                            accel_cal_indicator.classes(f'bg-{get_cal_color(gps.compass_accel_cal)}')
                            
                            mag_cal_indicator.set_value(gps.compass_mag_cal / 3.0)
                            mag_cal_indicator.classes(f'bg-{get_cal_color(gps.compass_mag_cal)}')
                        else:
                            compass_status.set_text('Compass: Not Ready')
                            compass_status.classes('text-negative')
                            compass_heading.set_text('Heading: --')
                            compass_pitch.set_text('Pitch: --')
                            compass_roll.set_text('Roll: --')
                            compass_time.set_text('Last Update: --')
                            
                            # Reset calibration indicators
                            system_cal_indicator.set_value(0)
                            gyro_cal_indicator.set_value(0)
                            accel_cal_indicator.set_value(0)
                            mag_cal_indicator.set_value(0)
                        
                        # Show "No GPS Fix" message with improved styling
                        map_display.set_content(
                            '<div style="display: flex; flex-direction: column; justify-content: center; align-items: center; '
                            'height: 100%; background-color: #f5f5f5; color: #666; font-size: 1.2em; gap: 1rem;">'
                            '<i class="material-icons" style="font-size: 48px; color: #F44336;">gps_off</i>'
                            '<div>No GPS Fix Available</div>'
                            '<div style="font-size: 0.9em; color: #999;">Waiting for satellite connection...</div>'
                            '</div>'
                        )

                    # Add waypoints table update
                    update_waypoints_table()

                # Update GPS display every second
                ui.timer(1.0, update_gps_display)

                # Update GPS data reading every 100ms
                def read_gps():
                    controller.serial.read_serial_data()
                ui.timer(0.1, read_gps)

def return_to_home():
    gps = controller.serial.gps_data
    if gps.home_set and gps.home_position:
        wp = gps.trigger_return_to_home()
        if wp:
            controller.serial.update_map()
            ui.notify('Return to Home waypoint added!', type='positive')
        else:
            ui.notify('Home position not set.', type='warning')
    else:
        ui.notify('No home position available yet. Wait for GPS fix.', type='warning')

async def update_ui():
    while True:
        with controller.lock:
            event_table.rows = list(controller.events_history)
        await asyncio.sleep(0.1)

# Start the UI update loop
ui.timer(0.1, update_ui)

# Cleanup on shutdown
@app.on_shutdown
def shutdown():
    controller.stop_monitoring()

# Add a helper function to get appropriate color based on calibration level
def get_cal_color(cal_level):
    """Return appropriate color class based on calibration level (0-3)"""
    if cal_level == 0:
        return 'negative'
    elif cal_level == 1:
        return 'warning'
    elif cal_level == 2:
        return 'info'
    else:  # Level 3 (fully calibrated)
        return 'positive'

# Add autopilot functions
def toggle_autopilot(value):
    """Toggle autopilot mode on/off."""
    controller.serial.gps_data.autopilot_active = value
    if value:
        autopilot_status.set_text('Autopilot: Enabled')
        autopilot_status.classes('text-h6 text-positive q-pa-sm rounded')
        ui.notify('Autopilot enabled', type='positive')
    else:
        autopilot_status.set_text('Autopilot: Disabled')
        autopilot_status.classes('text-h6 text-negative q-pa-sm rounded')
        ui.notify('Autopilot disabled', type='warning')
        # Reset override state
        controller.autopilot_override = False

def start_follow_waypoints():
    """Start following waypoints with autopilot."""
    gps = controller.serial.gps_data
    if not gps.waypoints:
        ui.notify('No waypoints to follow', type='warning')
        return
        
    # Reset completion status of waypoints
    for wp in gps.waypoints:
        wp.completed = False
    gps.active_waypoint_index = 0
    
    # Enable autopilot
    gps.autopilot_active = True
    autopilot_toggle.set_value(True)
    autopilot_status.set_text('Autopilot: Following Waypoints')
    autopilot_status.classes('text-h6 text-positive q-pa-sm rounded')
    ui.notify('Autopilot following waypoints', type='positive')

def stop_autopilot():
    """Stop autopilot and return to manual control."""
    controller.serial.gps_data.autopilot_active = False
    autopilot_toggle.set_value(False)
    autopilot_status.set_text('Autopilot: Disabled')
    autopilot_status.classes('text-h6 text-negative q-pa-sm rounded')
    ui.notify('Autopilot stopped', type='warning')
    # Reset override state
    controller.autopilot_override = False

def set_autopilot_speed(value):
    """Set the autopilot speed limit."""
    controller.serial.gps_data.autopilot_speed_limit = value
    speed_display.set_text(f"{value}%")

# Update the UI update timer to also update autopilot calculations
def update_autopilot():
    """Update autopilot calculations and UI."""
    gps = controller.serial.gps_data
    if not gps or not gps.compass_ready or not gps.has_fix:
        return
        
    if gps.autopilot_active:
        # Calculate steering commands
        current_heading = gps.compass_heading if gps.compass_heading is not None else 0
        steering = gps.calculate_pure_pursuit_steering(gps.latitude, gps.longitude, current_heading)
        
        # Update UI
        if gps.autopilot_target_heading is not None:
            autopilot_target.set_text(f"Target Heading: {gps.autopilot_target_heading:.1f}°")
            
            # Calculate heading error
            heading_error = gps.autopilot_target_heading - current_heading
            # Normalize to -180 to 180
            heading_error = (heading_error + 180) % 360 - 180
            autopilot_error.set_text(f"Heading Error: {heading_error:.1f}°")
        else:
            autopilot_target.set_text("Target Heading: --")
            autopilot_error.set_text("Heading Error: --")
            
        # Update cross-track error
        autopilot_xte.set_text(f"Cross Track Error: {gps.autopilot_cross_track_error:.1f}m")
        
        # Update distance to waypoint
        autopilot_distance.set_text(f"Distance to Waypoint: {gps.autopilot_distance_to_waypoint:.1f}m")
        
        # Update steering and throttle commands
        autopilot_steering.set_text(f"Steering Command: {gps.autopilot_steering_angle}")
        autopilot_throttle.set_text(f"Throttle Command: {gps.autopilot_throttle}")
        
        # Update override indicator
        if controller.autopilot_override:
            override_indicator.set_text("ACTIVE")
            override_indicator.props(f"color=warning")
        else:
            override_indicator.set_text("INACTIVE")
            override_indicator.props(f"color=grey")
    else:
        # Reset displays when not active
        autopilot_target.set_text("Target Heading: --")
        autopilot_error.set_text("Heading Error: --")
        autopilot_xte.set_text("Cross Track Error: --")
        autopilot_distance.set_text("Distance to Waypoint: --")
        autopilot_steering.set_text("Steering Command: --")
        autopilot_throttle.set_text("Throttle Command: --")
        override_indicator.set_text("INACTIVE")
        override_indicator.props(f"color=grey")

# Add autopilot update timer
ui.timer(0.2, update_autopilot)  # Update autopilot calculations 5 times per second

# Add functions for geofence management
drawing_geofence = False
temp_geofence_points = []
temp_geofence_markers = []

def toggle_geofence(value):
    """Toggle geofence active state."""
    gps = controller.serial.gps_data
    if value:
        # Try to activate current geofence
        if gps.activate_geofence():
            geofence_status.set_text('Geofence: Active')
            geofence_status.classes('text-h6 text-positive q-pa-sm rounded')
            ui.notify('Geofence activated', type='positive')
            update_geofence_display()
        else:
            geofence_toggle.set_value(False)
            ui.notify('No valid geofence defined', type='warning')
    else:
        gps.deactivate_geofence()
        geofence_status.set_text('Geofence: Inactive')
        geofence_status.classes('text-h6 text-negative q-pa-sm rounded')
        ui.notify('Geofence deactivated', type='warning')
        fence_position_status.set_text('Unknown')

def start_geofence_drawing():
    """Start drawing geofence on the map."""
    global drawing_geofence, temp_geofence_points, temp_geofence_markers
    
    gps = controller.serial.gps_data
    if not gps.has_fix:
        ui.notify('Need GPS fix to draw geofence', type='warning')
        return
    
    # Reset temporary storage
    drawing_geofence = True
    temp_geofence_points = []
    temp_geofence_markers = []
    
    # Show instructions
    geofence_dialog.open()
    
    # Update map to show we're in drawing mode
    controller.serial.update_map()
    
    ui.notify('Click on map to add geofence points', type='info', timeout=5000)

def handle_map_click(e):
    """Handle map click events for waypoints or geofence drawing."""
    # Check if we're in geofence drawing mode
    global drawing_geofence, temp_geofence_points, temp_geofence_markers
    
    if drawing_geofence:
        try:
            # Extract coordinates from the click event
            lat = float(e.args.get('lat', 0))
            lon = float(e.args.get('lng', 0))
            
            # Validate coordinates
            if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                ui.notify('Invalid coordinates received', type='negative')
                return
            
            # Add point to temporary geofence
            temp_geofence_points.append((lat, lon))
            
            # Update map to show the point
            controller.serial.update_map()
            
            ui.notify(f'Added point {len(temp_geofence_points)}: {lat:.6f}, {lon:.6f}', type='info')
            
        except (ValueError, TypeError, AttributeError) as e:
            print(f"Error handling map click for geofence: {e}")
            ui.notify('Error adding geofence point', type='negative')
    else:
        # Regular waypoint handling
        if not controller.serial.gps_data or not controller.serial.gps_data.has_fix:
            ui.notify('No GPS signal available. Please wait for GPS fix.', type='warning')
            return

        try:
            # Extract coordinates from the click event
            lat = float(e.args.get('lat', 0))
            lon = float(e.args.get('lng', 0))
            
            # Validate coordinates
            if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                ui.notify('Invalid coordinates received', type='negative')
                return

            # Create waypoint dialog
            dialog = ui.dialog()
            with dialog, ui.card().classes('w-96'):
                ui.label('Add Waypoint').classes('text-h6 q-mb-md')
                description = ui.input('Description (optional)').classes('w-full')
                with ui.row().classes('w-full justify-end gap-2'):
                    ui.button('Cancel', on_click=dialog.close).classes('bg-grey-2')
                    ui.button('Add', on_click=lambda: add_waypoint(lat, lon, description.value, dialog)).classes('bg-primary text-white')
        except (ValueError, TypeError, AttributeError) as e:
            print(f"Error handling map click: {e}")
            ui.notify('Error processing map click', type='negative')

def finish_geofence_drawing():
    """Finish drawing geofence on the map."""
    global drawing_geofence, temp_geofence_points
    
    geofence_dialog.close()
    
    if len(temp_geofence_points) < 3:
        ui.notify('Need at least 3 points for a valid geofence', type='warning')
        drawing_geofence = False
        temp_geofence_points = []
        controller.serial.update_map()
        return
    
    # Close the polygon by connecting back to the first point
    if temp_geofence_points[0] != temp_geofence_points[-1]:
        temp_geofence_points.append(temp_geofence_points[0])
    
    # Set the geofence
    gps = controller.serial.gps_data
    if gps.set_geofence(temp_geofence_points):
        ui.notify('Geofence created successfully', type='positive')
        current_fence_name.set_text(gps.geofence_name)
        
        # Update fence selector
        update_fence_selector()
    else:
        ui.notify('Failed to create geofence', type='negative')
    
    # Exit drawing mode
    drawing_geofence = False
    temp_geofence_points = []
    
    # Update map to show the new geofence
    controller.serial.update_map()

def clear_geofence():
    """Clear the current geofence."""
    gps = controller.serial.gps_data
    gps.geofence_points = []
    gps.geofence_polygon = None
    gps.geofence_active = False
    geofence_toggle.set_value(False)
    geofence_status.set_text('Geofence: Inactive')
    geofence_status.classes('text-h6 text-negative q-pa-sm rounded')
    current_fence_name.set_text('None')
    ui.notify('Geofence cleared', type='info')
    
    # Update map to remove geofence
    controller.serial.update_map()

def save_current_geofence():
    """Save the current geofence with a name."""
    gps = controller.serial.gps_data
    name = fence_name_input.value
    
    if not name:
        ui.notify('Please enter a name for the geofence', type='warning')
        return
    
    if not gps.geofence_points or len(gps.geofence_points) < 3:
        ui.notify('No valid geofence to save', type='warning')
        return
    
    # Save with new name
    gps.set_geofence(gps.geofence_points, name)
    current_fence_name.set_text(name)
    fence_name_input.set_value('')
    
    # Update fence selector
    update_fence_selector()
    
    ui.notify(f'Geofence "{name}" saved', type='positive')

def load_geofence(name):
    """Load a saved geofence."""
    if not name:
        return
    
    gps = controller.serial.gps_data
    if name in gps.available_geofences:
        points = gps.available_geofences[name]
        if gps.set_geofence(points, name):
            current_fence_name.set_text(name)
            ui.notify(f'Geofence "{name}" loaded', type='positive')
            
            # Update map to show the geofence
            controller.serial.update_map()
            
            # If geofence was active, update status with new geofence
            if gps.geofence_active:
                gps.activate_geofence()
                update_geofence_display()
        else:
            ui.notify(f'Failed to load geofence "{name}"', type='negative')

def delete_selected_geofence():
    """Delete the selected geofence."""
    name = fence_selector.value
    if not name:
        ui.notify('No geofence selected', type='warning')
        return
    
    gps = controller.serial.gps_data
    if gps.delete_geofence(name):
        ui.notify(f'Geofence "{name}" deleted', type='info')
        
        # If this was the current fence, update display
        if name == gps.geofence_name:
            current_fence_name.set_text('None')
            geofence_toggle.set_value(False)
            geofence_status.set_text('Geofence: Inactive')
            geofence_status.classes('text-h6 text-negative q-pa-sm rounded')
        
        # Update fence selector
        update_fence_selector()
        fence_selector.set_value(None)
        
        # Update map
        controller.serial.update_map()
    else:
        ui.notify(f'Failed to delete geofence "{name}"', type='negative')

def update_fence_selector():
    """Update the fence selector with available geofences."""
    gps = controller.serial.gps_data
    fence_selector.options = [{'label': name, 'value': name} for name in gps.available_geofences.keys()]

def update_geofence_display():
    """Update geofence status display."""
    gps = controller.serial.gps_data
    if gps.geofence_active:
        if gps.inside_geofence:
            fence_position_status.set_text('Inside Fence')
            fence_position_status.classes('font-mono text-subtitle1 text-positive')
        else:
            fence_position_status.set_text('OUTSIDE FENCE!')
            fence_position_status.classes('font-mono text-subtitle1 text-negative font-weight-bold')
            
            # Show alert for geofence violation with detailed safety information
            if gps.geofence_violation:
                ui.notify(
                    'GEOFENCE VIOLATION! Multiple safety measures activated:\n'
                    '1. Robot disarmed\n'
                    '2. Emergency stop signal sent\n'
                    '3. All movement commands blocked',
                    type='negative',
                    timeout=0,  # Don't auto-close
                    position='center',  # Center of screen
                    classes='text-h6'  # Larger text
                )
    else:
        fence_position_status.set_text('Unknown')
        fence_position_status.classes('font-mono text-subtitle1')

# Load geofences on startup
def initialize_geofence_ui():
    """Initialize geofence UI with saved geofences."""
    update_fence_selector()
    
    # Check if there's an active geofence to display
    gps = controller.serial.gps_data
    if gps.geofence_name != "Default Fence" and gps.geofence_points:
        current_fence_name.set_text(gps.geofence_name)
    else:
        current_fence_name.set_text('None')

# Call initialization after UI is built
initialize_geofence_ui()

if __name__ in {"__main__", "__mp_main__"}:
    ui.run()

