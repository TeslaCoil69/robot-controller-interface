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

    def read_gps_data(self):
        """Read and parse GPS data from serial port"""
        if not self.connected or not self.serial or not self.serial.is_open:
            return None

        try:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8').strip()
                if line.startswith('GPS,'):
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
        except Exception as e:
            print(f"Error reading GPS data: {e}")
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

        # Add click handler for waypoints
        m.get_root().html.add_child(folium.Element("""
            <script>
                var map = document.querySelector('#map');
                map.addEventListener('click', function(e) {
                    var lat = e.latlng.lat;
                    var lng = e.latlng.lng;
                    window.pywebview.api.on_map_click(lat, lng);
                });
            </script>
        """))

        # Save the map and store its HTML content
        m.save(self.map_file)
        with open(self.map_file, 'r') as f:
            self.map_html = f.read()

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

        # Format: LX,LY,RX,RY,LT,RT,A,B,X,Y,LB,RB,SELECT,START,DPAD_UP,DPAD_DOWN,DPAD_LEFT,DPAD_RIGHT
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
                            gps_hdop = ui.label('HDOP: --').classes('font-mono')
                            gps_time = ui.label('Last Update: --').classes('font-mono')

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
                    controller.serial.read_gps_data()
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

if __name__ in {"__main__", "__mp_main__"}:
    ui.run()

