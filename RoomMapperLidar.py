import pygame
import sys
import math
import time
from rplidar import RPLidar
import numpy as np

class RoomMapperLidar:
    def __init__(self, port='COM3', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.lidar = None
        self.is_scanning = False
        
        # Pygame settings
        self.screen_size = 800
        self.screen = None
        self.center_x = self.screen_size // 2
        self.center_y = self.screen_size // 2
        self.scale = 0.3
        self.point_size = 4
        
        # Colors
        self.bg_color = (10, 10, 40)
        self.point_color = (0, 255, 255)
        self.robot_color = (255, 50, 50)
        self.axis_color = (50, 255, 50)
        self.text_color = (255, 255, 100)
        self.ray_color = (80, 80, 120)
        self.measure_color = (255, 255, 0)  # Yellow for measurements
        
        # Data storage
        self.all_points = []
        self.scan_count = 0
        self.start_time = time.time()
        self.font = None
        
        # Room measurement
        self.show_measurements = True
        self.room_data = None
        
    def connect(self):
        """Connect to RPLidar"""
        try:
            print(f"🔄 Connecting to RPLidar on {self.port}...")
            self.lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=3)
            time.sleep(1)
            
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            print("✅ Connected to RPLidar successfully!")
            print(f"   Model: {info['model']}")
            print(f"   Firmware: {info['firmware']}")
            print(f"   Health: {health}")
            
            return True
            
        except Exception as e:
            print(f"❌ Connection error: {e}")
            return False
    
    def init_pygame(self):
        """Initialize Pygame"""
        try:
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_size, self.screen_size))
            pygame.display.set_caption("RPLidar Room Mapper - ESC: Exit | M: Measurements | +/-: Zoom")
            self.font = pygame.font.SysFont('Arial', 16, bold=True)
            self.large_font = pygame.font.SysFont('Arial', 20, bold=True)
            print("✅ Pygame initialized successfully!")
            return True
        except Exception as e:
            print(f"❌ Pygame init error: {e}")
            return False
    
    def start_scan(self):
        """Start scanning"""
        try:
            print("🚀 Starting scan...")
            self.is_scanning = True
            
            # Start scanning in separate thread
            import threading
            scan_thread = threading.Thread(target=self.scan_worker, daemon=True)
            scan_thread.start()
            
            # Main display loop
            clock = pygame.time.Clock()
            while self.is_scanning:
                self.room_data = self.measure_room()  # Update measurements
                self.draw_frame()
                if self.handle_events():
                    break
                clock.tick(30)
                
        except Exception as e:
            print(f"❌ Scan error: {e}")
        finally:
            self.stop_scan()
    
    def scan_worker(self):
        """Worker thread for scanning"""
        try:
            print("📡 Starting scan worker...")
            scan_num = 0
            
            for scan in self.lidar.iter_scans():
                if not self.is_scanning:
                    break
                
                new_points = []
                valid_points = 0
                
                for quality, angle, distance in scan:
                    if distance > 0:
                        angle_rad = np.radians(angle)
                        x = distance * np.cos(angle_rad)
                        y = distance * np.sin(angle_rad)
                        new_points.append((x, y))
                        valid_points += 1
                
                if new_points:
                    self.all_points.extend(new_points)
                    # Keep only last 3000 points for performance
                    if len(self.all_points) > 3000:
                        self.all_points = self.all_points[-3000:]
                    
                    self.scan_count += 1
                    scan_num += 1
                    
                    if scan_num % 20 == 0:
                        print(f"📊 Scan {scan_num}: {valid_points} points")
                        
        except Exception as e:
            print(f"❌ Scan worker error: {e}")
            self.is_scanning = False
    
    def measure_room(self):
        """Calculate room dimensions and additional metrics"""
        if len(self.all_points) > 100:
            xs = [p[0] for p in self.all_points]
            ys = [p[1] for p in self.all_points]
            
            width = max(xs) - min(xs)
            height = max(ys) - min(ys)
            
            # Additional metrics
            room_area = width * height / 1000000  # Convert to m²
            perimeter = 2 * (width + height)
            center_x = (max(xs) + min(xs)) / 2
            center_y = (max(ys) + min(ys)) / 2
            
            return {
                'width': width,
                'height': height, 
                'area': room_area,
                'perimeter': perimeter,
                'center': (center_x, center_y),
                'bounds': (min(xs), min(ys), max(xs), max(ys)),
                'point_count': len(self.all_points)
            }
        return None
    
    def draw_room_measurements(self):
        """Draw room measurements on the screen"""
        if not self.room_data or not self.show_measurements:
            return
            
        min_x, min_y, max_x, max_y = self.room_data['bounds']
        
        # Convert bounds to screen coordinates
        screen_min_x = int(min_x * self.scale) + self.center_x
        screen_min_y = int(min_y * self.scale) + self.center_y
        screen_max_x = int(max_x * self.scale) + self.center_x
        screen_max_y = int(max_y * self.scale) + self.center_y
        
        # Draw bounding box
        rect = pygame.Rect(screen_min_x, screen_min_y, 
                          screen_max_x - screen_min_x, 
                          screen_max_y - screen_min_y)
        pygame.draw.rect(self.screen, self.measure_color, rect, 3)
        
        # Draw measurement labels with better formatting
        width_m = self.room_data['width'] / 1000.0
        height_m = self.room_data['height'] / 1000.0
        area_m2 = self.room_data['area']
        
        measurements = [
            f"ROOM MEASUREMENTS:",
            f"Width: {width_m:.2f} m",
            f"Height: {height_m:.2f} m", 
            f"Area: {area_m2:.2f} m²",
            f"Points: {self.room_data['point_count']}",
            f"Scans: {self.scan_count}"
        ]
        
        # Draw measurement box background
        measure_bg = pygame.Rect(self.screen_size - 220, 80, 210, 150)
        pygame.draw.rect(self.screen, (0, 0, 0, 180), measure_bg)
        pygame.draw.rect(self.screen, self.measure_color, measure_bg, 2)
        
        # Draw measurement text
        for i, text in enumerate(measurements):
            color = self.measure_color if i == 0 else self.text_color
            font = self.large_font if i == 0 else self.font
            text_surface = font.render(text, True, color)
            self.screen.blit(text_surface, (self.screen_size - 210, 90 + i * 25))
    
    def draw_frame(self):
        """Draw one frame with all points"""
        # Clear screen
        self.screen.fill(self.bg_color)
        
        # Draw distance circles
        self.draw_distance_circles()
        
        # Draw axes
        pygame.draw.line(self.screen, self.axis_color, 
                        (self.center_x, 0), (self.center_x, self.screen_size), 1)
        pygame.draw.line(self.screen, self.axis_color, 
                        (0, self.center_y), (self.screen_size, self.center_y), 1)
        
        # Draw all points
        if self.all_points:
            for x, y in self.all_points:
                screen_x = int(x * self.scale) + self.center_x
                screen_y = int(y * self.scale) + self.center_y
                
                if 0 <= screen_x < self.screen_size and 0 <= screen_y < self.screen_size:
                    pygame.draw.circle(self.screen, self.point_color, 
                                     (screen_x, screen_y), self.point_size)
        
        # Draw robot (center)
        pygame.draw.circle(self.screen, self.robot_color, 
                         (self.center_x, self.center_y), 12)
        pygame.draw.circle(self.screen, (255, 255, 255), 
                         (self.center_x, self.center_y), 12, 2)
        
        # Draw room measurements
        self.draw_room_measurements()
        
        # Draw basic statistics
        self.draw_stats()
        
        # Update display
        pygame.display.flip()
    
    def draw_distance_circles(self):
        """Draw distance reference circles"""
        distances = [1000, 2000, 3000, 4000, 5000]  # mm
        for dist in distances:
            radius = int(dist * self.scale)
            pygame.draw.circle(self.screen, (50, 50, 80), 
                             (self.center_x, self.center_y), radius, 1)
            
            # Add distance labels
            if dist in [2000, 4000]:  # Label only some circles to avoid clutter
                text = self.font.render(f"{dist/1000:.1f}m", True, (100, 100, 150))
                text_rect = text.get_rect(center=(self.center_x + radius + 20, self.center_y))
                self.screen.blit(text, text_rect)
    
    def draw_stats(self):
        """Draw basic statistics on screen"""
        elapsed_time = time.time() - self.start_time
        
        stats = [
            f"Time: {elapsed_time:.1f}s",
            f"Zoom: {self.scale:.2f}",
            f"Measurements: {'ON' if self.show_measurements else 'OFF'}",
            "ESC: Exit  M: Measurements",
            "+/-: Zoom  R: Reset"
        ]
        
        for i, text in enumerate(stats):
            text_surface = self.font.render(text, True, self.text_color)
            self.screen.blit(text_surface, (10, 10 + i * 22))
    
    def handle_events(self):
        """Handle Pygame events"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return True
                elif event.key == pygame.K_PLUS or event.key == pygame.K_EQUALS:
                    self.scale = min(self.scale * 1.2, 2.0)
                    print(f"🔍 Zoom: {self.scale:.2f}")
                elif event.key == pygame.K_MINUS:
                    self.scale = max(self.scale * 0.8, 0.1)
                    print(f"🔍 Zoom: {self.scale:.2f}")
                elif event.key == pygame.K_r:
                    self.all_points = []
                    self.scan_count = 0
                    self.start_time = time.time()
                    self.room_data = None
                    print("🔄 View reset")
                elif event.key == pygame.K_m:
                    self.show_measurements = not self.show_measurements
                    status = "ON" if self.show_measurements else "OFF"
                    print(f"📐 Measurements: {status}")
                elif event.key == pygame.K_s:
                    self.save_scan_data()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:  # Mouse wheel up
                    self.scale = min(self.scale * 1.1, 2.0)
                elif event.button == 5:  # Mouse wheel down
                    self.scale = max(self.scale * 0.9, 0.1)
        
        return False
    
    def save_scan_data(self):
        """Save current scan data to file"""
        try:
            filename = f"room_scan_{time.strftime('%Y%m%d_%H%M%S')}.txt"
            with open(filename, 'w') as f:
                f.write(f"Room Scan Data - {time.ctime()}\n")
                f.write("=" * 50 + "\n")
                
                if self.room_data:
                    f.write(f"Room Width: {self.room_data['width']/1000:.3f} m\n")
                    f.write(f"Room Height: {self.room_data['height']/1000:.3f} m\n")
                    f.write(f"Room Area: {self.room_data['area']:.3f} m²\n")
                    f.write(f"Room Perimeter: {self.room_data['perimeter']/1000:.3f} m\n")
                    f.write(f"Total Points: {self.room_data['point_count']}\n")
                    f.write(f"Total Scans: {self.scan_count}\n\n")
                
                f.write("Point Data (x, y in mm):\n")
                for point in self.all_points:
                    f.write(f"{point[0]:.1f}, {point[1]:.1f}\n")
            
            print(f"💾 Scan data saved to: {filename}")
            return True
            
        except Exception as e:
            print(f"❌ Error saving data: {e}")
            return False
    
    def stop_scan(self):
        """Stop scanning"""
        self.is_scanning = False
        if self.lidar:
            try:
                self.lidar.stop()
                time.sleep(0.5)
                self.lidar.disconnect()
                print("⏹️ Scan stopped")
            except Exception as e:
                print(f"⚠️ Stop warning: {e}")
    
    def disconnect(self):
        """Disconnect everything"""
        self.stop_scan()
        pygame.quit()
        print("🔌 Disconnected successfully")

def test_com_ports():
    """Test different COM ports to find RPLidar"""
    ports = ['COM3', 'COM4', 'COM5', 'COM6', 'COM7']
    
    for port in ports:
        try:
            print(f"🔍 Testing {port}...")
            lidar = RPLidar(port, timeout=2)
            info = lidar.get_info()
            print(f"✅ Found RPLidar on {port}: {info['model']}")
            lidar.disconnect()
            return port
        except:
            print(f"❌ No device on {port}")
            continue
    
    print("❌ No RPLidar found on any COM port!")
    return None

def main():
    print("🎯 RPLidar Room Mapper & Measurement Tool")
    print("=" * 50)
    
    # Test COM ports first
    port = test_com_ports()
    if not port:
        print("💡 Make sure RPLidar is connected and drivers are installed")
        return
    
    # Create viewer
    mapper = RoomMapperLidar(port)
    
    try:
        # Initialize Pygame
        if not mapper.init_pygame():
            return
        
        # Connect to RPLidar
        if mapper.connect():
            print("\n🎮 Controls:")
            print("  + / - : Zoom in/out")
            print("  Mouse wheel : Zoom")
            print("  M : Toggle measurements")
            print("  R : Reset view")
            print("  S : Save scan data")
            print("  ESC : Exit")
            print("\n🚀 Starting room mapping...")
            print("   Move the lidar around the room for best results!")
            
            # Start scanning
            mapper.start_scan()
        else:
            print("❌ Failed to connect to RPLidar")
            
    except KeyboardInterrupt:
        print("\n⏹️ Program interrupted by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        mapper.disconnect()

if __name__ == "__main__":
    main()
