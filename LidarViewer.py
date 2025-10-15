import pygame
import sys
import math
import time
from rplidar import RPLidar
import numpy as np

class FixedLidarViewer:
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
        self.scale = 0.3  # Fixed scale for proper visualization
        self.point_size = 4
        
        # Colors - brighter for better visibility
        self.bg_color = (10, 10, 40)  # Dark blue background
        self.point_color = (0, 255, 255)  # Cyan points
        self.robot_color = (255, 50, 50)  # Bright red robot
        self.axis_color = (50, 255, 50)  # Green axes
        self.text_color = (255, 255, 100)  #Yellow text
        self.ray_color = (80, 80, 120)  # Subtle rays
        
        # Data storage
        self.all_points = []  # Store all points
        self.scan_count = 0
        self.start_time = time.time()
        self.font = None
        
    def connect(self):
        """Connect to RPLidar"""
        try:
            print(f"🔄 Connecting to RPLidar on {self.port}...")
            self.lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=3)
            time.sleep(1)
            
            # Get device info
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            print("✅ Connected to RPLidar successfully!")
            print(f"   Model: {info['model']}")
            print(f"   Firmware: {info['firmware']}")
            print(f"   Hardware: {info['hardware']}")
            print(f"   Health: {health}")
            
            return True
            
        except Exception as e:
            print(f"❌ Connection error: {e}")
            print("💡 Try different COM port: COM4, COM5, etc.")
            return False
    
    def init_pygame(self):
        """Initialize Pygame"""
        try:
            pygame.init()
            self.screen = pygame.display.set_mode((self.screen_size, self.screen_size))
            pygame.display.set_caption("RPLidar Real-Time Viewer - ESC to exit | +/- to zoom")
            self.font = pygame.font.SysFont('Arial', 18, bold=True)
            print("✅ Pygame initialized successfully!")
            return True
        except Exception as e:
            print(f"❌ Pygame init error: {e}")
            return False
    
    def start_scan(self):
        """Start scanning with proper error handling"""
        try:
            print("🚀 Starting scan...")
            self.is_scanning = True
            
            # Start scanning in a separate thread
            import threading
            scan_thread = threading.Thread(target=self.scan_worker, daemon=True)
            scan_thread.start()
            
            # Main display loop
            clock = pygame.time.Clock()
            while self.is_scanning:
                self.draw_frame()
                if self.handle_events():
                    break
                clock.tick(30)  # 30 FPS
                
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
                    if distance > 0:  # Basic filter
                        # Convert to cartesian coordinates
                        angle_rad = np.radians(angle)
                        x = distance * np.cos(angle_rad)
                        y = distance * np.sin(angle_rad)
                        new_points.append((x, y))
                        valid_points += 1
                
                if new_points:
                    self.all_points.extend(new_points)
                    # Keep only last 2000 points for performance
                    if len(self.all_points) > 2000:
                        self.all_points = self.all_points[-2000:]
                    
                    self.scan_count += 1
                    scan_num += 1
                    
                    if scan_num % 10 == 0:
                        print(f"📊 Scan {scan_num}: {valid_points} points")
                        
        except Exception as e:
            print(f"❌ Scan worker error: {e}")
            self.is_scanning = False
    
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
                
                # Check if point is within screen bounds
                if 0 <= screen_x < self.screen_size and 0 <= screen_y < self.screen_size:
                    # Draw point
                    pygame.draw.circle(self.screen, self.point_color, 
                                     (screen_x, screen_y), self.point_size)
        
        # Draw robot (center)
        pygame.draw.circle(self.screen, self.robot_color, 
                         (self.center_x, self.center_y), 12)
        pygame.draw.circle(self.screen, (255, 255, 255), 
                         (self.center_x, self.center_y), 12, 2)
        
        # Draw statistics
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
    
    def draw_stats(self):
        """Draw statistics on screen"""
        elapsed_time = time.time() - self.start_time
        stats = [
            f"Points: {len(self.all_points)}",
            f"Scans: {self.scan_count}",
            f"Time: {elapsed_time:.1f}s",
            f"Scale: {self.scale:.2f}",
            "ESC: Exit",
            "+/-: Zoom",
            "R: Reset view"
        ]
        
        for i, text in enumerate(stats):
            text_surface = self.font.render(text, True, self.text_color)
            self.screen.blit(text_surface, (10, 10 + i * 25))
    
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
                    print("🔄 View reset")
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:  # Mouse wheel up
                    self.scale = min(self.scale * 1.1, 2.0)
                elif event.button == 5:  # Mouse wheel down
                    self.scale = max(self.scale * 0.9, 0.1)
        
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
    print("🎯 RPLidar Real-Time Viewer - FIXED VERSION")
    print("=" * 50)
    
    # Test COM ports first
    port = test_com_ports()
    if not port:
        print("💡 Make sure RPLidar is connected and drivers are installed")
        return
    
    # Create viewer
    viewer = FixedLidarViewer(port)
    
    try:
        # Initialize Pygame
        if not viewer.init_pygame():
            return
        
        # Connect to RPLidar
        if viewer.connect():
            print("\n🎮 Controls:")
            print("  + / - : Zoom in/out")
            print("  Mouse wheel : Zoom")
            print("  R : Reset view")
            print("  ESC : Exit")
            print("\n🚀 Starting real-time scanning...")
            print("   Points should appear immediately!")
            
            # Start scanning
            viewer.start_scan()
        else:
            print("❌ Failed to connect to RPLidar")
            
    except KeyboardInterrupt:
        print("\n⏹️ Program interrupted by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        viewer.disconnect()

if __name__ == "__main__":
    main()
