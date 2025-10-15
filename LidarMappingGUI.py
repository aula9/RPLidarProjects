import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from rplidar import RPLidar
import time
import threading
from collections import deque
import queue
import json
import os
import serial.tools.list_ports

class LidarMappingGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RPLidar Mapping System - Real Time Environment Mapping")
        self.root.geometry("1400x900")
        self.root.configure(bg='#f0f2f5')
        
        # System variables
        self.lidar = None
        self.is_scanning = False
        self.all_points = deque(maxlen=100000)  # Use deque to limit memory usage
        self.scan_count = 0
        self.start_time = None
        self.connection_status = False
        self.scan_thread = None
        
        # Data queue for thread-safe communication
        self.data_queue = queue.Queue()
        
        # Create GUI
        self.setup_gui()
        
        # Auto-detect ports
        self.auto_detect_ports()
        
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Title
        title_label = tk.Label(main_frame, text="RPLidar Real-Time Mapping System", 
                              font=('Arial', 16, 'bold'), bg='#f0f2f5', fg='#2c3e50')
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # Control panel
        control_frame = ttk.LabelFrame(main_frame, text="Control Panel", padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.N, tk.S, tk.W), padx=(0, 10))
        
        # Port selection
        ttk.Label(control_frame, text="COM Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, sticky=(tk.W, tk.E), pady=2)
        
        # Refresh ports button
        self.refresh_btn = ttk.Button(control_frame, text="Refresh Ports", 
                                     command=self.auto_detect_ports, width=12)
        self.refresh_btn.grid(row=0, column=2, padx=(5, 0))
        
        # Connection button
        self.connect_btn = ttk.Button(control_frame, text="Connect to RPLidar", 
                                     command=self.toggle_connection, width=20)
        self.connect_btn.grid(row=1, column=0, columnspan=2, pady=10, sticky=(tk.W, tk.E))
        
        # Status indicator
        self.status_label = tk.Label(control_frame, text="● Disconnected", 
                                   fg='#e74c3c', font=('Arial', 10, 'bold'))
        self.status_label.grid(row=1, column=2, padx=(5, 0))
        
        # Scan controls
        self.scan_btn = ttk.Button(control_frame, text="Start Scanning", 
                                  command=self.start_scan, state='disabled', width=20)
        self.scan_btn.grid(row=2, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        self.stop_btn = ttk.Button(control_frame, text="Stop Scanning", 
                                  command=self.stop_scan, state='disabled', width=20)
        self.stop_btn.grid(row=3, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        # Data management
        self.clear_btn = ttk.Button(control_frame, text="Clear Data", 
                                   command=self.clear_data, state='disabled', width=20)
        self.clear_btn.grid(row=4, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        self.save_btn = ttk.Button(control_frame, text="Save Scan Data", 
                                  command=self.save_data, state='disabled', width=20)
        self.save_btn.grid(row=5, column=0, columnspan=2, pady=5, sticky=(tk.W, tk.E))
        
        # Statistics frame
        stats_frame = ttk.LabelFrame(control_frame, text="Statistics", padding="10")
        stats_frame.grid(row=6, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(20, 0))
        
        self.stats_text = tk.Text(stats_frame, height=12, width=30, font=('Arial', 9),
                                 bg='#f8f9fa', relief='flat')
        self.stats_text.grid(row=0, column=0, sticky=(tk.W, tk.E))
        self.stats_text.insert('1.0', "Disconnected\n\nConnect to RPLidar to start mapping...")
        self.stats_text.config(state='disabled')
        
        # Visualization area
        viz_frame = ttk.LabelFrame(main_frame, text="Real-time Map", padding="10")
        viz_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        viz_frame.columnconfigure(0, weight=1)
        viz_frame.rowconfigure(0, weight=1)
        
        # Create matplotlib figure
        self.fig = Figure(figsize=(8, 6), dpi=100, facecolor='#f8f9fa')
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#f8f9fa')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_title('Lidar Scan Data', pad=20)
        
        # Initialize scatter plot
        self.scatter = self.ax.scatter([], [], s=1, alpha=0.6, color='blue')
        self.ax.set_xlim(-5000, 5000)
        self.ax.set_ylim(-5000, 5000)
        self.ax.set_aspect('equal')
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure control frame stretching
        control_frame.columnconfigure(1, weight=1)
        stats_frame.columnconfigure(0, weight=1)
        
    def auto_detect_ports(self):
        """Auto-detect available COM ports"""
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_var.set(ports[0])
        
    def toggle_connection(self):
        """Toggle lidar connection"""
        if self.connection_status:
            self.disconnect_lidar()
        else:
            self.connect_to_lidar()
            
    def connect_to_lidar(self):
        try:
            port = self.port_var.get()
            if not port:
                messagebox.showerror("Error", "Please select a COM port")
                return
                
            self.lidar = RPLidar(port)
            
            # Test connection
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            
            self.connection_status = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text="● Connected", fg='#2ecc71')
            self.scan_btn.config(state='normal')
            self.clear_btn.config(state='normal')
            self.save_btn.config(state='normal')
            self.port_combo.config(state='disabled')
            self.refresh_btn.config(state='disabled')
            
            # Update stats
            stats = (f"Device Connected\n\n"
                    f"Model: {info['model']}\n"
                    f"Firmware: {info['firmware']}\n"
                    f"Hardware: {info['hardware']}\n"
                    f"Health: {health[0]}\n"
                    f"Status: Ready to scan")
            self.update_stats(stats)
            
            messagebox.showinfo("Success", f"Connected to RPLidar on {port}")
            
        except Exception as e:
            messagebox.showerror("Connection Error", f"Failed to connect: {str(e)}")
            
    def disconnect_lidar(self):
        self.stop_scan()
        if self.lidar:
            try:
                self.lidar.disconnect()
                self.lidar.stop_motor()
            except:
                pass
            self.lidar = None
            
        self.connection_status = False
        self.connect_btn.config(text="Connect to RPLidar")
        self.status_label.config(text="● Disconnected", fg='#e74c3c')
        self.scan_btn.config(state='disabled')
        self.stop_btn.config(state='disabled')
        self.port_combo.config(state='readonly')
        self.refresh_btn.config(state='normal')
        
        self.update_stats("Disconnected\n\nConnect to RPLidar to start mapping...")
        
    def start_scan(self):
        if not self.connection_status:
            messagebox.showwarning("Warning", "Not connected to RPLidar")
            return
            
        self.is_scanning = True
        self.scan_btn.config(state='disabled')
        self.stop_btn.config(state='normal')
        self.start_time = time.time()
        self.scan_count = 0
        
        # Clear previous data if any
        self.all_points.clear()
        
        # Start scanning thread
        self.scan_thread = threading.Thread(target=self.scan_worker, daemon=True)
        self.scan_thread.start()
        
        # Start processing the queue in the main thread
        self.process_queue()
        
    def stop_scan(self):
        self.is_scanning = False
        self.scan_btn.config(state='normal')
        self.stop_btn.config(state='disabled')
        
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
            except:
                pass
                
        # Wait for thread to finish
        if self.scan_thread and self.scan_thread.is_alive():
            self.scan_thread.join(timeout=2.0)
            
        # Update final statistics
        self.update_visualization()
        
    def scan_worker(self):
        try:
            for scan in self.lidar.iter_scans(scan_type='normal', max_buf_meas=500):
                if not self.is_scanning:
                    break
                    
                points = []
                for quality, angle, distance in scan:
                    if distance > 0 and quality > 0:
                        # Convert to Cartesian coordinates
                        angle_rad = np.radians(angle)
                        x = distance * np.cos(angle_rad)
                        y = distance * np.sin(angle_rad)
                        points.append([x, y, quality])
                
                # Put the points in the queue
                if points:
                    self.data_queue.put(points)
                
        except Exception as e:
            if self.is_scanning:  # Only show error if we didn't stop intentionally
                self.data_queue.put(("error", str(e)))
                
    def process_queue(self):
        try:
            while True:
                # Get data from the queue (non-blocking)
                data = self.data_queue.get_nowait()
                
                if isinstance(data, tuple) and data[0] == "error":
                    messagebox.showerror("Scan Error", f"Scanning error: {data[1]}")
                    self.stop_scan()
                else:
                    # Add new points
                    self.all_points.extend(data)
                    self.scan_count += 1
                    
                    # Update visualization periodically
                    if self.scan_count % 2 == 0:  # Update every 2 scans for performance
                        self.update_visualization()
                    
        except queue.Empty:
            pass
        
        # Schedule the next update if still scanning
        if self.is_scanning:
            self.root.after(50, self.process_queue)  # Process every 50ms
            
    def update_visualization(self):
        if not self.all_points:
            return
            
        # Convert to numpy array for efficiency
        points_array = np.array(self.all_points)
        
        # Update scatter plot data
        self.scatter.set_offsets(points_array[:, :2])
        
        # Adjust limits dynamically
        if len(points_array) > 10:
            x_data = points_array[:, 0]
            y_data = points_array[:, 1]
            
            x_range = x_data.max() - x_data.min()
            y_range = y_data.max() - y_data.min()
            max_range = max(x_range, y_range, 1000)  # Minimum range of 1000mm
            
            margin = max_range * 0.1
            self.ax.set_xlim(x_data.min() - margin, x_data.max() + margin)
            self.ax.set_ylim(y_data.min() - margin, y_data.max() + margin)
        
        # Update statistics
        if self.start_time:
            duration = time.time() - self.start_time
            stats = (f"Scanning...\n\n"
                    f"Total Points: {len(self.all_points)}\n"
                    f"Scan Duration: {duration:.1f}s\n"
                    f"Scans Processed: {self.scan_count}\n"
                    f"Points/Sec: {len(self.all_points)/duration:.0f}\n"
                    f"Data Rate: {len(self.all_points)/duration/1000:.1f}K pts/s")
            self.update_stats(stats)
        
        # Refresh canvas
        self.canvas.draw_idle()
        
    def update_stats(self, text):
        self.stats_text.config(state='normal')
        self.stats_text.delete('1.0', tk.END)
        self.stats_text.insert('1.0', text)
        self.stats_text.config(state='disabled')
        
    def clear_data(self):
        self.all_points.clear()
        self.scan_count = 0
        self.update_visualization()
        self.update_stats("Data Cleared\n\nReady for new scan...")
        
    def save_data(self):
        if not self.all_points:
            messagebox.showwarning("Warning", "No data to save")
            return
            
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Save Scan Data"
        )
        
        if filename:
            try:
                # Convert deque to list for JSON serialization
                data_to_save = {
                    'points': list(self.all_points),
                    'scan_count': self.scan_count,
                    'timestamp': time.time(),
                    'total_points': len(self.all_points)
                }
                
                with open(filename, 'w') as f:
                    json.dump(data_to_save, f, indent=2)
                    
                messagebox.showinfo("Success", f"Data saved to {filename}")
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save data: {str(e)}")

def main():
    root = tk.Tk()
    app = LidarMappingGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
