import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.colors as mcolors
from rplidar import RPLidar
import time
import threading
from collections import deque
import queue
import json
import os
import serial.tools.list_ports
from datetime import datetime
import csv

class ModernLidarMappingGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RPLidar Mapping System - Real Time Environment Mapping")
        
        
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        
        
        window_width = int(screen_width * 0.85)
        window_height = int(screen_height * 0.85)
        
        
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        
        self.root.geometry(f"{window_width}x{window_height}+{x}+{y}")
        self.root.configure(bg='#1e1e2e')
        
        # Set theme colors
        self.colors = {
            'bg': '#1e1e2e',
            'card_bg': '#2a2a3c',
            'accent': '#7e57c2',
            'accent_hover': '#9575cd',
            'text': '#e2e2e2',
            'text_secondary': '#a0a0a0',
            'success': '#66bb6a',
            'warning': '#ffb74d',
            'error': '#ef5350'
        }
        
        # System variables
        self.lidar = None
        self.is_scanning = False
        self.all_points = deque(maxlen=150000)
        self.scan_count = 0
        self.start_time = None
        self.connection_status = False
        self.scan_thread = None
        self.visualization_mode = "scatter"  # "scatter", "heatmap", "lines"
        self.point_color = "#4fc3f7"
        self.point_size = 2
        self.filter_distance = 8000  # Maximum distance in mm
        
        # Data queue for thread-safe communication
        self.data_queue = queue.Queue()
        
        # Create GUI
        self.setup_gui()
        
        # Auto-detect ports
        self.auto_detect_ports()
        
        # Start background tasks
        self.update_clock()
        
    def setup_gui(self):
        # Configure style
        self.setup_styles()
        
        # Main container
        main_container = tk.Frame(self.root, bg=self.colors['bg'])
        main_container.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)  
        
        # Header
        self.setup_header(main_container)
        
        # Content area
        content_frame = tk.Frame(main_container, bg=self.colors['bg'])
        content_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        
        # Left panel (controls) 
        self.setup_control_panel(content_frame)
        
        # Right panel (visualization)
        self.setup_visualization_panel(content_frame)
        
    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        # Configure styles
        style.configure('Custom.TFrame', background=self.colors['card_bg'])
        style.configure('Title.TLabel', 
                       background=self.colors['bg'],
                       foreground=self.colors['text'],
                       font=('Arial', 16, 'bold'))  # خط أصغر
        
        style.configure('Card.TLabelframe', 
                       background=self.colors['card_bg'],
                       foreground=self.colors['text'],
                       relief='flat',
                       borderwidth=1)
        
        style.configure('Card.TLabelframe.Label', 
                       background=self.colors['card_bg'],
                       foreground=self.colors['accent'],
                       font=('Arial', 10, 'bold'))  
        
        style.configure('Accent.TButton',
                       background=self.colors['accent'],
                       foreground=self.colors['text'],
                       focuscolor='none',
                       borderwidth=0,
                       font=('Arial', 9, 'bold'))  
        
        style.map('Accent.TButton',
                 background=[('active', self.colors['accent_hover']),
                           ('pressed', self.colors['accent_hover'])])
        
        style.configure('Secondary.TButton',
                       background=self.colors['card_bg'],
                       foreground=self.colors['text_secondary'],
                       focuscolor='none',
                       borderwidth=1,
                       font=('Arial', 8))  
        
    def setup_header(self, parent):
        header_frame = tk.Frame(parent, bg=self.colors['bg'])
        header_frame.pack(fill=tk.X, pady=(0, 8))  
        
        # Title and subtitle
        title_label = tk.Label(header_frame, 
                              text="RPLidar Mapping System", 
                              font=('Arial', 20, 'bold'),  
                              bg=self.colors['bg'],
                              fg=self.colors['text'])
        title_label.pack(side=tk.LEFT)
        
        subtitle_label = tk.Label(header_frame,
                                text="Real-time Environment Mapping",
                                font=('Arial', 10), 
                                bg=self.colors['bg'],
                                fg=self.colors['text_secondary'])
        subtitle_label.pack(side=tk.LEFT, padx=(8, 0), pady=(3, 0))  
        
        # Clock and status
        status_frame = tk.Frame(header_frame, bg=self.colors['bg'])
        status_frame.pack(side=tk.RIGHT)
        
        self.clock_label = tk.Label(status_frame,
                                   text="00:00:00",
                                   font=('Arial', 10, 'bold'),  
                                   bg=self.colors['bg'],
                                   fg=self.colors['text_secondary'])
        self.clock_label.pack(anchor=tk.E)
        
        self.status_label = tk.Label(status_frame,
                                    text="● Disconnected",
                                    font=('Arial', 9, 'bold'),  
                                    bg=self.colors['bg'],
                                    fg=self.colors['error'])
        self.status_label.pack(anchor=tk.E)
        
    def setup_control_panel(self, parent):
        control_frame = ttk.LabelFrame(parent, text="Control Panel", style='Card.TLabelframe', width=280)  
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 8))  
        control_frame.pack_propagate(False)
        
        # Connection section
        conn_frame = ttk.Frame(control_frame, style='Custom.TFrame')
        conn_frame.pack(fill=tk.X, padx=12, pady=12)  
        
        ttk.Label(conn_frame, text="Device Connection", 
                 font=('Arial', 10, 'bold'),  
                 background=self.colors['card_bg'],
                 foreground=self.colors['text']).pack(anchor=tk.W)
        
        # Port selection
        port_frame = ttk.Frame(conn_frame, style='Custom.TFrame')
        port_frame.pack(fill=tk.X, pady=(8, 3))  
        
        ttk.Label(port_frame, text="COM Port:", 
                 background=self.colors['card_bg'],
                 foreground=self.colors['text_secondary']).pack(side=tk.LEFT)
        
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(port_frame, textvariable=self.port_var, width=10)  
        self.port_combo.pack(side=tk.LEFT, padx=(3, 0))  
        
        self.refresh_btn = ttk.Button(port_frame, text="Refresh", 
                                     command=self.auto_detect_ports, width=6)  
        self.refresh_btn.pack(side=tk.LEFT, padx=(3, 0))  
        
        # Connection button
        self.connect_btn = ttk.Button(conn_frame, text="Connect to RPLidar", 
                                     command=self.toggle_connection, 
                                     style='Accent.TButton', width=18)  
        self.connect_btn.pack(fill=tk.X, pady=(8, 0))  
        
        # Scan controls
        scan_control_frame = ttk.Frame(control_frame, style='Custom.TFrame')
        scan_control_frame.pack(fill=tk.X, padx=12, pady=12)  
        
        ttk.Label(scan_control_frame, text="Scan Controls", 
                 font=('Arial', 10, 'bold'),  
                 background=self.colors['card_bg'],
                 foreground=self.colors['text']).pack(anchor=tk.W)
        
        self.scan_btn = ttk.Button(scan_control_frame, text="Start Scanning", 
                                  command=self.start_scan, state='disabled', 
                                  style='Accent.TButton', width=18)  
        self.scan_btn.pack(fill=tk.X, pady=(8, 3))  
        
        self.stop_btn = ttk.Button(scan_control_frame, text="Stop Scanning", 
                                  command=self.stop_scan, state='disabled',
                                  style='Secondary.TButton', width=18)  
        self.stop_btn.pack(fill=tk.X, pady=3) 
        
        # Visualization settings
        viz_frame = ttk.Frame(control_frame, style='Custom.TFrame')
        viz_frame.pack(fill=tk.X, padx=12, pady=12)  
        
        ttk.Label(viz_frame, text="Visualization Settings", 
                 font=('Arial', 10, 'bold'),  
                 background=self.colors['card_bg'],
                 foreground=self.colors['text']).pack(anchor=tk.W)
        
        # Visualization mode
        mode_frame = ttk.Frame(viz_frame, style='Custom.TFrame')
        mode_frame.pack(fill=tk.X, pady=(8, 3))  
        
        ttk.Label(mode_frame, text="Display Mode:", 
                 background=self.colors['card_bg'],
                 foreground=self.colors['text_secondary']).pack(anchor=tk.W)
        
        self.viz_mode_var = tk.StringVar(value="scatter")
        modes = [("Scatter Plot", "scatter"), ("Heat Map", "heatmap"), ("Line Plot", "lines")]
        for text, mode in modes:
            rb = ttk.Radiobutton(mode_frame, text=text, value=mode, 
                               variable=self.viz_mode_var,
                               command=self.change_visualization_mode)
            rb.pack(anchor=tk.W, pady=1)  
        
        # Point settings
        point_frame = ttk.Frame(viz_frame, style='Custom.TFrame')
        point_frame.pack(fill=tk.X, pady=3)  
        
        ttk.Label(point_frame, text="Point Color:", 
                 background=self.colors['card_bg'],
                 foreground=self.colors['text_secondary']).pack(anchor=tk.W)
        
        color_frame = ttk.Frame(point_frame, style='Custom.TFrame')
        color_frame.pack(fill=tk.X, pady=(3, 0)) 
        
        colors = [("#4fc3f7", "Blue"), ("#66bb6a", "Green"), ("#ffb74d", "Orange"), 
                 ("#ef5350", "Red"), ("#ba68c8", "Purple")]
        self.color_var = tk.StringVar(value="#4fc3f7")
        
        for color_code, color_name in colors:
            color_btn = tk.Button(color_frame, text="", bg=color_code, width=2, height=1,  
                                relief='solid', bd=1,
                                command=lambda c=color_code: self.set_point_color(c))
            color_btn.pack(side=tk.LEFT, padx=(0, 3))  
        
        # Distance filter
        filter_frame = ttk.Frame(viz_frame, style='Custom.TFrame')
        filter_frame.pack(fill=tk.X, pady=3)  
        
        ttk.Label(filter_frame, text="Max Distance (mm):", 
                 background=self.colors['card_bg'],
                 foreground=self.colors['text_secondary']).pack(anchor=tk.W)
        
        self.distance_var = tk.IntVar(value=8000)
        distance_scale = ttk.Scale(filter_frame, from_=1000, to=16000, 
                                 variable=self.distance_var, orient=tk.HORIZONTAL,
                                 command=self.update_distance_filter)
        distance_scale.pack(fill=tk.X, pady=(3, 0))  
        
        self.distance_label = ttk.Label(filter_frame, text="8000 mm",
                                      background=self.colors['card_bg'],
                                      foreground=self.colors['text_secondary'])
        self.distance_label.pack(anchor=tk.E)
        
        # Data management
        data_frame = ttk.Frame(control_frame, style='Custom.TFrame')
        data_frame.pack(fill=tk.X, padx=12, pady=12)  
        
        ttk.Label(data_frame, text="Data Management", 
                 font=('Arial', 10, 'bold'),  
                 background=self.colors['card_bg'],
                 foreground=self.colors['text']).pack(anchor=tk.W)
        
        self.clear_btn = ttk.Button(data_frame, text="Clear Current Data", 
                                   command=self.clear_data, state='disabled',
                                   style='Secondary.TButton', width=18)  
        self.clear_btn.pack(fill=tk.X, pady=(8, 3))  
        
        self.save_btn = ttk.Button(data_frame, text="Export Scan Data", 
                                  command=self.export_data, state='disabled',
                                  style='Secondary.TButton', width=18)  
        self.save_btn.pack(fill=tk.X, pady=3)  
        
        self.load_btn = ttk.Button(data_frame, text="Load Previous Data", 
                                  command=self.load_data, state='normal',
                                  style='Secondary.TButton', width=18)  
        self.load_btn.pack(fill=tk.X, pady=3)  
        
        # Statistics
        stats_frame = ttk.LabelFrame(control_frame, text="Real-time Statistics", style='Card.TLabelframe')
        stats_frame.pack(fill=tk.BOTH, expand=True, padx=12, pady=12)  
        
        self.stats_text = tk.Text(stats_frame, height=10, width=25, font=('Arial', 9), 
                                 bg=self.colors['card_bg'], fg=self.colors['text'],
                                 relief='flat', padx=8, pady=8, wrap=tk.WORD)  
        self.stats_text.pack(fill=tk.BOTH, expand=True, padx=3, pady=3) 
        self.stats_text.insert('1.0', "System Ready\n\nConnect to RPLidar to start mapping...")
        self.stats_text.config(state='disabled')
        
    def setup_visualization_panel(self, parent):
        viz_frame = ttk.LabelFrame(parent, text="Real-time Environment Map", style='Card.TLabelframe')
        viz_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Toolbar
        toolbar_frame = tk.Frame(viz_frame, bg=self.colors['card_bg'])
        toolbar_frame.pack(fill=tk.X, padx=8, pady=3)  
        
        ttk.Button(toolbar_frame, text="Fit to Data", 
                  command=self.fit_to_data, style='Secondary.TButton').pack(side=tk.LEFT)
        
        ttk.Button(toolbar_frame, text="Reset View", 
                  command=self.reset_view, style='Secondary.TButton').pack(side=tk.LEFT, padx=(3, 0))  
        
        self.auto_fit_var = tk.BooleanVar(value=True)
        auto_fit_cb = ttk.Checkbutton(toolbar_frame, text="Auto Fit", 
                                     variable=self.auto_fit_var,
                                     style='TCheckbutton')
        auto_fit_cb.pack(side=tk.LEFT, padx=(15, 0)) 
        

        zoom_frame = tk.Frame(toolbar_frame, bg=self.colors['card_bg'])
        zoom_frame.pack(side=tk.RIGHT)
        
        ttk.Button(zoom_frame, text="−", 
                  command=self.zoom_out, width=2,
                  style='Secondary.TButton').pack(side=tk.LEFT, padx=(2, 0))
        
        ttk.Button(zoom_frame, text="＋", 
                  command=self.zoom_in, width=2,
                  style='Secondary.TButton').pack(side=tk.LEFT, padx=(2, 0))
        
        ttk.Button(zoom_frame, text="🗖", 
                  command=self.toggle_fullscreen, width=2,
                  style='Secondary.TButton').pack(side=tk.LEFT, padx=(2, 0))
        
        # Matplotlib figure
        self.fig = Figure(figsize=(8, 6), dpi=100, facecolor=self.colors['card_bg'])  
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor('#1a1a2e')
        self.ax.grid(True, alpha=0.2, color='#444466')
        self.ax.set_xlabel('X (mm)', color=self.colors['text'], fontsize=10)  
        self.ax.set_ylabel('Y (mm)', color=self.colors['text'], fontsize=10)  
        self.ax.set_title('LIDAR Environment Scan', color=self.colors['text'], fontsize=12, pad=15)  
        
        # Set initial axis colors
        self.ax.tick_params(colors=self.colors['text_secondary'])
        for spine in self.ax.spines.values():
            spine.set_color(self.colors['text_secondary'])
        
        # Initialize scatter plot
        self.scatter = self.ax.scatter([], [], s=self.point_size, alpha=0.7, color=self.point_color)
        self.ax.set_xlim(-5000, 5000)
        self.ax.set_ylim(-5000, 5000)
        self.ax.set_aspect('equal')
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=viz_frame)
        self.canvas.draw()
        
        # Create navigation toolbar
        self.toolbar = NavigationToolbar2Tk(self.canvas, viz_frame, pack_toolbar=False)
        self.toolbar.update()
        
        # Pack canvas and toolbar
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))  
        self.toolbar.pack(side=tk.BOTTOM, fill=tk.X, padx=8, pady=(0, 3))  
        
        # Style the toolbar
        self.style_toolbar()

    def zoom_in(self):
        current_geometry = self.root.geometry()
        parts = current_geometry.split('+')
        size_part = parts[0]
        w, h = map(int, size_part.split('x'))
        self.root.geometry(f"{int(w*1.1)}x{int(h*1.1)}")

    def zoom_out(self):
        current_geometry = self.root.geometry()
        parts = current_geometry.split('+')
        size_part = parts[0]
        w, h = map(int, size_part.split('x'))
        self.root.geometry(f"{int(w*0.9)}x{int(h*0.9)}")

    def toggle_fullscreen(self):
        self.root.attributes('-fullscreen', not self.root.attributes('-fullscreen'))
        
    
    def style_toolbar(self):
        """Apply custom styling to the matplotlib toolbar"""
        try:
            # Configure toolbar background
            self.toolbar.config(background=self.colors['card_bg'])
            
            # Style all buttons in the toolbar
            for toolitem in self.toolbar.winfo_children():
                if isinstance(toolitem, tk.Button):
                    toolitem.config(
                        background=self.colors['accent'],
                        foreground=self.colors['text'],
                        relief='flat',
                        borderwidth=0,
                        width=6  
                    )
        except Exception as e:
            print(f"Toolbar styling error: {e}")
        
    def auto_detect_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_var.set(ports[0])
        
    def toggle_connection(self):
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
            self.status_label.config(text="● Connected", fg=self.colors['success'])
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
        self.status_label.config(text="● Disconnected", fg=self.colors['error'])
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
                    if 0 < distance <= self.filter_distance and quality > 0:
                        # Convert to Cartesian coordinates
                        angle_rad = np.radians(angle)
                        x = distance * np.cos(angle_rad)
                        y = distance * np.sin(angle_rad)
                        points.append([x, y, quality, distance])
                
                # Put the points in the queue
                if points:
                    self.data_queue.put(points)
                
        except Exception as e:
            if self.is_scanning:
                self.data_queue.put(("error", str(e)))
                
    def process_queue(self):
        try:
            while True:
                data = self.data_queue.get_nowait()
                
                if isinstance(data, tuple) and data[0] == "error":
                    messagebox.showerror("Scan Error", f"Scanning error: {data[1]}")
                    self.stop_scan()
                else:
                    self.all_points.extend(data)
                    self.scan_count += 1
                    
                    # Update visualization periodically for performance
                    if self.scan_count % 3 == 0:
                        self.update_visualization()
                    
        except queue.Empty:
            pass
        
        if self.is_scanning:
            self.root.after(50, self.process_queue)
            
    def update_visualization(self):
        if not self.all_points:
            return
            
        # Convert to numpy array for efficiency
        points_array = np.array(self.all_points)
        
        # Filter by distance
        points_array = points_array[points_array[:, 3] <= self.filter_distance]
        
        if len(points_array) == 0:
            return
            
        # Clear the axis for different visualization modes
        if self.visualization_mode != "scatter":
            self.ax.clear()
            self.ax.set_facecolor('#1a1a2e')
            self.ax.grid(True, alpha=0.2, color='#444466')
            self.ax.set_xlabel('X (mm)', color=self.colors['text'])
            self.ax.set_ylabel('Y (mm)', color=self.colors['text'])
            self.ax.set_title('LIDAR Environment Scan', color=self.colors['text'])
            self.ax.tick_params(colors=self.colors['text_secondary'])
            for spine in self.ax.spines.values():
                spine.set_color(self.colors['text_secondary'])
        
        # Update visualization based on mode
        if self.visualization_mode == "scatter":
            self.scatter.set_offsets(points_array[:, :2])
            self.scatter.set_color(self.point_color)
            self.scatter.set_sizes([self.point_size] * len(points_array))
        elif self.visualization_mode == "heatmap":
            if len(points_array) > 100:  # Only create heatmap if we have enough points
                self.ax.hist2d(points_array[:, 0], points_array[:, 1], bins=50, cmap='hot')
                self.ax.set_title('LIDAR Heat Map', color=self.colors['text'])
        elif self.visualization_mode == "lines":
            # Simple line plot from center to points (sample for performance)
            sample_points = points_array[::20]  # Sample every 20th point
            for point in sample_points:
                self.ax.plot([0, point[0]], [0, point[1]], color=self.point_color, 
                           alpha=0.1, linewidth=0.5)
            self.ax.scatter(points_array[:, 0], points_array[:, 1], 
                          s=1, alpha=0.5, color=self.point_color)
            self.ax.set_title('LIDAR Line Plot', color=self.colors['text'])
        
        # Adjust limits dynamically if auto-fit is enabled
        if self.auto_fit_var.get() and len(points_array) > 10:
            x_data = points_array[:, 0]
            y_data = points_array[:, 1]
            
            x_range = x_data.max() - x_data.min()
            y_range = y_data.max() - y_data.min()
            max_range = max(x_range, y_range, 1000)
            
            margin = max_range * 0.1
            self.ax.set_xlim(x_data.min() - margin, x_data.max() + margin)
            self.ax.set_ylim(y_data.min() - margin, y_data.max() + margin)
        
        # Update statistics
        if self.start_time:
            duration = time.time() - self.start_time
            stats = (f"Scanning in Progress...\n\n"
                    f"Total Points: {len(self.all_points):,}\n"
                    f"Scan Duration: {duration:.1f}s\n"
                    f"Scans Processed: {self.scan_count}\n"
                    f"Points/Sec: {len(self.all_points)/duration:.0f}\n"
                    f"Data Rate: {len(self.all_points)/duration/1000:.1f}K pts/s\n"
                    f"Memory Usage: {len(self.all_points)*4/1024:.1f} KB")
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
        self.ax.clear()
        self.ax.set_facecolor('#1a1a2e')
        self.ax.grid(True, alpha=0.2, color='#444466')
        self.ax.set_xlabel('X (mm)', color=self.colors['text'])
        self.ax.set_ylabel('Y (mm)', color=self.colors['text'])
        self.ax.set_title('LIDAR Environment Scan', color=self.colors['text'])
        self.ax.tick_params(colors=self.colors['text_secondary'])
        for spine in self.ax.spines.values():
            spine.set_color(self.colors['text_secondary'])
        self.scatter = self.ax.scatter([], [], s=self.point_size, alpha=0.7, color=self.point_color)
        self.ax.set_xlim(-5000, 5000)
        self.ax.set_ylim(-5000, 5000)
        self.canvas.draw_idle()
        self.update_stats("Data Cleared\n\nReady for new scan...")
        
    def export_data(self):
        if not self.all_points:
            messagebox.showwarning("Warning", "No data to export")
            return
            
        file_types = [
            ("JSON files", "*.json"),
            ("CSV files", "*.csv"),
            ("All files", "*.*")
        ]
        
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=file_types,
            title="Export Scan Data"
        )
        
        if filename:
            try:
                # Convert deque to list for serialization
                points_list = list(self.all_points)
                
                if filename.endswith('.json'):
                    data_to_save = {
                        'points': points_list,
                        'scan_count': self.scan_count,
                        'timestamp': datetime.now().isoformat(),
                        'total_points': len(self.all_points),
                        'filter_distance': self.filter_distance
                    }
                    
                    with open(filename, 'w') as f:
                        json.dump(data_to_save, f, indent=2)
                        
                elif filename.endswith('.csv'):
                    with open(filename, 'w', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(['X', 'Y', 'Quality', 'Distance'])
                        writer.writerows(points_list)
                    
                messagebox.showinfo("Success", f"Data exported to {filename}")
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to export data: {str(e)}")
    
    def load_data(self):
        file_types = [
            ("JSON files", "*.json"),
            ("CSV files", "*.csv"),
            ("All files", "*.*")
        ]
        
        filename = filedialog.askopenfilename(
            filetypes=file_types,
            title="Load Scan Data"
        )
        
        if filename:
            try:
                if filename.endswith('.json'):
                    with open(filename, 'r') as f:
                        data = json.load(f)
                    points = data['points']
                    
                elif filename.endswith('.csv'):
                    points = []
                    with open(filename, 'r') as f:
                        reader = csv.reader(f)
                        next(reader)  # Skip header
                        for row in reader:
                            points.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
                
                # Load points
                self.all_points.clear()
                self.all_points.extend(points)
                self.scan_count = len(points) // 100  # Estimate
                
                # Update visualization
                self.update_visualization()
                
                # Update stats
                stats = (f"Data Loaded\n\n"
                        f"Total Points: {len(self.all_points):,}\n"
                        f"File: {os.path.basename(filename)}\n"
                        f"Loaded successfully")
                self.update_stats(stats)
                
                messagebox.showinfo("Success", f"Data loaded from {filename}")
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load data: {str(e)}")
    
    def change_visualization_mode(self):
        self.visualization_mode = self.viz_mode_var.get()
        self.update_visualization()
    
    def set_point_color(self, color):
        self.point_color = color
        self.update_visualization()
    
    def update_distance_filter(self, value):
        self.filter_distance = int(float(value))
        self.distance_label.config(text=f"{self.filter_distance} mm")
        self.update_visualization()
    
    def fit_to_data(self):
        if self.all_points:
            points_array = np.array(self.all_points)
            if len(points_array) > 10:
                x_data = points_array[:, 0]
                y_data = points_array[:, 1]
                
                x_range = x_data.max() - x_data.min()
                y_range = y_data.max() - y_data.min()
                max_range = max(x_range, y_range, 1000)
                
                margin = max_range * 0.1
                self.ax.set_xlim(x_data.min() - margin, x_data.max() + margin)
                self.ax.set_ylim(y_data.min() - margin, y_data.max() + margin)
                self.canvas.draw_idle()
    
    def reset_view(self):
        self.ax.set_xlim(-5000, 5000)
        self.ax.set_ylim(-5000, 5000)
        self.canvas.draw_idle()
    
    def update_clock(self):
        now = datetime.now().strftime("%H:%M:%S")
        self.clock_label.config(text=now)
        self.root.after(1000, self.update_clock)

def main():
    root = tk.Tk()
    app = ModernLidarMappingGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
