import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Circle
import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QLabel, QComboBox, QTextEdit, 
                             QGroupBox, QCheckBox, QSlider, QDoubleSpinBox, QSpinBox,
                             QMessageBox, QFileDialog, QProgressBar)
from PyQt5.QtCore import QTimer, Qt, QThread, pyqtSignal, QMutex
from PyQt5.QtGui import QFont
import time
from rplidar import RPLidar
import csv

class LidarWorker(QThread):
    data_ready = pyqtSignal(list)
    error_signal = pyqtSignal(str)
    info_signal = pyqtSignal(dict)
    
    def __init__(self, port):
        super().__init__()
        self.port = port
        self.lidar = None
        self.is_scanning = False
        self.mutex = QMutex()
        
    def connect_lidar(self):
        try:
            self.lidar = RPLidar(self.port)
            info = self.lidar.get_info()
            health = self.lidar.get_health()
            self.info_signal.emit({
                'info': info,
                'health': health,
                'connected': True
            })
            return True
        except Exception as e:
            self.error_signal.emit(f"Connection failed: {str(e)}")
            return False
            
    def disconnect_lidar(self):
        self.mutex.lock()
        self.is_scanning = False
        if self.lidar:
            try:
                self.lidar.stop()
                time.sleep(0.1)
                self.lidar.disconnect()
            except:
                pass
            finally:
                self.lidar = None
        self.mutex.unlock()
            
    def run(self):
        if not self.connect_lidar():
            return
            
        self.is_scanning = True
        consecutive_errors = 0
        max_consecutive_errors = 3
        
        try:
            while self.is_scanning and consecutive_errors < max_consecutive_errors:
                try:
                    # Get one scan at a time
                    scan = next(self.lidar.iter_scans(max_buf_meas=500))
                    points = []
                    for quality, angle, distance in scan:
                        if distance > 0 and quality > 0:
                            angle_rad = np.radians(angle)
                            x = distance * np.cos(angle_rad)
                            y = distance * np.sin(angle_rad)
                            points.append((x, y))
                    
                    if points:
                        self.data_ready.emit(points)
                        consecutive_errors = 0
                    else:
                        consecutive_errors += 1
                        
                    time.sleep(0.02)  # Small delay to prevent buffer overflow
                    
                except StopIteration:
                    break
                except Exception as e:
                    consecutive_errors += 1
                    print(f"Scan error: {e}")
                    time.sleep(0.1)
                    
        except Exception as e:
            self.error_signal.emit(f"Scanning thread error: {str(e)}")
        finally:
            self.disconnect_lidar()

class RealTimePlot(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(10, 8), dpi=100, facecolor='#f8f9fa')
        super().__init__(self.fig)
        self.setParent(parent)
        
        self.ax = self.fig.add_subplot(111)
        self.setup_plot()
        
        self.points = []
        self.scatter = None
        self.robot_marker = None
        self.max_points = 5000
        
    def setup_plot(self):
        """إعداد الرسم البياني مع كائن scatter فعلي"""
        self.ax.clear()
        self.ax.set_facecolor('#2c3e50')
        self.ax.set_xlim(-6000, 6000)
        self.ax.set_ylim(-6000, 6000)
        self.ax.set_xlabel('X Distance (mm)', fontsize=12, color='white')
        self.ax.set_ylabel('Y Distance (mm)', fontsize=12, color='white')
        self.ax.set_title('Real-Time Environment Mapping', fontsize=14, 
                         fontweight='bold', color='white', pad=20)
        self.ax.grid(True, alpha=0.3, color='white', linestyle='--')
        
        # إضافة دوائر المسافات
        for radius in [1000, 2000, 3000, 4000, 5000]:
            circle = Circle((0, 0), radius, fill=False, color='green', 
                          alpha=0.2, linestyle='--')
            self.ax.add_patch(circle)
            
        # إنشاء كائن scatter فعلي من البداية
        self.scatter = self.ax.scatter([], [], s=3, c='#3498db', alpha=0.7)
        self.robot_marker = self.ax.scatter([0], [0], s=200, c='#e74c3c', 
                                          marker='^', edgecolors='white', linewidth=2)
        
        self.draw_idle()
        
    def update_plot(self, new_points):
        """تحديث الرسم مع التأكد من وجود كائن scatter"""
        if new_points:
            self.points.extend(new_points)
            
            # تحديد عدد النقاط لمنع مشاكل الذاكرة
            if len(self.points) > self.max_points:
                self.points = self.points[-self.max_points:]
            
        # التأكد من وجود كائن scatter
        if self.scatter is None:
            print("Creating new scatter plot...")
            self.scatter = self.ax.scatter([], [], s=3, c='#3498db', alpha=0.7)
        
        if self.points:
            points_array = np.array(self.points)
            try:
                self.scatter.set_offsets(points_array)
                self.scatter.set_sizes([3] * len(points_array))
            except Exception as e:
                print(f"Error updating scatter: {e}")
                # إعادة إنشاء كائن scatter إذا فشل التحديث
                self.scatter.remove()
                self.scatter = self.ax.scatter(points_array[:, 0], points_array[:, 1], 
                                             s=3, c='#3498db', alpha=0.7)
        
        self.draw_idle()
        
    def clear_points(self):
        """مسح جميع النقاط"""
        self.points = []
        if self.scatter:
            self.scatter.set_offsets([])
        self.draw_idle()
        
    def update_visualization_settings(self, point_size, alpha, show_grid):
        """تحديث إعدادات التصور"""
        if self.scatter and len(self.points) > 0:
            self.scatter.set_sizes([point_size] * len(self.points))
            self.scatter.set_alpha(alpha)
        self.ax.grid(show_grid)
        self.draw_idle()

class FixedLidarGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.lidar_worker = None
        self.all_points = []
        self.scan_count = 0
        self.start_time = None
        self.connection_state = False
        
        self.setup_ui()
        self.setup_timers()
        
    def setup_ui(self):
        self.setWindowTitle("RPLidar Mapping - Fixed Version")
        self.setGeometry(100, 100, 1400, 900)
        self.setStyleSheet("""
            QMainWindow { background-color: #2c3e50; }
            QGroupBox {
                font-weight: bold; font-size: 12px;
                border: 2px solid #34495e; border-radius: 8px;
                margin-top: 1ex; padding-top: 10px;
                background-color: #34495e; color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin; left: 10px;
                padding: 0 5px 0 5px; color: #ecf0f1;
            }
            QPushButton {
                background-color: #3498db; border: none;
                color: white; padding: 8px 16px;
                border-radius: 4px; font-weight: bold;
            }
            QPushButton:hover { background-color: #2980b9; }
            QPushButton:disabled {
                background-color: #7f8c8d; color: #bdc3c7;
            }
            QTextEdit {
                border: 1px solid #34495e; border-radius: 4px;
                padding: 8px; background-color: #2c3e50;
                color: white; font-family: Consolas;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Left panel - Controls
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel)
        
        # Right panel - Visualization
        right_panel = self.create_visualization_panel()
        main_layout.addWidget(right_panel)
        
    def create_control_panel(self):
        panel = QWidget()
        panel.setMaximumWidth(350)
        layout = QVBoxLayout(panel)
        
        # Connection Group
        connection_group = QGroupBox("🔌 Device Connection")
        connection_layout = QVBoxLayout(connection_group)
        
        port_layout = QHBoxLayout()
        port_layout.addWidget(QLabel("COM Port:"))
        self.port_combo = QComboBox()
        self.port_combo.addItems(['COM3', 'COM4', 'COM5', 'COM6'])
        self.port_combo.setCurrentText('COM3')
        port_layout.addWidget(self.port_combo)
        connection_layout.addLayout(port_layout)
        
        self.connect_btn = QPushButton("Connect to RPLidar")
        self.connect_btn.clicked.connect(self.toggle_connection)
        connection_layout.addWidget(self.connect_btn)
        
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold;")
        connection_layout.addWidget(self.status_label)
        
        # Scan Control Group
        scan_group = QGroupBox("🎯 Scan Control")
        scan_layout = QVBoxLayout(scan_group)
        
        self.scan_btn = QPushButton("Start Mapping")
        self.scan_btn.clicked.connect(self.start_scan)
        self.scan_btn.setEnabled(False)
        scan_layout.addWidget(self.scan_btn)
        
        self.stop_btn = QPushButton("Stop Scanning")
        self.stop_btn.clicked.connect(self.stop_scan)
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet("background-color: #e74c3c;")
        scan_layout.addWidget(self.stop_btn)
        
        # Visualization Settings
        vis_group = QGroupBox("📊 Visualization")
        vis_layout = QVBoxLayout(vis_group)
        
        point_layout = QHBoxLayout()
        point_layout.addWidget(QLabel("Point Size:"))
        self.point_size = QSpinBox()
        self.point_size.setRange(1, 10)
        self.point_size.setValue(3)
        self.point_size.valueChanged.connect(self.update_visualization)
        point_layout.addWidget(self.point_size)
        vis_layout.addLayout(point_layout)
        
        alpha_layout = QHBoxLayout()
        alpha_layout.addWidget(QLabel("Transparency:"))
        self.alpha_spin = QDoubleSpinBox()
        self.alpha_spin.setRange(0.1, 1.0)
        self.alpha_spin.setValue(0.7)
        self.alpha_spin.valueChanged.connect(self.update_visualization)
        alpha_layout.addWidget(self.alpha_spin)
        vis_layout.addLayout(alpha_layout)
        
        self.show_grid = QCheckBox("Show Grid")
        self.show_grid.setChecked(True)
        self.show_grid.stateChanged.connect(self.update_visualization)
        vis_layout.addWidget(self.show_grid)
        
        # Data Management
        data_group = QGroupBox("💾 Data Management")
        data_layout = QVBoxLayout(data_group)
        
        self.clear_btn = QPushButton("Clear Map")
        self.clear_btn.clicked.connect(self.clear_map)
        self.clear_btn.setEnabled(False)
        data_layout.addWidget(self.clear_btn)
        
        self.save_btn = QPushButton("Save Map")
        self.save_btn.clicked.connect(self.save_map)
        self.save_btn.setEnabled(False)
        data_layout.addWidget(self.save_btn)
        
        # Statistics
        stats_group = QGroupBox("📈 Statistics")
        stats_layout = QVBoxLayout(stats_group)
        
        self.stats_text = QTextEdit()
        self.stats_text.setMaximumHeight(150)
        self.stats_text.setPlainText("Ready to connect...")
        stats_layout.addWidget(self.stats_text)
        
        # Add all groups to layout
        layout.addWidget(connection_group)
        layout.addWidget(scan_group)
        layout.addWidget(vis_group)
        layout.addWidget(data_group)
        layout.addWidget(stats_group)
        layout.addStretch()
        
        return panel
        
    def create_visualization_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Create matplotlib canvas
        self.plot = RealTimePlot(self)
        layout.addWidget(self.plot)
        
        return panel
        
    def setup_timers(self):
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self.update_statistics)
        self.stats_timer.start(1000)
        
    def toggle_connection(self):
        if self.connection_state:
            self.disconnect_lidar()
        else:
            self.connect_lidar()
            
    def connect_lidar(self):
        port = self.port_combo.currentText()
        self.lidar_worker = LidarWorker(port)
        
        self.lidar_worker.data_ready.connect(self.on_data_received)
        self.lidar_worker.error_signal.connect(self.on_error)
        self.lidar_worker.info_signal.connect(self.on_lidar_info)
        
        self.connect_btn.setEnabled(False)
        self.connect_btn.setText("Connecting...")
        
        self.lidar_worker.start()
        
    def disconnect_lidar(self):
        if self.lidar_worker:
            self.lidar_worker.disconnect_lidar()
            if self.lidar_worker.isRunning():
                self.lidar_worker.wait(2000)
                
        self.connection_state = False
        self.connect_btn.setText("Connect to RPLidar")
        self.connect_btn.setEnabled(True)
        self.scan_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        self.status_label.setText("Status: Disconnected")
        self.status_label.setStyleSheet("color: #e74c3c; font-weight: bold;")
        self.update_statistics("Disconnected from RPLidar")
        
    def on_lidar_info(self, info):
        self.connection_state = True
        self.connect_btn.setText("Disconnect")
        self.connect_btn.setEnabled(True)
        self.scan_btn.setEnabled(True)
        self.clear_btn.setEnabled(True)
        self.save_btn.setEnabled(True)
        
        self.status_label.setText("Status: Connected")
        self.status_label.setStyleSheet("color: #2ecc71; font-weight: bold;")
        
        stats = f"Connected to RPLidar!\n\nModel: {info['info']['model']}\nFirmware: {info['info']['firmware']}\nHardware: {info['info']['hardware']}\nHealth: {info['health'][0]}\n\nClick 'Start Mapping' to begin!"
        self.update_statistics(stats)
        
    def on_data_received(self, points):
        """استقبال البيانات وتحديث الرسم"""
        try:
            self.all_points.extend(points)
            self.plot.update_plot(points)
            self.scan_count += 1
        except Exception as e:
            print(f"Error in on_data_received: {e}")
        
    def on_error(self, error_msg):
        QMessageBox.critical(self, "Error", error_msg)
        self.disconnect_lidar()
        
    def start_scan(self):
        if not self.connection_state:
            QMessageBox.warning(self, "Warning", "Not connected to RPLidar")
            return
            
        self.scan_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.start_time = time.time()
        self.scan_count = 0
        self.all_points = []
        self.plot.clear_points()
        
        self.update_statistics("Scanning started...")
        
    def stop_scan(self):
        self.scan_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        
        if self.start_time:
            duration = time.time() - self.start_time
            stats = f"Scan Completed!\n\nTotal Points: {len(self.all_points)}\nDuration: {duration:.1f}s\nScans Processed: {self.scan_count}\nPoints/Second: {len(self.all_points)/max(duration, 0.1):.0f}"
            self.update_statistics(stats)
            
    def update_visualization(self):
        """تحديث إعدادات التصور"""
        try:
            if hasattr(self, 'plot') and self.plot.scatter is not None:
                self.plot.update_visualization_settings(
                    self.point_size.value(),
                    self.alpha_spin.value(),
                    self.show_grid.isChecked()
                )
        except Exception as e:
            print(f"Error in update_visualization: {e}")
            
    def update_statistics(self, text=None):
        if text:
            self.stats_text.setPlainText(text)
        elif self.start_time and self.connection_state:
            elapsed = time.time() - self.start_time
            stats = (f"Scanning...\n\n"
                    f"Points: {len(self.all_points)}\n"
                    f"Duration: {elapsed:.1f}s\n"
                    f"Scans: {self.scan_count}\n"
                    f"Rate: {len(self.all_points)/max(elapsed, 0.1):.0f} pts/sec")
            self.stats_text.setPlainText(stats)
            
    def clear_map(self):
        reply = QMessageBox.question(self, "Clear Map", 
                                   "Clear all points from the map?",
                                   QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.all_points = []
            self.plot.clear_points()
            self.update_statistics("Map cleared - Ready for new scan")
            
    def save_map(self):
        if not self.all_points:
            QMessageBox.warning(self, "Warning", "No data to save")
            return
            
        filename, _ = QFileDialog.getSaveFileName(
            self, "Save Map", f"lidar_map_{time.strftime('%Y%m%d_%H%M%S')}",
            "PNG Files (*.png);;CSV Files (*.csv)")
            
        if filename:
            try:
                if filename.endswith('.png'):
                    self.plot.fig.savefig(filename, dpi=300, bbox_inches='tight', 
                                        facecolor='#f8f9fa')
                elif filename.endswith('.csv'):
                    with open(filename, 'w', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(['X', 'Y'])
                        writer.writerows(self.all_points)
                QMessageBox.information(self, "Success", f"Map saved successfully!")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Save failed: {str(e)}")
                
    def closeEvent(self, event):
        if self.lidar_worker and self.lidar_worker.isRunning():
            self.lidar_worker.disconnect_lidar()
            self.lidar_worker.wait(2000)
        event.accept()

def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = FixedLidarGUI()
    window.show()
    
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
