from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import numpy as np
import os
from threading import Lock
import matplotlib.pyplot as plt

from timer_manager import TimerManager


class SerialPlotter(QtWidgets.QWidget):
    """
    Handles visualization of data from HILOController.
    This is a passive component that only displays data and forwards user actions to HILOController.
    """
    def __init__(self, hilo_controller, parent=None):
        super().__init__(parent)
        self.hilo = hilo_controller
        self.setGeometry(100, 100, 1200, 800)

        self.isRun = True
        self.data_lock = Lock()
        self.displayed_generations = 3

        self.current_params_dots = []
        # self.param_colors = ['r', 'g', 'b', 'c', 'm', 'orange']
        self.param_colors = [
            QtGui.QColor.fromRgbF(r, g, b, 1.0) for (r,g,b) in plt.cm.tab10.colors
        ]

        self.timer_manager = TimerManager()
        self.timer_manager.register_callback(self.update_plots, interval=100)

        self.initUI()

    def initUI(self):
        layout = QtWidgets.QVBoxLayout()
        self.setLayout(layout)

        top_bar = QtWidgets.QHBoxLayout()
        layout.addLayout(top_bar)

        self.btn_start = QtWidgets.QPushButton("Start Optimization")
        self.btn_start.clicked.connect(self.on_start_optimization)
        top_bar.addWidget(self.btn_start)

        self.btn_stop = QtWidgets.QPushButton("Stop Optimization")
        self.btn_stop.clicked.connect(self.on_stop_optimization)
        self.btn_stop.setEnabled(True)
        top_bar.addWidget(self.btn_stop)

        self.btn_skip = QtWidgets.QPushButton("Skip Condition")
        self.btn_skip.clicked.connect(self.on_skip_condition)
        top_bar.addWidget(self.btn_skip)

        self.gen_dash_lines_condition = []
        self.gen_dash_lines_params = []

        row1 = QtWidgets.QHBoxLayout()
        row2 = QtWidgets.QHBoxLayout()
        layout.addLayout(row1)
        layout.addLayout(row2)

        pg.setConfigOption('background', 'w')

        # Top Left
        self.plot_condition = pg.PlotWidget(title="Per Condition (Estimate)")
        self.plot_condition.setLabel('bottom', 'Condition Number')
        self.plot_condition.setLabel('left', 'Met Cost [W]')
        self.plot_condition.setXRange(0, self.hilo.lambdaPopSize * self.displayed_generations, padding=0)
        self.curve_condition = self.plot_condition.plot(
            pen=pg.mkPen('b', width=2), symbol='o', symbolSize=8, symbolBrush='b'
        )
        row1.addWidget(self.plot_condition)

        # Top Right
        self.plot_params = pg.PlotWidget(title="Parameter Values")
        self.plot_params.setLabel('bottom', 'Condition Number')
        self.plot_params.setLabel('left', 'Param Value (scaled)')
        self.plot_params.setXRange(0, self.hilo.lambdaPopSize * self.displayed_generations, padding=0)

        self.param_curves = []
        color_list = self.param_colors

        self.param_legend = pg.LegendItem((100, 60), offset=(80, 20))
        self.param_legend.setParentItem(self.plot_params.graphicsItem())

        if hasattr(self.param_legend, "setColumnCount"):
            self.param_legend.setColumnCount(6)

        if hasattr(self.param_legend, "layout"):
            self.param_legend.layout.setHorizontalSpacing(25)
            self.param_legend.layout.setVerticalSpacing(0)

        for i in range(6):
            color = color_list[i % len(color_list)]
            curve = self.plot_params.plot(
                pen=pg.mkPen(color, width=2),
                symbol=None, 
                symbolBrush=color, 
                symbolSize=6
            )
            self.param_legend.addItem(curve, f'p{i + 1}')
            self.param_curves.append(curve)
            
        row1.addWidget(self.plot_params)

        # Bottom Left
        self.plot_breath = pg.PlotWidget(title="Per Breath")
        self.plot_breath.setLabel('bottom', 'Time [s]')
        self.plot_breath.setLabel('left', 'Met Cost [W]')
        self.plot_breath.setXRange(0, self.hilo.timeToEstimate, padding=0)
        self.curve_breath = self.plot_breath.plot(
            pen=pg.mkPen('g', width=2),
            symbol='o', symbolSize=6, symbolBrush='g'
        )
        row2.addWidget(self.plot_breath)

        # Bottom Right
        self.plot_torque = pg.PlotWidget(title="Torque Profile")
        self.plot_torque.setLabel('bottom', 'Gait Cycle [%]')
        self.plot_torque.setLabel('left', 'Torque [Nm]')
        self.plot_torque.setXRange(0, 100, padding=0)
        self.curve_torque = self.plot_torque.plot(
            pen=pg.mkPen('b', width=3)
        )
        self.scatter_torque = self.plot_torque.plot(
            pen=None, symbol='o', symbolSize=8, symbolBrush='r'
        )
        row2.addWidget(self.plot_torque)

    def on_skip_condition(self):
        """Forward skip condition button click to HILOController"""
        print("[SerialPlotter] Skip Condition button clicked.")
        self.hilo.skipCondition()
        self.hilo.dataCollectFlag = False
    
    def on_start_optimization(self):
        """Forward start optimization button click to HILOController"""
        print("[SerialPlotter] Start Optimization button clicked.")
        self.hilo.start_optimization()
        # Disable the button after starting to prevent multiple starts
        self.btn_start.setEnabled(False)
        self.btn_start.setText("Optimization Running...")
    
    def on_stop_optimization(self):
        """Forward stop optimization button click to the controller"""
        print("[SerialPlotter] Stop Optimization button clicked.")
        self.hilo.stop_optimization()
        self.btn_start.setEnabled(True)
        self.btn_start.setText("Start Optimization")

    def update_plots(self):
        """Update all plots with current data from HILOController"""
        if not self.isRun:
            return

        # If requested, signal that frame has been saved
        if getattr(self.hilo, "frameSaveFlag", False):
            self.save_screenshot()
            self.hilo.frameSaveFlag = False

        # Calculate visible range for plots
        gen_range_start = max(0, self.hilo.genCnt - self.displayed_generations)
        gen_range_end = self.hilo.genCnt

        with self.data_lock:
            # Update the metabolic cost plot
            met_estimate = self.hilo.met_estimate
            if len(met_estimate) > 0:
                x_vals = np.arange(1, len(met_estimate) + 1)
                self.curve_condition.setData(x_vals, met_estimate)
                x_min = gen_range_start * self.hilo.lambdaPopSize + 1
                x_max = gen_range_end * self.hilo.lambdaPopSize
                self.plot_condition.setXRange(x_min-0.5, x_max+0.5, padding=0)

                # Update generation boundary lines
                for line in self.gen_dash_lines_condition:
                    self.plot_condition.removeItem(line)
                self.gen_dash_lines_condition.clear()
                for i in range(1, self.hilo.genCnt):
                    vline = pg.InfiniteLine(pos=i * self.hilo.lambdaPopSize, angle=90,
                                            pen=pg.mkPen('gray', style=QtCore.Qt.DashLine))
                    self.plot_condition.addItem(vline)
                    self.gen_dash_lines_condition.append(vline)

            # Update the parameter values plot
            if len(self.hilo.candidate_params_list) > 0:
                candidate_list = self.hilo.candidate_params_list[0]
                if len(candidate_list) > 0:
                    x_params = np.arange(1, len(candidate_list) + 1)
                    arr = np.array(candidate_list)
                    for i in range(min(arr.shape[1], len(self.param_curves))):
                        self.param_curves[i].setData(x_params, arr[:, i])
                    x_min = gen_range_start * self.hilo.lambdaPopSize + 1
                    x_max = gen_range_end * self.hilo.lambdaPopSize
                    self.plot_params.setXRange(x_min-0.5, x_max+0.5, padding=0)
                    self.plot_params.setYRange(0, 10, padding=0)

                    # Update generation boundary lines
                    for line in self.gen_dash_lines_params:
                        self.plot_params.removeItem(line)
                    self.gen_dash_lines_params.clear()
                    for i in range(1, self.hilo.genCnt):
                        vline = pg.InfiniteLine(pos=i * self.hilo.lambdaPopSize, angle=90,
                                                pen=pg.mkPen('gray', style=QtCore.Qt.DashLine))
                        self.plot_params.addItem(vline)
                        self.gen_dash_lines_params.append(vline)
                    # Remove previous highlight dots
                    for dot in self.current_params_dots:
                        self.plot_params.removeItem(dot)
                    self.current_params_dots.clear()

                    # Add highlight dots for current condition parameters
                    current_condition_index = min(self.hilo.condCnt, len(candidate_list) - 1)
                    if current_condition_index >= 0 and current_condition_index < len(candidate_list):
                        current_params = candidate_list[current_condition_index]
                        condition_x = current_condition_index + 1
                        
                        # Create scatter plot items for each parameter
                        for i, param_value in enumerate(current_params):
                            if i < len(self.param_colors):
                                # Create a larger, more visible dot for highlighting
                                highlight_dot = pg.ScatterPlotItem(
                                    pos=[(condition_x, param_value)],
                                    size=10,  # Larger size for highlighting
                                    pen=pg.mkPen('black', width=2),  # Black border for white background
                                    brush=pg.mkBrush(self.param_colors[i]),  # Same color as the line
                                    symbol='o'
                                )
                                self.plot_params.addItem(highlight_dot)
                                self.current_params_dots.append(highlight_dot)

            # Update breath data plot
            met_list = self.hilo.met_list
            if len(met_list[0]) == len(met_list[3]):
                if len(met_list[0]) > 0:
                    self.curve_breath.setData(met_list[0], met_list[3])
                else:
                    # Clear the breath plot when data is reset
                    self.curve_breath.setData([], [])

            # Update torque profile plot
            torque_profile = self.hilo.torqueDesired
            if isinstance(torque_profile, np.ndarray) and torque_profile.size > 0:
                x_t = np.linspace(0, 100, len(torque_profile))
                self.curve_torque.setData(x_t, torque_profile)
                if self.hilo.torque_dots is not None:
                    dx, dy = self.hilo.torque_dots
                    self.scatter_torque.setData(dx, dy)
                else:
                    self.scatter_torque.setData([], [])

    def save_screenshot(self):
        """
        Save a screenshot of the current plotting window
        """
        try:
            # Generate filename based on current condition and generation
            filename = f"cond_{self.hilo.condCnt + 1}_gen_{self.hilo.genCnt}.png"
            filepath = os.path.join(self.hilo.frame_path, filename)
            
            # Ensure the directory exists
            os.makedirs(os.path.dirname(filepath), exist_ok=True)
            
            # Capture screenshot of the entire widget
            pixmap = self.grab()
            
            # Save the screenshot
            success = pixmap.save(filepath, 'PNG')
            
            if success:
                print(f"Screenshot saved: {filename}")
            else:
                print(f"Failed to save screenshot: {filename}")
                
        except Exception as e:
            print(f"Error saving screenshot: {e}")
            import traceback
            traceback.print_exc()

    def closeEvent(self, event):
        """Handle window close event by shutting down HILOController gracefully"""
        self.isRun = False

        for dot in self.current_params_dots:
            if hasattr(self, 'plot_params'):
                self.plot_params.removeItem(dot)
        self.current_params_dots.clear()
        
        self.timer_manager.unregister_callback(self.update_plots)
        self.hilo.isRun = False
        self.hilo.dataCollectFlag = False
        self.hilo.close()
        
        super().closeEvent(event)