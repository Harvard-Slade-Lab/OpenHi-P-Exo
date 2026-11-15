import sys
import time
from threading import Thread
from PyQt5 import QtWidgets

from serialPlotter import SerialPlotter
from metabolics_collect import init_met_task
from bleFunctions import DataSender, BleFunctions
from hiloMetCmaController import HILOMetCmaController

def main():
    # Eight generations of nine conditions, as in the study
    max_number_generations = 8

    windowname = 'Omnia'
    
    # OCR regions (Time, VO2, VCO2): None, so init_met_task asks you to draw them on the Omnia window
    boundaries = None

    # Ask for subject number
    subject_number = input("Enter subject number: ")
    
    body_weight = float(input("Enter body weight (kg): "))

    app = QtWidgets.QApplication(sys.argv)

    # Create the HILO controller (the main logic + CMA thread)
    hilo = HILOMetCmaController(
        subject_number=subject_number,
        max_generations=max_number_generations,
        body_weight=body_weight
    )

    # Connect to the exoskeleton over BLE
    ds = DataSender()
    ble_thread = Thread(
        target=BleFunctions.start_ble_task,
        args=(ds,),
        daemon=True
    )
    ble_thread.start()
    while not ds.bleConnected:
        time.sleep(0.1)
    # The controller must acknowledge a PING before an experiment can start
    while not ds.check_link():
        if not ds.bleConnected:
            print("BLE not connected; retrying.")
        elif ds.controller_faulted:
            print("The controller reports a latched fault; retrying. Reset the controller.")
        else:
            print("No ACK to PING from the controller; retrying. The controller must run the "
                  "firmware of this repository and the HM-10 RXD must be wired to PB6 (USART1_TX).")
        time.sleep(1.0)
    hilo.client = ds  # pass the BLE client to HILO
    # Frames never acknowledged stop the conditions. Set after the start-up PINGs, so their
    # faults do not leave a message in the GUI.
    ds.on_link_fault = hilo.on_link_fault

    # Read the metabolics (OCR of the Omnia window) in a separate thread
    boundaries = init_met_task(hilo, boundaries, windowname)
    hilo.start_Metabolics_collector(boundaries)

    # ---------------------------------------------------------------------
    # Create our PyQt application and the custom Metabolics-plot widget
    # ---------------------------------------------------------------------
    plotter = SerialPlotter(hilo_controller=hilo)
    plotter.setWindowTitle("HILO Metabolics Real-Time Plot")
    plotter.show()

    hilo.plotter = plotter

    # Start CMA optimizer in a separate thread
    hilo.start_CMA_thread()

    # Hand control over to the Qt event loop
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()