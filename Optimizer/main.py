import sys
import time
from threading import Thread
from PyQt5 import QtWidgets

from serialPlotter import SerialPlotter
from metabolics_collect import init_met_task
from bleFunctions import DataSender, BleFunctions
from hiloMetCmaController import HILOMetCmaController

def main():
    max_number_generations = 100

    # CMA-ES random seed. Fixed so an optimization run is reproducible;
    # it is also written to outcmaes/subject_<N>/subject_<N>_cma_settings.json.
    cma_seed = 0
    
    bleUse = True
    metUse = True

    windowname = 'Omnia'
    
    # bounding boxes for OCR (example)admin
    boundaries = None

    # Ask for subject number
    subject_number = input("Enter subject number: ")
    
    body_weight = float(input("Enter body weight (kg): "))

    app = QtWidgets.QApplication(sys.argv)

    # Create the HILO controller (the main logic + CMA thread)
    hilo = HILOMetCmaController(
        subject_number=subject_number,
        max_generations=max_number_generations,
        frame_path="frames",
        body_weight=body_weight,
        cma_seed=cma_seed
    )

    # Set up BLE if desired
    ds = DataSender()
    if bleUse:
        ble_thread = Thread(
            target=BleFunctions.start_ble_task,
            args=(ds,),
            daemon=True
        )
        ble_thread.start()
        while not ds.bleConnected:
            time.sleep(0.1)
        hilo.client = ds  # pass the BLE client to HILO

    # If you want to connect Metabolics (OCR) in a separate thread
    if metUse:
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