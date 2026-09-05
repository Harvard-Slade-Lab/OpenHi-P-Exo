import time
import threading
import cv2

from metabolics_extract import find_window_containing_title, windowGrab, click_and_crop, extract_data

class MetabolicsCollector(threading.Thread):
    def __init__(self, controller, boundaries, capture_interval=0.5):
        super().__init__()
        self.controller = controller  # Now takes controller instead of serial_plot
        self.boundaries = boundaries
        self.capture_interval = capture_interval
        self.is_running = True

    def run(self):
        print("Starting MetabolicsCollector thread.")
        try:
            self.stream_metabolics()
        except Exception as e:
            print(f"Exception in MetabolicsCollector thread: {e}")

    def stream_metabolics(self):
        """Stream metabolics data from screen capture"""        
        
        while self.is_running and self.controller.isRun:
            if self.controller.dataCollectFlag:
                image = windowGrab(self.controller.hwnd)
                t, vo2, vco2 = extract_data(image, self.boundaries)

                # Define VO2 and VCO2 boundaries (from reference code)
                boundary_vo2_Lower = 100
                boundary_vo2_Upper = 3000
                boundary_vco2_Lower = 100
                boundary_vco2_Upper = 3000
                
                # Define VO2/VCO2 ratio boundaries
                boundary_ratio_Lower = 0.6
                boundary_ratio_Upper = 2.2
                
                # Calculate VO2/VCO2 ratio
                ratio = vo2 / vco2 if vco2 != 0 else float('inf')  # Prevent division by zero
                
                # Check if VO2, VCO2, and their ratio are within specified boundaries
                if not ((boundary_vo2_Lower <= vo2 <= boundary_vo2_Upper) and 
                        (boundary_vco2_Lower <= vco2 <= boundary_vco2_Upper) and 
                        (boundary_ratio_Lower <= ratio <= boundary_ratio_Upper) and
                        (last_recorded_time == 0 or last_recorded_time <= t <= last_recorded_time + 30)):
                    # Determine which boundaries are violated
                    print("\n")
                    if not (boundary_vo2_Lower <= vo2 <= boundary_vo2_Upper):
                        print(f"VO2 ({vo2}) is out of bounds: Lower={boundary_vo2_Lower}, Upper={boundary_vo2_Upper}")
                    if not (boundary_vco2_Lower <= vco2 <= boundary_vco2_Upper):
                        print(f"VCO2 ({vco2}) is out of bounds: Lower={boundary_vco2_Lower}, Upper={boundary_vco2_Upper}")
                    if not (boundary_ratio_Lower <= ratio <= boundary_ratio_Upper):
                        print(f"VO2/VCO2 ratio ({ratio:.2f}) is out of bounds: Lower={boundary_ratio_Lower}, Upper={boundary_ratio_Upper}")
                    if not (last_recorded_time == 0 or last_recorded_time <= t <= last_recorded_time + 30):
                        print(f"Wrong time ({t}) detected: Previous={last_recorded_time}, Current={t}")
                    print("\nData excluded due to exceeding boundaries\n")
                    time.sleep(self.capture_interval)
                    continue
                
                # Only forward new breath data (when time has changed)
                if t != last_recorded_time:
                    
                    # Calculate metabolic cost
                    met_cost = (16.48 * vo2 + 4.48 * vco2) / 60.0
                    
                    # Forward the data directly to the controller
                    self.controller.process_breath(t, vo2, vco2, met_cost)
                    
                    # Update last recorded time
                    last_recorded_time = t
                
                time.sleep(self.capture_interval)
            else:
                last_recorded_time = 0  # Last recorded absolute time
                # Small delay when not collecting data to avoid CPU thrashing
                time.sleep(0.1)

    def stop(self):
        self.is_running = False


def init_met_task(controller, boundaries, windowname='Omnia'):
    controller.hwnd = find_window_containing_title(windowname)
    if controller.hwnd is None:
        print(f"Window with title '{windowname}' not found. Exiting.")
        return

    image = windowGrab(controller.hwnd)

    if boundaries is None:
        refPt = []
        selection_count = [0]
        print('Select [Time, VO2, VCO2]')

        cv2.imshow("screen", image)
        cv2.setMouseCallback("screen", click_and_crop, [refPt, controller.final_boundaries, selection_count, image])
        cv2.waitKey(0)
        print(f'Boundaries:', controller.final_boundaries)
    else:
        controller.final_boundaries = boundaries

    t, vo2, vco2 = extract_data(image, controller.final_boundaries)
    met_cost = (16.48*vo2 + 4.48*vco2)/60.0
    print('Time:', t, ' / VO2:', vo2, ' / VCO2:', vco2, ' / Metabolics:', met_cost)

    controller.metConnected = True

    return controller.final_boundaries