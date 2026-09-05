import win32gui
import win32ui
from PIL import Image
import pytesseract
import os
import numpy as np
import cv2
import re

from ctypes import windll

# Configure the Tesseract OCR path - download separately - move to \measurements\metabolics\
pytesseract.pytesseract.tesseract_cmd = os.path.join(os.path.dirname(__file__), "Tesseract-OCR", "tesseract.exe")

def proc_string(str_temp):
    """
    Processes a raw OCR string to clean and convert it to an integer.
    Replaces common misreads and removes non-digit characters.
    """
    # Replace common misreads before filtering digits
    str_temp = re.sub(r'(?<!7)/(?!7)', '7', str_temp)  # Replace '/' with '7' if not surrounded by '7'
    str_temp = re.sub(r'(?<!0)O(?!0)', '0', str_temp)  # Replace 'O' with '0' if not surrounded by '0'
    str_temp = re.sub(r'(?<!1)1(?!1)', '1', str_temp)  # Replace any isolated '1' (not surrounded by '1') with '1'
    str_temp = re.sub(r'(?<!5)§(?!5)', '5', str_temp)  # Replace '§' with '5' if not surrounded by '5'

    cleaned_str = ''.join(filter(str.isdigit, str_temp))  # Keep only digits

    if cleaned_str:
        return int(cleaned_str)
    else:
        print("String error, not an integer", str_temp)
        return -1

def formatTime(t_split):
    """
    Formats time based on whether input is in hh:mm:ss, mm:ss, or ss format.
    """
    if len(t_split) > 3:
        print("Error with time", t_split)
    elif len(t_split) == 3:  # hh:mm:ss
        t = 3600 * proc_string(t_split[0]) + 60 * proc_string(t_split[1]) + proc_string(t_split[2])
    elif len(t_split) == 2:  # mm:ss
        t = 60 * proc_string(t_split[0]) + proc_string(t_split[1])
    else: # ss
        raw_time = t_split[0]
        raw_time = raw_time.replace('\n', '').replace('\r', '') 
        if len(raw_time) > 4:
            # Case: hhmmss format
            t = 3600 * proc_string(raw_time[:-4]) + 60 * proc_string(raw_time[-4:-2]) + proc_string(raw_time[-2:])
        elif len(raw_time) > 2:
            # Case: mmss format
            t = 60 * proc_string(raw_time[:-2]) + proc_string(raw_time[-2:])
        else:
            # Case: ss
            t = proc_string(raw_time)
    return t

def find_window_containing_title(part_of_title):
    """
    Finds the window handle containing the specified title fragment.
    """
    def callback(hwnd, data):
        if part_of_title.lower() in win32gui.GetWindowText(hwnd).lower():
            data.append(hwnd)
    handles = []
    win32gui.EnumWindows(callback, handles)
    return handles[0] if handles else None

def windowGrab(hwnd):
    """
    Captures the screen content of a specific window, given by `hwnd` (handle),
    and returns it as a grayscale numpy array.
    """
    left, top, right, bot = win32gui.GetWindowRect(hwnd)
    w = right - left
    h = bot - top

    wDC = win32gui.GetWindowDC(hwnd)
    dcObj = win32ui.CreateDCFromHandle(wDC)
    cDC = dcObj.CreateCompatibleDC()

    dataBitMap = win32ui.CreateBitmap()
    dataBitMap.CreateCompatibleBitmap(dcObj, w, h)

    cDC.SelectObject(dataBitMap)

    result = windll.user32.PrintWindow(hwnd, cDC.GetSafeHdc(), 2)

    bmpinfo = dataBitMap.GetInfo()
    bmpstr = dataBitMap.GetBitmapBits(True)

    im = Image.frombuffer(
        'RGB',
        (bmpinfo['bmWidth'], bmpinfo['bmHeight']),
        bmpstr, 'raw', 'BGRX', 0, 1)

    # Free Resources
    dcObj.DeleteDC()
    cDC.DeleteDC()
    win32gui.ReleaseDC(hwnd, wDC)
    win32gui.DeleteObject(dataBitMap.GetHandle())

    # Convert PIL image to numpy array
    screen = np.array(im)

    # Convert image to grayscale
    output = cv2.cvtColor(screen, cv2.COLOR_BGR2GRAY)

    return output

def convert_image(image):
    """
    Enhances an image by resizing and thresholding for better OCR results.
    """
    high_res_image = cv2.resize(image, None, fx=10.0, fy=10.0, interpolation=cv2.INTER_CUBIC)
    ret1, image_final = cv2.threshold(high_res_image, 180, 255, cv2.THRESH_BINARY)
    return image_final

def click_and_crop(event, x, y, flags, param):
    """
    Mouse callback function for selecting regions on the image.
    """
    refPt, final_boundaries, selection_count, image = param

    # When the left mouse button is pressed
    if event == cv2.EVENT_LBUTTONDOWN:
        if selection_count[0] < 3:  # Allows selecting up to 3 areas
            refPt.append((x, y))  # Save the starting point

    # When the left mouse button is released
    elif event == cv2.EVENT_LBUTTONUP:
        if selection_count[0] < 3:
            refPt.append((x, y))  # Save the ending point
            final_boundaries.append((refPt[0], refPt[1]))
            selection_count[0] += 1  # Increment the count of selected areas
            refPt.clear()  # Reset refPt for the next selection
            clone = image.copy()
            # Display all saved areas on the image
            for pt1, pt2 in final_boundaries:
                cv2.rectangle(clone, pt1, pt2, (0, 255, 0), 2)
            cv2.imshow("screen", clone)
            if selection_count[0] == 3:
                cv2.destroyAllWindows()  # Close the window after 3 areas have been selected

    # When the mouse is moving (including during drag)
    elif event == cv2.EVENT_MOUSEMOVE:
        if selection_count[0] < 3:  # If less than 3 areas have been selected
            clone = image.copy()
            # Display already selected areas
            for pt1, pt2 in final_boundaries:
                cv2.rectangle(clone, pt1, pt2, (0, 255, 0), 2)
            # Display the currently selecting area (during drag)
            if refPt:
                cv2.rectangle(clone, refPt[0], (x, y), (0, 255, 0), 2)
            cv2.imshow("screen", clone)

def extract_data(image, final_boundaries):
    """
    Extracts metabolic data values from specified regions of the image.
    :param image: Grayscale image from which to extract data
    :param final_boundaries: Tuple of boundaries for Time, VO2, and VCO2 regions
    :return: Tuple containing extracted time, VO2, and VCO2 values
    """
    image_time = image[final_boundaries[0][0][1]:final_boundaries[0][1][1], final_boundaries[0][0][0]:final_boundaries[0][1][0]]
    image_vo2 = image[final_boundaries[1][0][1]:final_boundaries[1][1][1], final_boundaries[1][0][0]:final_boundaries[1][1][0]]
    image_vco2 = image[final_boundaries[2][0][1]:final_boundaries[2][1][1], final_boundaries[2][0][0]:final_boundaries[2][1][0]]

    time_raw = pytesseract.image_to_string(convert_image(image_time), config="--psm 7")
    vo2_raw = pytesseract.image_to_string(convert_image(image_vo2), config="--psm 7")
    vco2_raw = pytesseract.image_to_string(convert_image(image_vco2), config="--psm 7")

    time_raw = time_raw.replace('.', ':')
    t = formatTime(time_raw.split(':'))
    vo2 = proc_string(vo2_raw)
    vco2 = proc_string(vco2_raw)

    # Uncomment below lines to visualize problematic regions
    # if t == -1:
    #     Image.fromarray(image_time).show()
    # if vo2 == -1:
    #     Image.fromarray(image_vo2).show()
    # if vco2 == -1:
    #     Image.fromarray(image_vco2).show()

    return t, vo2, vco2

def metabolic_rate_estimation(t, y_meas, tau=42):
    """
    Estimates metabolic rate based on time and measured values.
    :param t: Time array
    :param y_meas: Measured metabolic values
    :param tau: Time constant
    :return: Tuple containing estimated metabolic rate, predicted values, and mean squared error
    """
    n_samp = len(t)
    A = np.zeros((n_samp,2))
    A[0,:] = [1,0]
    for i in range(1,n_samp):
        for j in range(2):
            dt = t[i] - t[i-1]
            if j == 0:
                A[i,j] = A[i-1,j]*(1-dt/tau)
            else:
                A[i,j] = A[i-1,j]*(1-dt/tau) + (dt/tau)
    # print(A)
    x_star = np.dot(np.linalg.pinv(A),y_meas)
    y_bar = np.dot(A,x_star)
    mean_squared_err = np.dot(np.transpose(y_bar-y_meas),(y_bar-y_meas))/n_samp
    met_est = x_star[1]

    return met_est, y_bar, mean_squared_err