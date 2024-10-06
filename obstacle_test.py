import cv2
import numpy as np
import serial
import math
import concurrent.futures
import subprocess
import math

def calculate_distance(real_width_cm, real_height_cm, img_width_px, img_height_px):
  
    # -------------------------
    # Hardcoded Camera Constants
    # -------------------------

    # Sensor specifications
    sensor_width_mm = 3.68      # Sensor width in millimeters
    sensor_height_mm = 2.76     # Sensor height in millimeters

    # Pixel size in micrometers
    pixel_width_um = 1.12        # Pixel width in micrometers
    pixel_height_um = 1.12       # Pixel height in micrometers

    # Convert pixel size to centimeters
    pixel_width_cm = pixel_width_um * 1e-4   # 1 µm = 1e-4 cm
    pixel_height_cm = pixel_height_um * 1e-4 # 1 µm = 1e-4 cm

    # Focal length in millimeters
    focal_length_mm = 3.6        # Focal length in millimeters

    # Convert focal length to centimeters
    focal_length_cm = focal_length_mm / 10.0  # 1 cm = 10 mm

    # -------------------------
    # Calculate Distance
    # -------------------------

    # Calculate ratios to determine if the object is fully within the FOV
    ratio_width = img_width_px / real_width_cm
    ratio_height = img_height_px / real_height_cm

    if math.isclose(ratio_width, ratio_height, rel_tol=1e-2):
        # Object is fully within the Field of View (FOV)
        # Using height for distance calculation
        distance_cm = (focal_length_cm * real_height_cm) / (img_height_px * pixel_height_cm)
    else:
        # Object is partially outside the FOV
        # Use the constraining dimension (the one with the smaller ratio)
        min_ratio = min(ratio_width, ratio_height)
        if min_ratio == ratio_width:
            # Width is the constraining dimension
            distance_cm = (focal_length_cm * real_width_cm) / (img_width_px * pixel_width_cm)
        else:
            # Height is the constraining dimension
            distance_cm = (focal_length_cm * real_height_cm) / (img_height_px * pixel_height_cm)

    return distance_cm


def detect_red_histogram(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    kernel = np.ones((5,5), np.uint8)
    hsv = cv2.morphologyEx(hsv, cv2.MORPH_CLOSE, kernel)
    hsv = cv2.morphologyEx(hsv, cv2.MORPH_OPEN, kernel)
    
    lower_red1 = np.array([0, 40, 40])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 40, 40])
    upper_red2 = np.array([180, 255, 255])
    
    time1 = time.time()
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2
    
    hist = cv2.calcHist([hsv], [0, 1], mask, [180, 256], [0, 180, 0, 256])
    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
    
    backproj = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], 1)
    
    _, thresh = cv2.threshold(backproj, 50, 255, cv2.THRESH_BINARY)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    time2 = time.time()
    print(f"Red detection time: {time2-time1:.4f} seconds")
    
    if contours and max(cv2.contourArea(c) for c in contours) > 500:
        max_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(max_contour)
        return True, (x, y, w, h)
    return False, None

def detect_green_histogram(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_green1 = np.array([40, 40, 40])
    upper_green1 = np.array([80, 255, 255])
    lower_green2 = np.array([35, 40, 40])
    upper_green2 = np.array([45, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_green1, upper_green1)
    mask2 = cv2.inRange(hsv, lower_green2, upper_green2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    hist = cv2.calcHist([hsv], [0, 1], mask, [180, 256], [0, 180, 0, 256])
    cv2.normalize(hist, hist, 0, 255, cv2.NORM_MINMAX)
    
    backproj = cv2.calcBackProject([hsv], [0, 1], hist, [0, 180, 0, 256], 1)
    
    _, thresh = cv2.threshold(backproj, 5, 255, cv2.THRESH_BINARY)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours and max(cv2.contourArea(c) for c in contours) > 500:
        max_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(max_contour)
        return True, (x, y, w, h)
    return False, None


def main():
    # Initialize libcamera-vid
    libcamera_cmd = ['libcamera-vid', '--inline', '-t', '0', '--width', '3280', '--height', '2464', '--codec', 'mjpeg', '-o', '-']
    camera_process = subprocess.Popen(libcamera_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    mjpeg_buffer = b""

    # Initialize serial communication with Arduino
    ser = serial.Serial('/dev/ttyACM0', 9600)  # Adjust port as needed

    # Camera constants
    FOCAL_LENGTH_MM = 3.04  # Focal length of the camera
    SENSOR_WIDTH_MM = 3.68  # Width of the camera sensor
    IMAGE_WIDTH_PIXELS = 3280  # Width of the captured image in pixels

    with concurrent.futures.ThreadPoolExecutor() as executor:
        try:
            while True:
                buffer = camera_process.stdout.read(1024)
                if not buffer:
                    break
                mjpeg_buffer += buffer
                start = mjpeg_buffer.find(b'\xff\xd8')
                end = mjpeg_buffer.find(b'\xff\xd9')
                if start != -1 and end != -1 and start < end:
                    jpg_data = mjpeg_buffer[start:end+2]
                    mjpeg_buffer = mjpeg_buffer[end+2:]
                    frame = cv2.imdecode(np.frombuffer(jpg_data, dtype=np.uint8), cv2.IMREAD_COLOR)

                    # Detect blocks using concurrent execution
                    future_red = executor.submit(detect_red_histogram, frame)
                    future_green = executor.submit(detect_green_histogram, frame)
                    
                    red_detected, red_box = future_red.result()
                    green_detected, green_box = future_green.result()

                    if red_detected and not green_detected:
                        print("Red is detected")
                        steering_angle, distance = calculate_steering_angle(
                            FOCAL_LENGTH_MM, SENSOR_WIDTH_MM, IMAGE_WIDTH_PIXELS, red_box[3],
                            red_box[0] + red_box[2]//2, red_box[0] + red_box[2], red_box[0], False
                        )
                        message = f"r{steering_angle:.2f}"
                        print(distance)
                        #ser.write(message.encode())
                        
                    elif green_detected and not red_detected:
                        print("Green is detected")
                        steering_angle, distance = calculate_steering_angle(
                            FOCAL_LENGTH_MM, SENSOR_WIDTH_MM, IMAGE_WIDTH_PIXELS, green_box[3],
                            green_box[0] + green_box[2]//2, green_box[0] + green_box[2], green_box[0], True
                        )
                        message = f"g{steering_angle:.2f}"
                        print(distance)
                        #ser.write(message.encode())
                        
                    elif red_detected and green_detected:
                        print("Both red and green are detected")
                        # Choose the block with the larger area
                        red_area = red_box[2] * red_box[3]
                        green_area = green_box[2] * green_box[3]
                        if red_area > green_area:
                            steering_angle, distance = calculate_steering_angle(
                                FOCAL_LENGTH_MM, SENSOR_WIDTH_MM, IMAGE_WIDTH_PIXELS, red_box[3],
                                red_box[0] + red_box[2]//2, red_box[0] + red_box[2], red_box[0], False
                            )
                            print(distance)#message = f"r{steering_angle:.2f}"
                        else:
                            steering_angle, distance = calculate_steering_angle(
                                FOCAL_LENGTH_MM, SENSOR_WIDTH_MM, IMAGE_WIDTH_PIXELS, green_box[3],
                                green_box[0] + green_box[2]//2, green_box[0] + green_box[2], green_box[0], True
                            )
                            print(distance)
                            #message = f"g{steering_angle:.2f},{distance:.2f}"
                        #ser.write(message.encode())
                        
                    else:
                        print("Neither red nor green is detected")
                        ser.write(b'n')

                    # Display the frame (optional, for debugging)
                    #cv2.imshow('Frame', frame)

                    # Break the loop if 'q' is pressed
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

        finally:
            # Clean up
            camera_process.terminate()
            camera_process.wait()
            cv2.destroyAllWindows()
            ser.close()

if __name__ == "__main__":
    main()
  import serial
import time

def send_receive_message(port: str, baud_rate: int = 9600, timeout: int = 2):#more than 1 sec required to read and send message back
    # Set up the serial connection
    ser = serial.Serial(port, baudrate=baud_rate, timeout=timeout)
    time.sleep(2)  # Wait for the connection to establish

    # Send 'Hello, World!' to Arduino
    message = "Hello, World!"
    ser.write(message.encode())

    print(f"Message sent to Arduino: {message}")

    # Read the response from Arduino
    response = ser.readline().decode().strip()

    print(f"Message received from Arduino: {response}")

    ser.close()

# Example usage: port and baud rate depend on your setup
if __name__ == "__main__":
    send_receive_message('/dev/ttyUSB0', 9600)


import cv2
import numpy as np
import subprocess
import io
from PIL import Image
import time

def get_frame(camera_process):
    global mjpeg_buffer
    while True:
        chunk = camera_process.stdout.read(1024)
        if not chunk:
            break
        mjpeg_buffer += chunk
        a = mjpeg_buffer.find(b'\xff\xd8')
        b = mjpeg_buffer.find(b'\xff\xd9')
        if a != -1 and b != -1:
            jpg = mjpeg_buffer[a:b+2]
            mjpeg_buffer = mjpeg_buffer[b+2:]
            return cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

def calibrate():
    # Initialize libcamera-vid
    libcamera_cmd = ['libcamera-vid', '--inline', '-t', '0', '--width', '3280', '--height', '2464', '--codec', 'mjpeg', '-o', '-']
    camera_process = subprocess.Popen(libcamera_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Known object size in cm
    real_size = float(input("Enter the real size of the object in cm: "))

    pixel_sizes = []
    distances = []

    print("Move the object to different distances and press 'c' to capture, 'q' to quit.")

    while True:
        frame = get_frame(camera_process)
        cv2.imshow('Frame', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('c'):
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.imshow('Detected Object', frame)

                pixel_size = max(w, h)
                pixel_sizes.append(pixel_size)
                
                distance = float(input("Enter the distance of the object from the camera in cm: "))
                distances.append(distance)

                print(f"Captured: Distance = {distance} cm, Pixel Size = {pixel_size}")

        elif key == ord('q'):
            break

    camera_process.terminate()
    cv2.destroyAllWindows()

    # Calculate the relation
    pixel_sizes = np.array(pixel_sizes)
    distances = np.array(distances)
    
    # Fit a curve: pixel_size = a * (distance^b)
    log_distances = np.log(distances)
    log_pixel_sizes = np.log(pixel_sizes)
    coeffs = np.polyfit(log_distances, log_pixel_sizes, 1)
    a = np.exp(coeffs[1])
    b = coeffs[0]

    print(f"\nCalibration complete.")
    print(f"Relation: pixel_size = {a:.4f} * (distance^{b:.4f})")
    print(f"To get distance from pixel size: distance = (pixel_size / {a:.4f})^(1/{b:.4f})")
    
    return a, b, real_size

def estimate_distance(pixel_size, a, b, real_size):
    # Estimate distance based on pixel size
    estimated_distance = (pixel_size / a) ** (1/b)
    
    # Calculate real-world size at this distance
    real_world_size = (real_size * pixel_size) / (a * (estimated_distance ** b))
    
    return estimated_distance, real_world_size

if __name__ == "__main__":
    a, b, real_size = calibrate()

