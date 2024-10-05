import cv2
import numpy as np
import serial
import math
import concurrent.futures
import subprocess

def calculate_steering_angle(focal_length_mm, sensor_width_mm, image_width_pixels, 
                             object_height_pixels, x1, block_right_edge_x, block_left_edge_x, is_green):
    # Calculate horizontal field of view
    horizontal_fov = 2 * math.atan(sensor_width_mm / (2 * focal_length_mm))
    
    # Calculate pixels per millimeter
    pixels_per_mm = image_width_pixels / sensor_width_mm
    
    # Estimate distance to object (assuming object height is known)
    ACTUAL_OBJECT_HEIGHT_MM = 50  # Adjust this based on your actual object height
    distance_mm = (focal_length_mm * ACTUAL_OBJECT_HEIGHT_MM * image_width_pixels) / (object_height_pixels * sensor_width_mm)
    
    # Calculate angular position of the object
    pixels_from_center = (x1 + (block_right_edge_x - block_left_edge_x) / 2) - (image_width_pixels / 2)
    angle_rad = math.atan2(pixels_from_center, pixels_per_mm * focal_length_mm)
    angle_deg = math.degrees(angle_rad)
    
    # Adjust steering based on whether it's green (left) or red (right)
    if is_green:
        steering_angle = -angle_deg  # Turn left for green
    else:
        steering_angle = angle_deg  # Turn right for red
    
    return steering_angle, distance_mm

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
