from multiprocessing import Process
from vision import qrcode
import zmq
import cv2
import time
from locorobo import LocoRobo
from locorobo import MotorDirection
from locorobo import WaitType
from locorobo import Data
import numpy as np

def get_robot(robots, name):
    robot = None

    for r in robots.values():
        print(r.name, r.communicator.mac, r.communicator.mac_str, r.communicator.mac_type)
        if r.name == name:
            robot = r
            break

    if not robot:
        raise Exception('Could not find robot with specified name')

    return robot

def cam():
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.bind("tcp://127.0.0.1:5557") #push socket

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()

        #Detect color
        if(color_detect(frame, "blue")):
            color = "blue"
        else:
            color="red"

        codes = qrcode(frame)
        socket.send_string(str(codes)+color) #Send QR reading to other process
    
def color_detect(image, color: str):
    # Format is [upper limit, lower limit]
    color_dict_HSV = {
        'black': [[180, 255, 30], [0, 0, 0]],
        'white': [[180, 18, 255], [0, 0, 231]],
        'red1': [[180, 255, 255], [159, 50, 70]],
        'red2': [[9, 255, 255], [0, 50, 70]],
        'green': [[89, 255, 255], [36, 50, 70]],
        'blue': [[128, 255, 255], [90, 50, 70]],
        'yellow': [[35, 255, 255], [25, 50, 70]],
        'purple': [[158, 255, 255], [129, 50, 70]],
        'orange': [[24, 255, 255], [10, 50, 70]],
        'gray': [[180, 18, 230], [0, 0, 40]]
    }
    
    HSVLowerLimit = np.array(color_dict_HSV[color][1])
    HSVUpperLimit = np.array(color_dict_HSV[color][0])
    
    # Since the images are naturally a bit noisy, apply a 15x15 blur filter
    # to soften the image before the rest of the operations.
    kernel_size = 15
    blur = cv2.blur(image, (kernel_size, kernel_size))

    # Convert the BGR frame to HSV color space
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    # The inRange will examine the image and form a mask off only the H,S,& V 
    mask = cv2.inRange(hsv, HSVLowerLimit, HSVUpperLimit)

    # Find all contours within the masked image
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # If any contours were found, we have more to do
    if len(contours) != 0:

        # Find the biggest contour by the area
        c = max(contours, key = cv2.contourArea)
        x,y,w,h = cv2.boundingRect(c)
   
        # Calculate the midpoint of the largest contour
        midx = int(x+w/2)
        midy = int(y+h/2)
        mid = (midx, midy)
        
        return mid
    return None

def robot():
    # Init pull socket
    context = zmq.Context()
    socket = context.socket(zmq.PULL)
    socket.setsockopt(zmq.CONFLATE, 1)
    socket.RCVTIMEO = 1000 #time in milliseconds
    socket.connect("tcp://127.0.0.1:5557") #pull socket

    LocoRobo.setup("/dev/ttyACM0")
    robots = LocoRobo.scan(2000)

    robot = get_robot(robots, "Rupert")
    robot.connect()
    robot.activate_motors()
    robot.enable_sensor(Data.ULTRASONIC, True)
    LocoRobo.wait(1) 

    last_read_qr = None

    while True:
        #Receive QR code from the camera process
        msg = socket.recv_string()
        msg = msg.lower()
        print(msg)
        dist = robot.get_sensor_value(Data.ULTRASONIC)

        #set direction
        if 'left' in msg:
            last_read_qr = "left"
        elif 'right' in msg:
            last_read_qr = "right"
        elif 'end' in msg:
            last_read_qr = "end"

        #set turn speed
        if 'blue' in msg:
            speed = 1
        elif 'red' in msg:
            speed = .5

        if dist > 10:
            robot.setup_wait(WaitType.DISTANCE, 5000) #move 5cm forward
            robot.move(MotorDirection.FORWARD, MotorDirection.FORWARD, 0.25, 0.25, True)
        else:
            if last_read_qr == 'left':
                robot.setup_wait(WaitType.ROTATION, 90000)
                robot.move(MotorDirection.BACKWARD, MotorDirection.FORWARD, speed, speed, True)
                
            elif last_read_qr == 'right':
                robot.setup_wait(WaitType.ROTATION, 90000)
                robot.move(MotorDirection.FORWARD, MotorDirection.BACKWARD, speed, speed, True)

            elif last_read_qr == 'end':
                robot.deactivate_motors()
                robot.disconnect()
                robot.stop()
                print("Reached the end")
                exit(1)

if __name__ == "__main__":
    Process(target=cam).start()
    Process(target=robot).start()
