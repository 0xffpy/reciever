import cv2
import time
import numpy as np
import os
import zmq
from pymavlink import mavutil

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=57600, blocking=True)


context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:5566")
time.sleep(1)

def mser_create(image, mser_object, length=350, ratio=4):
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_blur = cv2.GaussianBlur(image_gray, ksize=(9, 9), sigmaX=0)
    regions, _ = mser_object.detectRegions(image_blur)
    boxes = []

    for p in regions:
        x_max, y_max = np.amax(p, axis=0)
        x_min, y_min = np.amin(p, axis=0)
        if not (abs(x_max - x_min) > length or abs(y_min - y_max) > length):
            if not (abs(y_min - y_max) / abs(x_max - x_min) > ratio
                    or abs(x_max - x_min) / abs(y_min - y_max) > ratio):
                boxes.append((x_min, y_min, x_max, y_max))
    return np.array(boxes)

def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

def non_max_suppression_fast(boxes, overlapThresh=0.25):
    if len(boxes) == 0:
        return []
    if boxes.dtype.kind == "i":
        boxes = boxes.astype("float")
    pick = []
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]
    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y2)
    while len(idxs) > 0:
        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)
        xx1 = np.maximum(x1[i], x1[idxs[:last]])
        yy1 = np.maximum(y1[i], y1[idxs[:last]])
        xx2 = np.minimum(x2[i], x2[idxs[:last]])
        yy2 = np.minimum(y2[i], y2[idxs[:last]])
        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)
        overlap = (w * h) / area[idxs[:last]]
        idxs = np.delete(idxs, np.concatenate(([last],
                                               np.where(overlap > overlapThresh)[0])))
    return boxes[pick].astype("int")

def main():
    # view = False
    # cam = cv2.VideoCapture('/dev/video0')
    bar = 0


    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 0.5)
    mser_object = cv2.MSER_create(_max_area = 30000000, _min_area=10000)
    gps = b'None'
    # IMAGE_NAME = "capt0000.jpg"
    IMAGE_NAME = "capture_preview.jpg"

    while True:
        message = socket.recv()
        print("Received request: %s" % message)
        frame = cv2.imread(IMAGE_NAME)

        try:
            #ret, frame = cam.read()
            #frame = cv2.resize(frame,(1440,720))
            os.system('gphoto2 --trigger-capture')
            os.system('gphoto2 --capture-preview --force-overwrite')

            box = non_max_suppression_fast(mser_create(frame, mser_object))
            if  len(box) > 0:
                start = time.time()
                retval, buffer = cv2.imencode('.jpeg', frame)
                
                data = buffer.tobytes()
                
                temp = master.recv_match().to_dict()
                print(temp)
                if temp:
                    if temp['mavpackettype'] == 'GLOBAL_POSITION_INT':
                        lat = temp['lat']
                        lon = temp['lon']
                        alt = temp['alt']
                        gps = f"{lat}!{lon}!{alt}".encode()
                    else:
                        gps = b'None'
            
                if bar == 0:
                    print("sending data")
                    socket.send(data)
                    bar = 1
                elif bar == 1:
                    print("Sending ",gps)
                    socket.send(gps)
                    bar = 0
                    flag = False
                # if view: 
                #     fps = cam.get(cv2.CAP_PROP_FPS)
                #     print(len(frame))
                #     print("FPS = ",fps)
                #     cv2.imshow("window", frame)
            # if cv2.waitKey(1) == ord('q'):
            #     break
                print(time.time() - start)
            else:
                socket.send(b'None')
        except:
            socket.send(b'None')


if __name__ == '__main__':
    main()