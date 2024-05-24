import serial.tools.list_ports
import serial
import time
import threading
import keyboard
import camera
import cv2
import os
import torch
import ai
from PIL import Image
import sys

def prediction(cap):
    ret, frame = cap.read()
    # If frame is read correctly, ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        return -1

    # Display the resulting frame
    #cv2.imshow('Camera Output', frame)

    

    img_path = os.path.join("temp/", "temp.jpg")
    cv2.imwrite(img_path, frame)
    
    img = Image.open(img_path)
    #pred = color_vision.predict_running(frame,alpha=100)
    pred = color_vision.predict_running(img,alpha=100)  #0.3
    
    #i=0
    #i+=1

    # Handle key presses
    # cv2.waitKey() returns the ASCII value of the key pressed
    #print(pred)
    return pred

    

def act_on_pred(pred, drive_flag):
    if pred == 2:   # NONE
        return drive_flag
    elif pred == 1: # RED
        return False
    else:           # GREEN
        return True

def read_from_arduino():
    global vel, left, right, error
    while True:
        if arduino.in_waiting > 0:
            data = arduino.readline().decode('ascii')

            #vals = data.split('-')
            #vel = vals[0]
            #left = vals[1]
            #right = vals[2]
            #error = right-left
            #print(f"\r vel : {vals[0]:<10} left : {vals[1]:<10} right : {vals[2]:<10}", end = "")


def write_to_arduino(arduino,data):
    arduino.write(data.encode('ascii'))
    arduino.flush()

def run(drive_flag):
    drive_flag = act_on_pred(prediction(cap), drive_flag)
    if drive_flag:
        print("gogogogo")
        return drive_flag, "1\n"
        #write_to_arduino("2")
    else:
        print("dont gogogogo")
        return drive_flag, "0\n"
        #write_to_arduino("0")


drive_flag = False
last_drive_flag = False
if __name__ == '__main__':
    cam_num = 0
    for i in range(200):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print("\n")
            print(f"Camera index: {i}")
            cam_num = i
            cap.release()
            cv2.destroyAllWindows()
            print("\n")
            break

    color_vision = ai.ai("model_scripted2.pt")

    save_dir = "temp"
    os.makedirs(save_dir, exist_ok=True) 


    # Initialize the video capture object.

    cap = cv2.VideoCapture(cam_num)


    # Find COM port
    ports = serial.tools.list_ports.comports()
    port = ports[0].device if ports else None
    arduino = serial.Serial(port=port, baudrate=9600)
    print(f"Connected to: {port}")
    
    vel = 0
    right = 0
    left = 0
    
    
    #stop_thread = threading.Event()

    #read_thread = threading.Thread(target=read_from_arduino)
    #read_thread.start()


    #try:
    while True:
        time.sleep(0.1)
        drive_flag, inp = run(drive_flag)
        if drive_flag != last_drive_flag and drive_flag:
            write_to_arduino(arduino,inp)
            print("WROTE TO ARDUINO!!!!")
            drive_flag = False
        last_drive_flag = drive_flag

    #finally:
        #stop_thread.set()
        #read_thread.join()
