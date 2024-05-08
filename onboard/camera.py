import cv2
import os
import torch
import ai
from PIL import Image

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
    return pred

def init_camera():
    color_vision = ai.ai("model_scripted2.pt")

    save_dir = "temp"
    os.makedirs(save_dir, exist_ok=True) 

    file_name_prefix = "temp"
    suffix_index = 0

    # Initialize the video capture object.
    cap = cv2.VideoCapture(5)

    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    return cap


if __name__ == '__main__':
    color_vision = ai.ai("model_scripted2.pt")

    save_dir = "temp"
    os.makedirs(save_dir, exist_ok=True) 

    file_name_prefix = "temp"
    suffix_index = 0

    # Initialize the video capture object.
    cap = cv2.VideoCapture(5)

    # Check if the webcam is opened correctly
    if not cap.isOpened():
        raise IOError("Cannot open webcam")

    while True:
        pred = prediction()
        print(pred)


    # Release the VideoCapture object and close the windows
    cap.release()
    cv2.destroyAllWindows()
