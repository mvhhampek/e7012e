import cv2
import os
import torch
from computer_vision.ai import ai
from PIL import Image

color_vision = ai("computer_vision\\model_scripted2.pt")

save_dir = "test"
os.makedirs(save_dir, exist_ok=True) 

file_name_prefix = "test"
suffix_index = 0

# Initialize the video capture object.
cap = cv2.VideoCapture(1)

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

i=0
while True:
    ret, frame = cap.read()
    # If frame is read correctly, ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Display the resulting frame
    cv2.imshow('Camera Output', frame)

    
    filename = f'{file_name_prefix}{suffix_index}.jpg'
    img_path = os.path.join(save_dir, filename)
    cv2.imwrite(img_path, frame)
    
    img = Image.open("test\\test0.jpg")
    #pred = color_vision.predict_running(frame,alpha=100)
    pred = color_vision.predict_running(img,alpha=0.3)
    print(pred)
    #i=0
    #i+=1

    # Handle key presses
    # cv2.waitKey() returns the ASCII value of the key pressed
    k = cv2.waitKey(1)

    if k & 0xFF == ord('q'):
        # 'q' key pressed so quit
        break

# Release the VideoCapture object and close the windows
cap.release()
cv2.destroyAllWindows()