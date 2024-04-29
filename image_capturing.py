import cv2
import os

save_dir = "images/none"
os.makedirs(save_dir, exist_ok=True) 

file_name_prefix = "none"
suffix_index = 0
# Initialize the video capture object.
cap = cv2.VideoCapture(1)

# Check if the webcam is opened correctly
if not cap.isOpened():
    raise IOError("Cannot open webcam")

while True:
    ret, frame = cap.read()
    # If frame is read correctly, ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Display the resulting frame
    cv2.imshow('Camera Output', frame)

    # Handle key presses
    # cv2.waitKey() returns the ASCII value of the key pressed
    k = cv2.waitKey(1)

    if k & 0xFF == ord('q'):
        # 'q' key pressed so quit
        break
    elif k & 0xFF == 32:  # ASCII value of the space bar is 32
        # Space bar pressed, so save an image
        filename = f'{file_name_prefix}{suffix_index}.jpg'
        img_path = os.path.join(save_dir, filename)
        cv2.imwrite(img_path, frame)
        suffix_index += 1
        print("Image saved!")

# Release the VideoCapture object and close the windows
cap.release()
cv2.destroyAllWindows()