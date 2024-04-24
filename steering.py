import keyboard
import time
import threading



def adjust_steering_angle(delta):
    global desired_steering_angle
    desired_steering_angle += delta
    desired_steering_angle = max(min_angle, min(max_angle, desired_steering_angle))

def adjust_forward_speed(delta):
    global desired_forward_speed
    desired_forward_speed += delta
    desired_forward_speed = max(min_speed, min(max_speed, desired_forward_speed))

def print_current_values():
    while not stop_thread.is_set():
        print(f"\rSteering Angle: {desired_steering_angle} degrees, Forward Speed: {desired_forward_speed:.2f} m/s", end='')
        time.sleep(0.1)

def reset_angle():
    global desired_steering_angle
    desired_steering_angle = 70



if __name__ == '__main__':
    # start values
    desired_steering_angle = 70  
    desired_forward_speed = 3.0  

    # constraints
    min_angle = 45
    max_angle = 95
    min_speed = 1.5
    max_speed = 4

    print("Press ESC to exit.")
    #kanske behöver byta a och d
    keyboard.add_hotkey('a', adjust_steering_angle, args=(-1,))
    keyboard.add_hotkey('d', adjust_steering_angle, args=(1,))
    keyboard.add_hotkey('w', adjust_forward_speed, args=(0.1,))
    keyboard.add_hotkey('s', adjust_forward_speed, args=(-0.1,))
    keyboard.add_hotkey('r', reset_angle)

    stop_thread = threading.Event()

    print_thread = threading.Thread(target=print_current_values)
    print_thread.start()

    # exits if esc is pressed
    try:
        keyboard.wait('esc')
    finally:
        stop_thread.set()
        print_thread.join() 