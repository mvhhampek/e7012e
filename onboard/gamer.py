import serial.tools.list_ports
import serial
import time
import threading
import keyboard
import camera

def act_on_pred(pred, drive_flag):
    if pred == 2:   # NONE
        return drive_flag
    elif pred == 1: # RED
        return False
    else:           # GREEN
        return True

def read_from_arduino():
    global vel, left, right
    while True:
        if arduino.in_waiting > 0:
            data = arduino.readline().decode('ascii')

            vals = data.split('-')
            vel = vals[0]
            left = vals[1]
            right = vals[2]
            #print(f"\r vel : {vals[0]:<10} left : {vals[1]:<10} right : {vals[2]:<10}", end = "")


def write_to_arduino(data):
    arduino.write(data.encode('ascii'))

def run():
    drive_flag = act_on_pred(prediction(cap, drive_flag))
    if drive_flag:
        pass
        # go 
    else:
        pass
        # dont go





if __name__ == '__main__':
    # Find COM port
    ports = serial.tools.list_ports.comports()
    port = ports[0].device if ports else None
    arduino = serial.Serial(port=port, baudrate=9600)
    print(f"Connected to: {port}")
    
    vel = 0
    right = 0
    left = 0
    
    drive_flag = False
    cap = init_camera()
    stop_thread = threading.Event()

    read_thread = threading.Thread(target=read_from_arduino)
    read_thread.start()


    try:
        while True:
            inp = "70-0"
            write_to_arduino(inp)
    finally:
        stop_thread.set()
        read_thread.join()