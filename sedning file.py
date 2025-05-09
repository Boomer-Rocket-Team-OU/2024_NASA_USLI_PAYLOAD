import argparse
import socket
import sys

import ax25
import kiss

import serial

ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
print("Listening on /dev/ttyS0...")

'''
while True:
    line = ser.readline().decode('utf-8', errors='replace').strip()
    if line:
        print(line)
        send1(line)
'''

_PID_TEXT = 0xF0

def create_ui_frame(call_from, call_to, message):
    return ax25.Frame(
        call_to,
        call_from,
        control=ax25.Control(ax25.FrameType.UI),
        pid=_PID_TEXT,
        data=message.encode('utf-8')
    )

def send_unproto(host, port, frame):
    connection = kiss.Connection(None)
    error = None
    try:
        connection.connect_to_server(host, int(port))
    except socket.gaierror as e:
        if e.errno == socket.EAI_NONAME:
            error = 'Server not found'
        else:
            error = 'Invalid server address'
    except ConnectionRefusedError:
        error = 'Connection refused by server'
    except Exception:
        error = 'Unknown error connecting to server'
    else:
        connection.send_data(frame.pack())
        connection.disconnect_from_server()
    
    return error

'''
while True:
    line = ser.readline().decode('utf-8', errors='replace').strip()
    if line:
        print(line)
        frame = create_ui_frame("KF0STO", "KF0STO", line)
        send_unproto("localhost", 8001, frame)
'''

def send1(line):
    frame = create_ui_frame("KF0STO", "KF0STO", line)
    send_unproto("localhost", 8001, frame)

while True:
    line = ser.readline().decode('utf-8', errors='replace').strip()
    if line:
        print(line)
        send1(line)
