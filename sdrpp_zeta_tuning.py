import socket
import time
import serial
import serial.tools.list_ports

HOST = 'localhost'
PORT = 4532

ports = serial.tools.list_ports.comports()
device = ""

for info in ports:
    print(info.device, info.description, info.manufacturer, info.product, info.serial_number)
    if(info.product=="ZetaSDR"):
        device = info.device

if(device==""):
    print("ZetaSDR nincs csatlakoztatva!")
    raise SystemExit

try:
    SDR = serial.Serial(device)
except serial.SerialException:
    print("Nem lehetett megnyitni a portot:", device)
    raise SystemExit

rigctl_client = socket.socket()

try:
    rigctl_client.connect((HOST, PORT))
except ConnectionRefusedError:
    print('A Rigctl szerver nem elérhető.')
    SDR.close()
    raise SystemExit

prev_data = 0

while(True):
    try:
        rigctl_client.sendall(b"f\x0a")
        data = rigctl_client.recv(1024)
        if(data!=prev_data):
            SDR.write(str(int(data)).encode('ascii'))
            print(str(int(data)))

            out = bytearray()
            time.sleep(0.1)
            while SDR.inWaiting() > 0:
                out.extend(SDR.read())
            #print(out.decode())
            print(out)
            prev_data = data

    except Exception as error:
        print(error)
        break
    time.sleep(0.1)

SDR.close()
print('A Rigctl szervert leállították.')

