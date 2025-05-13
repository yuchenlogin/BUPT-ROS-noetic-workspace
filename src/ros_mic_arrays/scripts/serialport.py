import serial
# s = serial.Serial('/dev/ttyUSB0', baudrate=115200)
# while True:
#     res = s.readline()
#     if 'WAKE UP!' in res:
#         print(res)
#
#     # time.sleep(0.5)

with serial.Serial('/dev/ttyUSB0', baudrate=115200) as ser:
    while True:
        res = ser.readline()
        if 'WAKE UP!' in res:
            print(res)
