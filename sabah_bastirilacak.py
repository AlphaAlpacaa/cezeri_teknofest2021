import requests
import time
from dronekit import *

iha = connect("COM10", baud=57600, wait_ready=True, timeout=120)

for i in range(20):
    enlem = iha.location.global_relative_frame.lat
    boylam = iha.location.global_relative_frame.lon
    irtifa = iha.location.global_relative_frame.alt
    attitudes = iha.attitude  # dikilme yönelme yatış
    velocity = iha.airspeed  # hız
    batarya = iha.battery  # pil voltajı
    print("enlem:",enlem)
    print("boylam:", boylam)
    print("irtifa:", irtifa)
    print("dikilme:",attitudes.pitch)
    print("yönelme:", attitudes.yaw)
    print("yatış:", attitudes.roll)
    print("hız:", velocity)
    print("batarya:", batarya.voltage)
    print(i,"-------")

# Hesap: 100 - [((14.8 - voltage) * 100) / (14.8 - %0)]
# max: 16.8 V
# min: 8 V