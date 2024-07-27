import requests
import time
from dronekit import *

iha = connect("COM10", baud=57600, wait_ready=True, timeout=120)

giris_bilgi = {"kadi": "cezeri", "sifre": "8vud04mn09"}
s = requests.session()
giris = s.post('http://212.174.75.78:64559/api/giris/', json=giris_bilgi)
giris.status_code

time.sleep(2)

for i in range(20):
    enlem = iha.location.global_relative_frame.lat
    boylam = iha.location.global_relative_frame.lon
    irtifa = iha.location.global_relative_frame.alt
    attitudes = iha.attitude  # dikilme yönelme yatış
    velocity = iha.airspeed  # hız
    batarya = iha.battery  # pil voltajı
    telemetriler = {
"takim_numarasi": 1,
"IHA_enlem": enlem,
"IHA_boylam": boylam,
"IHA_irtifa": irtifa,
"IHA_dikilme": attitudes.pitch,
"IHA_yonelme": attitudes.yaw,
"IHA_yatis": attitudes.roll,
"IHA_hiz": velocity,
"IHA_batarya": batarya,
"IHA_otonom": 0,
"IHA_kilitlenme": 1,
"Hedef_merkez_X": 315,
"Hedef_merkez_Y": 220,
"Hedef_genislik": 12,
"Hedef_yukseklik": 46,
"GPSSaati": {
 "saat": 19,
 "dakika": 1,
 "saniye": 23,
 "milisaniye": 507
 }
}
    telemetri_gonder = s.post('http://212.174.75.78:64559/api/telemetri_gonder/', json=telemetriler)

cikis = s.get('http://212.174.75.78:64559/api/cikis/')
cikis.status_code