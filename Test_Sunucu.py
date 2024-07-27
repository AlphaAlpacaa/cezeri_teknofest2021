import requests
import time

giris_bilgi = {"kadi": "cezeri", "sifre": "8vud04mn09"}
s = requests.session()
giris = s.post('http://192.168.20.10:64559/api/giris/', json=giris_bilgi)
print("giriş",giris.status_code)

time.sleep(2)

telemetri_bilgileri = {
"takim_numarasi": 1,
"IHA_enlem": 433.5,
"IHA_boylam": 222.3,
"IHA_irtifa": 222.3,
"IHA_dikilme": 5,
"IHA_yonelme": 256,
"IHA_yatis": 0,
"IHA_hiz": 223,
"IHA_batarya": 20,
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
telemetri_gonder = s.post('http://192.168.20.10:64559/api/telemetri_gonder/', json=telemetri_bilgileri)
#telemetri_gonder.status_code
print(telemetri_gonder.json())

time.sleep(2)

kilitlenme_bilgileri = {
 "kilitlenmeBaslangicZamani": {
 "saat": 19,
 "dakika": 1,
 "saniye": 23,
 "milisaniye": 507
 },
 "kilitlenmeBitisZamani": {
 "saat": 19,
 "dakika": 1,
 "saniye": 45,
 "milisaniye": 236
 },
 "otonom_kilitlenme": 0
}
kilitlenme_bilgisi = s.post('http://192.168.20.10:64559/api/kilitlenme_bilgisi/', json=kilitlenme_bilgileri)
print(kilitlenme_bilgisi.status_code)

time.sleep(2)

cikis = s.get('http://192.168.20.10:64559/api/cikis/')
print("çıkış",cikis.status_code)