import cv2 as cv
import numpy as np
#from timeit import default_timer as timer
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import math
import requests
import datetime



class Tracking():

    def __init__(self):

        global saat, dakika, saniye, milisaniye

        saat = 0
        dakika = 0
        saniye = 0
        milisaniye = 0

        self.iha = connect("COM12", baud=57600, wait_ready=True, timeout=100)

        """print(self.iha.mode)

        while not self.iha.armed:

            will_make_arm = int(input("Arm Edilsin mi?(1/2)"))

            if will_make_arm == 1:
                self.make_arm()

            else:
                print("Arm Icin Bekleniyor...")
                continue

        print("IHA Kalkisa Hazir!")

        target_height = int(input("Hedef Yukseklik:"))

        self.rise(target_height)

        print("IHA Kilitlenmeye Hazir!")

        a = input("Kilitlenme Baslasin Mi?(1/2)")
        while not a == 1:
            print("Komut Bekleniyor...")
            time.sleep(2)
            continue """

        kitlenme = 0
        telemetri_bilgi = 0
        yolotime = 0
        kitlenme_baslangic_saat = 0
        kitlenme_baslangic_dakika = 0
        kitlenme_baslangic_saniye = 0
        kitlenme_baslangic_milisaniye = 0

            ## Video Settings
        cap = cv.VideoCapture(0)
        self.whT = 416
        self.confThreshold = 0.5
        self.nmsThreshold = 0.2
        self.flag = True

        ## For FPS Counter
        self.prev_frame_time = 0
        self.new_frame_time = 0

        #### LOAD MODEL
        ## Coco Names
        self.classNames = ["drone"]

        ## Model Files
        modelConfiguration = "yolov3_custom.cfg"
        modelWeights = "yolov3_custom_final.weights"

        net = cv.dnn.readNetFromDarknet(modelConfiguration, modelWeights)
        net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)

        ## Tracker
        self.tracker = cv.TrackerCSRT_create()

        # Time Start
        self.start_duration = time.time()

        self.A_x = [62.5, 187, 312.5, 437.5, 62.5, 187, 312.5, 437.5, 62.5, 187, 312.5, 437.5]
        self.A_y = [58.5, 58.5, 58.5, 58.5, 175.5, 175.5, 175.5, 175.5, 292, 292, 292, 292]

        ## Video Writer
        out = cv.VideoWriter('outpy.avi', cv.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10,
                             (1280, 720))

        ## Telemetri
        """giris_bilgi = {"kadi": "cezeri", "sifre": "8vud04mn09"}
        s = requests.session()
        giris = s.post('http://192.168.20.10:64559/api/giris/', json=giris_bilgi)
        print(giris.status_code)"""

        #### Main Loop
        while True:
            success, img = cap.read()

            cu = self.iha

            # img = cv.flip(img, 1)

            img = cv.resize(img, (1280, 720), interpolation=cv.INTER_AREA)

            #print(self.iha.mode)

            ## FPS Counter
            self.new_frame_time = time.time()
            fps = 1 / (self.new_frame_time - self.prev_frame_time)
            self.prev_frame_time = self.new_frame_time


            #cv.putText(img, f"Fps: {round(fps, 2)}", (1100,50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1)

            # Kitlenmeden önce: 3.5 fps
            # Kitlenmede: 8.5 fps

            hT, wT, cT = img.shape

            self.center = (640, 360)

            ## Telemetri
            enlem = self.iha.location.global_relative_frame.lat
            boylam = self.iha.location.global_relative_frame.lon
            irtifa = self.iha.location.global_relative_frame.alt
            attitudes = self.iha.attitude  # dikilme yönelme yatış
            velocity = self.iha.airspeed  # hız
            batarya_voltaj = int(self.iha.battery.voltage) #self.iha.battery["voltage"]  # pil voltajı
            batarya = 100-((16.8-batarya_voltaj)*100/8)

            """@cu.on_message('SYSTEM_TIME')
            def listener(self, name, message):
                unix_time = (int)(message.time_unix_usec / 1000000)
                saat = datetime.datetime.fromtimestamp(unix_time).hour - 3
                dakika = datetime.datetime.fromtimestamp(unix_time).minute
                saniye = datetime.datetime.fromtimestamp(unix_time).second
                milisaniye = datetime.datetime.fromtimestamp(unix_time).microsecond / 1000
                return saat,dakika,saniye,milisaniye

            #print(listener)"""

            if self.iha.mode == 'GUIDED':
                otonom = 1
            else:
                otonom = 0

            ## Locking Rectangle
            space_y = int(hT / 10)
            space_x = int(wT / 4)

            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            gauss = cv.GaussianBlur(gray, (3, 3), 0)
            # gauss = cv.adaptiveThreshold(gauss,255,cv.ADAPTIVE_THRESH_MEAN_C,cv.THRESH_BINARY,17,7)

            if self.flag:
                print("Finding")

                ## YOLO Finding
                blob = cv.dnn.blobFromImage(img, 1 / 255, (self.whT, self.whT), [0, 0, 0], 1, crop=False)
                net.setInput(blob)
                layersNames = net.getLayerNames()
                outputNames = [(layersNames[i[0] - 1]) for i in net.getUnconnectedOutLayers()]
                outputs = net.forward(outputNames)

                ## Target Coords
                coord = self.findObjects(outputs, img, hT, wT, cT, space_y, space_x)
                print(coord)

                iha_kitlenme = 0

                if coord == None:
                    self.stop_duration = time.time()

                    now = self.stop_duration - self.start_duration
                    #print(now)
                    if now%10 <= 0.2:
                        pass

                else:

                    #if self.iha.mode == 'GUIDED':
                        ## Move
                        #self.move(coord[0] + coord[2] / 2, coord[1] + coord[3] / 2, coord[2], coord[3])

                    ## Passing CSRT
                    ok = self.tracker.init(img, (coord[0], coord[1], coord[2], coord[3]))
                    print(ok)

                    yolotime = time.time()

                    kitlenme_baslangic_saat = telemetri_bilgi["sistemSaati"]["saat"]
                    kitlenme_baslangic_dakika = telemetri_bilgi["sistemSaati"]["dakika"]
                    kitlenme_baslangic_saniye = telemetri_bilgi["sistemSaati"]["saniye"]
                    kitlenme_baslangic_milisaniye = telemetri_bilgi["sistemSaati"]["milisaniye"]

                    iha_kitlenme = 1

                    self.flag = False

            else:

                print("Tracking")

                trackingtime = time.time()
                trackedtime = trackingtime - yolotime

                ## Tracking Updates
                ok, coord = self.tracker.update(gauss)
                print(coord)

                if ok:

                    ## Target Points
                    p1 = (int(coord[0]), int(coord[1]))
                    p2 = (int(coord[0] + coord[2]),
                          int(coord[1] + coord[3]))
                    cv.rectangle(img, p1, p2, (0, 0, 255), 2, 2)

                    center_iha = (int(coord[0]+coord[2]/2),int(coord[1]+coord[3]/2))

                    ## Inside/Outside Control
                    if  320  <= center_iha[0] <= 960 and 72 <= center_iha[1] <= 648:
                        #320 + (coord[2]/2) <= center_iha[0] <= 960- (coord[2]/2) and 72 + (coord[3]/2) <= center_iha[1] <= 648 - (coord[3]/2):

                        ## Printing Target Detected
                        cv.putText(img, "Target Detected!", (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                        if trackedtime <= 10:
                            cv.putText(img, f"Tracked Time: {round(trackedtime, 2)}", (25, 635),
                                       cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

                        if trackedtime >= 10:
                            cv.putText(img, "Kitlenme Basarili", (25, 550), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                            kitlenme_bitis = telemetri_bilgi["sistemSaati"]

                            kitlenme_bitis_saat = telemetri_bilgi["sistemSaati"]["saat"]
                            kitlenme_bitis_dakika = telemetri_bilgi["sistemSaati"]["dakika"]
                            kitlenme_bitis_saniye = telemetri_bilgi["sistemSaati"]["saniye"]
                            kitlenme_bitis_milisaniye = telemetri_bilgi["sistemSaati"]["milisaniye"]

                            if self.iha.mode == 'GUIDED':
                                kitlenme += 1

                            kilitlenme_bilgileri = {
                                "kilitlenmeBaslangicZamani": {
                                    "saat": kitlenme_baslangic_saat,
                                    "dakika": kitlenme_baslangic_dakika,
                                    "saniye": kitlenme_baslangic_saniye,
                                    "milisaniye": kitlenme_baslangic_milisaniye
                                },
                                "kilitlenmeBitisZamani": {
                                    "saat": kitlenme_bitis_saat,
                                    "dakika": kitlenme_bitis_dakika,
                                    "saniye": kitlenme_bitis_saniye,
                                    "milisaniye": kitlenme_bitis_milisaniye
                                },
                                "otonom_kilitlenme": kitlenme
                            }

                            print(kilitlenme_bilgileri)

                            #kilitlenme_bilgisi = s.post('http://192.168.20.10:64559/api/kilitlenme_bilgisi/',
                            #                           json=kilitlenme_bilgileri)
                            #print(kilitlenme_bilgisi.status_code)


                            self.flag = True


                    else:

                        yolotime = time.time()

                    #if self.iha.mode == 'GUIDED':
                        ## Move
                        #self.move(coord[0] + coord[2] / 2, coord[1] + coord[3] / 2, coord[2], coord[3])

                    # target_center = ((coord[0] + int(coord[2]/2)),(coord[1]+int(coord[3]/2)))

                    ## '+' To Center of Target
                    # cv.putText(img, "+",targert_center, cv.FONT_HERSHEY_SIMPLEX, 1,(255, 0, 255), 3)

                    ## Tracking ON/OFF
                    if cv.waitKey(24) & 0xFF == ord("q"):

                        self.flag = True

                else:
                    self.flag = True

            ## FPS Printer
            # cv.putText(img, "FPS : " + str(fps), (100, 50), cv.FONT_HERSHEY_SIMPLEX, 0.75, (50, 170, 50), 2);

            if iha_kitlenme == 1:
                hedef_merkez_x = int(coord[0]+(coord[2]/2))
                hedef_merkez_y = int(coord[1]+(coord[3]/2))
                hedef_genislik = coord[2]
                hedef_yukseklik = coord[3]

            else:
                hedef_merkez_x = 0
                hedef_merkez_y = 0
                hedef_genislik = 0
                hedef_yukseklik = 0

            cv.rectangle(img, (space_x, space_y), (wT - space_x, hT - space_y), (255, 0, 255), 3)
            cv.putText(img, "+", (self.center[0], self.center[1]), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 1)

            out.write(img)
            cv.imshow('Image', img)
            # cv.imshow('Gauss', gauss)

            @self.iha.on_message('SYSTEM_TIME')
            def listener(self, name, message):
                global saat, dakika, saniye, milisaniye
                unix_time = (int)(message.time_unix_usec / 1000000)
                saat = datetime.datetime.fromtimestamp(unix_time).hour - 3
                dakika = datetime.datetime.fromtimestamp(unix_time).minute
                saniye = datetime.datetime.fromtimestamp(unix_time).second
                milisaniye = datetime.datetime.fromtimestamp(unix_time).microsecond / 1000

            listener

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
                "IHA_otonom": otonom,
                "IHA_kilitlenme": iha_kitlenme,
                "Hedef_merkez_X": hedef_merkez_x,
                "Hedef_merkez_Y": hedef_merkez_y,
                "Hedef_genislik": hedef_genislik,
                "Hedef_yukseklik": hedef_yukseklik,
                "GPSSaati": {
                    "saat": saat,
                    "dakika": dakika,
                    "saniye": saniye,
                    "milisaniye": milisaniye
                }
            }

            print(telemetriler)

            telemetri_bilgi = {
             "sistemSaati": {
             "saat": 6,
             "dakika": 53,
             "saniye": 42,
             "milisaniye": 500
             },
             "konumBilgileri": [

            {
             "takim_numarasi": 1,
             "iha_enlem": 500,
             "iha_boylam": 500,
             "iha_irtifa": 500,
             "iha_dikilme": 5,
             "iha_yonelme": 256,
             "iha_yatis": 0,
             "zaman_farki": 93
             },

            {
             "takim_numarasi": 2,
             "iha_enlem": 500,
             "iha_boylam": 500,
             "iha_irtifa": 500,
             "iha_dikilme": 5,
             "iha_yonelme": 256,
             "iha_yatis": 0,
             "zaman_farki": 74
             },

            {
             "takim_numarasi": 3,
             "iha_enlem": 433.5,
             "iha_boylam": 222.3,
             "iha_irtifa": 222.3,
             "iha_dikilme": 5,
             "iha_yonelme": 256,
             "iha_yatis": 0,
             "zaman_farki": 43

            }

            ]
            }
            #telemetri_gonder = s.post('http://192.168.20.10:64559/api/telemetri_gonder/', json=telemetriler)
            # telemetri_gonder.status_code
            #telemetri_bilgi = telemetri_gonder.json()
            #print(1,telemetri_bilgi)
            #time.sleep(0.5)

            k = cv.waitKey(24) & 0xff
            if k == 27:
                break

        #cikis = s.get('http://192.168.20.10:64559/api/cikis/')
        #print("çıkış", cikis.status_code)

        cap.release()
        cv.destroyAllWindows()

    def detected(self):
        pass

    def findObjects(self, outputs, img, hT, wT, cT, area_y, area_x):
        bbox = []
        classIds = []
        confs = []

        for output in outputs:
            for det in output:
                scores = det[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.confThreshold:
                    w, h = int(det[2] * wT), int(det[3] * hT)
                    x, y = int((det[0] * wT) - w / 2), int((det[1] * hT) - h / 2)
                    bbox.append([x, y, w, h])
                    classIds.append(classId)
                    confs.append(float(confidence))

        indices = cv.dnn.NMSBoxes(bbox, confs, self.confThreshold, self.nmsThreshold)

        for i in indices:
            i = i[0]
            box = bbox[i]
            x, y, w, h = box[0], box[1], box[2], box[3]

            cv.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)

            if area_x + w / 2 <= x + w / 2 <= wT - area_x - w / 2 and area_y + h / 2 <= y + h / 2 <= hT - area_y - h / 2:
                cv.putText(img, "Target Detected!", (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        try:
            return (x, y, w, h)
        except UnboundLocalError:
            pass

    def make_arm(self):

        while self.iha.is_armable == False:
            print("Arm ici gerekli sartlar saglanamadi.")
            time.sleep(1)
        print("Iha su anda armedilebilir")

        self.iha.mode = VehicleMode("GUIDED")
        while self.iha.mode != 'GUIDED':
            print('Guided moduna gecis yapiliyor')
            time.sleep(1.5)

        print("Guided moduna gecis yapildi")

        self.iha.armed = True
        while self.iha.armed is False:
            print("Arm icin bekleniliyor")
            time.sleep(1)

        print("Ihamiz arm olmustur")

    def rise(self, target_height):

        self.iha.simple_takeoff(target_height)

        while self.iha.location.global_relative_frame.alt <= target_height * 0.94:
            print("Su anki yukseklik{}".format(self.iha.location.global_relative_frame.alt))
            time.sleep(0.5)
        print("Takeoff gerceklesti")

    def velocity(self, velocity_x, velocity_y, velocity_z, yaw_rate):
        msg = self.iha.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000011111000111,
            0, 0, 0,
            velocity_x, velocity_y, velocity_z,
            0, 0, 0,
            0, math.radians(yaw_rate))

        self.iha.send_mavlink(msg)

    def move(self, x, y, w, h):

        area_rival_drone = w * h

        ## 1st Area
        if 0 <= x <= 426.66 and 0 <= y <= 240:
            print("1st Area")

            if 0 < area_rival_drone < (144 * 256):

                self.velocity(-4, 4, 3, -2)

            elif (144 * 256) < area_rival_drone < (288 * 512):

                self.velocity(-3, 3, 3, -2)

            elif (288 * 512) < area_rival_drone < (432 * 768):

                self.velocity(-3, 3, 2, -2)

            elif (432 * 768) < area_rival_drone < (576 * 1024):
                self.velocity(-2, 2, 2, -2)

            elif (576 * 1024) < area_rival_drone < (720 * 1280):
                self.velocity(-1, 0, 1, -2)
        ## 2nd Area
        elif 426.66 <= x <= 853.33 and 0 <= y <= 240:
            print("2nd Area")

            if 0 < area_rival_drone < (144 * 256):

                self.velocity(0, 4, 3, 0)

            elif (144 * 256) < area_rival_drone < (288 * 512):

                self.velocity(0, 4, 3, 0)

            elif (288 * 512) < area_rival_drone < (432 * 768):

                self.velocity(0, 3, 3, 0)

            elif (432 * 768) < area_rival_drone < (576 * 1024):
                self.velocity(0, 1, 2, 0)

            elif (576 * 1024) < area_rival_drone < (720 * 1280):
                self.velocity(0, 1, 1, 0)

        ## 3rd Area
        elif 853.33 <= x <= 1280 and 0 <= y <= 240:
            print("3rd Area")
            if 0 < area_rival_drone < (144 * 256):

                self.velocity(5, 4, 3, 2)

            elif (144 * 256) < area_rival_drone < (288 * 512):

                self.velocity(4, 4, 3, 2)

            elif (288 * 512) < area_rival_drone < (432 * 768):

                self.velocity(3, 3, 3, 2)

            elif (432 * 768) < area_rival_drone < (576 * 1024):
                self.velocity(2, 3, 2, 2)

            elif (576 * 1024) < area_rival_drone < (720 * 1280):
                self.velocity(1, 0, 1, 2)

        ## 4th Area
        elif 0 <= x <= 426.66 and 240 <= y <= 480:
            print("4th Area")
            if 0 < area_rival_drone < (144 * 256):

                self.velocity(-5, 4, 0, -2)

            elif (144 * 256) < area_rival_drone < (288 * 512):

                self.velocity(-4, 4, 0, -2)

            elif (288 * 512) < area_rival_drone < (432 * 768):

                self.velocity(-3, 3, 0, -2)

            elif (432 * 768) < area_rival_drone < (576 * 1024):
                self.velocity(-2, 2, 0, -2)

            elif (576 * 1024) < area_rival_drone < (720 * 1280):
                self.velocity(-1, 0, 0, -2)

        ## 5th Area
        elif 426.66 <= x <= 853.33 and 240 <= y <= 480:
            print("5th Area")
            if 0 < area_rival_drone < (144 * 256):

                self.velocity(0, 5, 0, 0)

            elif (144 * 256) < area_rival_drone < (288 * 512):

                self.velocity(0, 4, 0, 0)

            elif (288 * 512) < area_rival_drone < (432 * 768):

                self.velocity(0, 3, 0, 0)

            elif (432 * 768) < area_rival_drone < (576 * 1024):
                self.velocity(0, 2, 0, 0)

            elif (576 * 1024) < area_rival_drone < (720 * 1280):
                self.velocity(0, 0, 0, 0)

        ## 6th Area
        elif 853.33 <= x <= 1280 and 240 <= y <= 480:
            print("6th Area")
            if 0 < area_rival_drone < (144 * 256):

                self.velocity(5, 5, 0, 2)

            elif (144 * 256) < area_rival_drone < (288 * 512):

                self.velocity(4, 4, 0, 2)

            elif (288 * 512) < area_rival_drone < (432 * 768):

                self.velocity(3, 3, 0, 2)

            elif (432 * 768) < area_rival_drone < (576 * 1024):
                self.velocity(2, 2, 0, 2)

            elif (576 * 1024) < area_rival_drone < (720 * 1280):
                self.velocity(1, 0, 0, 2)

        ## 7th Area
        elif 0 <= x <= 426.66 and 480 <= y <= 720:
            print("7th Area")
            if 0 < area_rival_drone < (144 * 256):

                self.velocity(-5, 5, -3, -2)

            elif (144 * 256) < area_rival_drone < (288 * 512):

                self.velocity(-4, 4, -3, -2)

            elif (288 * 512) < area_rival_drone < (432 * 768):

                self.velocity(-3, 3, -3, -2)

            elif (432 * 768) < area_rival_drone < (576 * 1024):
                self.velocity(-2, 2, -2, -2)

            elif (576 * 1024) < area_rival_drone < (720 * 1280):
                self.velocity(-1, 0, -1, -2)

        ## 8th Area
        elif 426.66 <= x <= 853.33 and 480 <= y <= 720:
            print("8th Area")
            if 0 < area_rival_drone < (144 * 256):

                self.velocity(0, 5, -3, 0)

            elif (144 * 256) < area_rival_drone < (288 * 512):

                self.velocity(0, 4, -3, 0)

            elif (288 * 512) < area_rival_drone < (432 * 768):

                self.velocity(0, 3, -3, 0)

            elif (432 * 768) < area_rival_drone < (576 * 1024):
                self.velocity(0, 2, -2, 0)

            elif (576 * 1024) < area_rival_drone < (720 * 1280):
                self.velocity(0, 0, -1, 0)

        ## 9th Area
        elif 853.33 <= x <= 1280 and 480 <= y <= 720:
            print("9th Area")
            if 0 < area_rival_drone < (144 * 256):

                self.velocity(5, 5, -3, 2)

            elif (144 * 256) < area_rival_drone < (288 * 512):

                self.velocity(4, 4, -3, 2)

            elif (288 * 512) < area_rival_drone < (432 * 768):

                self.velocity(3, 3, -3, 2)

            elif (432 * 768) < area_rival_drone < (576 * 1024):
                self.velocity(2, 2, -2, 2)

            elif (576 * 1024) < area_rival_drone < (720 * 1280):
                self.velocity(1, 0, -1, 2)

tracking = Tracking()