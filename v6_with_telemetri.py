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


        self.iha = connect("COM7", baud=57600, wait_ready=True, timeout=100)

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
        giris_bilgi = {"kadi": "cezeri", "sifre": "8vud04mn09"}
        s = requests.session()
        giris = s.post('http://212.174.75.78:64559/api/giris/', json=giris_bilgi)
        print(giris.status_code)

        #### Main Loop
        while True:
            success, img = cap.read()

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
            batarya = self.iha.battery  # pil voltajı

            @self.iha.on_message('SYSTEM_TIME')
            def listener(self, name, message):
                global saat, dakika, saniye, milisaniye
                unix_time = (int)(message.time_unix_usec / 1000000)
                saat = datetime.datetime.fromtimestamp(unix_time).hour - 3
                dakika = datetime.datetime.fromtimestamp(unix_time).minute
                saniye = datetime.datetime.fromtimestamp(unix_time).second
                milisaniye = datetime.datetime.fromtimestamp(unix_time).microsecond / 1000

            listener()

            print(saat,dakika,saniye,milisaniye)
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

                yolotime = time.time()

                if coord == None:
                    self.stop_duration = time.time()

                    now = self.stop_duration - self.start_duration
                    #print(now)
                    if now%10 <= 0.2:
                        pass
                        #self.BDSM()

                else:

                    ### Calculating time
                    # start = timer()

                    #if self.iha.mode == 'GUIDED':
                        ## Move
                        #self.move(coord[0] + coord[2] / 2, coord[1] + coord[3] / 2, coord[2], coord[3])

                    ## Passing CSRT
                    ok = self.tracker.init(img, (coord[0], coord[1], coord[2], coord[3]))
                    print(ok)

                    iha_kitlenme = 1

                    ## '+' To Center of Target
                    # cv.putText(img, "+", (coord[0] + coord[2], coord[1] + coord[3]), cv.FONT_HERSHEY_SIMPLEX, 1,(255, 0, 255), 3)

                    self.flag = False

            else:

                print("Tracking")

                kitlenme_baslangic =  telemetri_bilgi["sistemSaati"]

                # print(float(trackedtime))

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
                        # 320  <= center_iha[0] <= 960 and 72 <= center_iha[1] <= 648:

                        ## Printing Target Detected
                        cv.putText(img, "Target Detected!", (10, 40), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                        trackingtime = time.time()
                        trackedtime = trackingtime - yolotime

                        if trackedtime <= 10:
                            cv.putText(img, f"Tracked Time: {round(trackedtime, 2)}", (25, 635),
                                       cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

                        if trackedtime >= 10:
                            cv.putText(img, "Kitlenme Basarili", (25, 550), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
                            kitlenme_bitis = telemetri_bilgi["sistemSaati"]
                            kitlenme += 1
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
                hadef_merkez_x = 0
                hedef_merkez_y = 0
                hedef_genislik = 0
                hedef_yukseklik = 0

            cv.rectangle(img, (space_x, space_y), (wT - space_x, hT - space_y), (255, 0, 255), 3)
            cv.putText(img, "+", (self.center[0], self.center[1]), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 1)

            out.write(img)
            #cv.imshow('Image', img)
            # cv.imshow('Gauss', gauss)

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
                    "saat": 19,
                    "dakika": 1,
                    "saniye": 23,
                    "milisaniye": 507
                }
            }

            telemetri_gonder = s.post('http://192.168.20.10:64559/api/telemetri_gonder/', json=telemetriler)
            # telemetri_gonder.status_code
            telemetri_bilgi = telemetri_gonder.json()

            kilitlenme_bilgileri = {
                "kilitlenmeBaslangicZamani": {
                    "saat": kitlenme_baslangic["saat"],
                    "dakika": kitlenme_baslangic["dakika"],
                    "saniye": kitlenme_baslangic["saniye"],
                    "milisaniye": kitlenme_baslangic["milisaniye"]
                },
                "kilitlenmeBitisZamani": {
                    "saat": kitlenme_bitis["saat"],
                    "dakika": kitlenme_bitis["dakika"],
                    "saniye": kitlenme_bitis["saniye"],
                    "milisaniye": kitlenme_bitis["milisaniye"]
                },
                "otonom_kilitlenme": kitlenme
            }

            kilitlenme_bilgisi = s.post('http://192.168.20.10:64559/api/kilitlenme_bilgisi/', json=kilitlenme_bilgileri)
            print(kilitlenme_bilgisi.status_code)

            k = cv.waitKey(24) & 0xff
            if k == 27:
                break

        cikis = s.get('http://192.168.20.10:64559/api/cikis/')
        print("çıkış", cikis.status_code)

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

    def BDSM(self):

        A_x = [62.5, 187, 312.5, 437.5, 62.5, 187, 312.5, 437.5, 62.5, 187, 312.5, 437.5]
        A_y = [58.5, 58.5, 58.5, 58.5, 175.5, 175.5, 175.5, 175.5, 292, 292, 292, 292]

        '''Bölgelerdeki araç sayısı'''
        A = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        '''Ağırlıklı Katsayılar'''
        a_kat = 4
        b_kat = 6

        '''Aracımızın verileri'''
        a_x = 100  # x ekseninde konumumuz
        a_y = 100  # y ekseninde konumumuz
        a_hiz = 20  # hızımız

        '''Rakip verileri'''
        b_x = [430, 30, 120, 460, 250, 200, 50, 100, 100, 150]
        b_y = [250, 200, 20, 150, 100, 100, 200, 150, 100, 100]
        b_yon = [270, 90, 270, 180, 0, 180, 90, 90, 180, 180]

        '''Toplam Rakip Sayısı'''
        total_competitor = 10

        '''Her bölgeye ulaşma sürelerimiz'''
        time = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        def distance(a_x, a_y, A_x, A_y):
            return (((a_x - A_x) * 2 + (a_y - A_y) * 2) ** (0.5))

        def weighted_region_coefficient(n, a_x, a_y, A_x, A_y):
            return ((a_kat * (100 / distance(a_x, a_y, A_x, A_y))) + (b_kat * n)) / (a_kat + b_kat)

        ################### EN SON ALANLARDAKİ ARAÇ SAYILARINI SIFIRLAMA EKLEMEYİ UNUTMAAAAAA #############
        def optimum_degerli_bolge_bulma(a_hiz, A, a_x, a_y, A_x, A_y, total_competitor):
            for counter in range(12):
                time[counter] = distance(a_x, a_y, A_x[counter],
                                         A_y[counter]) / a_hiz + 2  # (counter ıncı bölgeye)o bölgeye ulaşma hızımız
                print(counter, "---", time[counter])
                for temporary_counter in range(total_competitor):
                    b_x[temporary_counter] += a_hiz * time[counter] * np.cos(b_yon[
                                                                                 temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                    b_y[temporary_counter] += a_hiz * time[counter] * np.sin(
                        b_yon[temporary_counter] / 180 * 3.141592653589793)
                    if counter == 0:  ##################################################################################
                        if b_y[temporary_counter] < 117:  # 1.satır
                            if b_x[temporary_counter] <= 125:  # 1.sütun
                                A[counter] += 1
                    elif counter == 1:
                        if b_y[temporary_counter] < 117:
                            if b_x[temporary_counter] >= 125 and b_x[temporary_counter] <= 250:  # 2.sütun
                                A[counter] += 1
                    elif counter == 2:
                        if b_y[temporary_counter] < 117:
                            if b_x[temporary_counter] >= 250 and b_x[temporary_counter] <= 375:  # 3.sütun
                                A[counter] += 1
                    elif counter == 3:
                        if b_y[temporary_counter] < 117:
                            if b_x[temporary_counter] >= 375:  # 4.sütun
                                A[counter] += 1
                    elif counter == 4:  ##################################################################################
                        if b_y[temporary_counter] < 234 and b_y[temporary_counter] >= 117:  # 2.satır
                            if b_x[temporary_counter] <= 125:
                                A[counter] += 1
                    elif counter == 5:
                        if b_y[temporary_counter] < 234 and b_y[temporary_counter] >= 117:
                            if b_x[temporary_counter] >= 125 and b_x[temporary_counter] <= 250:
                                A[counter] += 1
                    elif counter == 6:
                        if b_y[temporary_counter] < 234 and b_y[temporary_counter] >= 117:
                            if b_x[temporary_counter] >= 250 and b_x[temporary_counter] <= 375:
                                A[counter] += 1
                    elif counter == 7:
                        if b_y[temporary_counter] < 234 and b_y[temporary_counter] >= 117:
                            if b_x[temporary_counter] >= 375:
                                A[counter] += 1
                    elif counter == 8:  ################################################################################
                        if b_y[temporary_counter] >= 234:  # 3.satır
                            if b_x[temporary_counter] <= 125:
                                A[counter] += 1
                    elif counter == 9:
                        if b_y[temporary_counter] >= 234:
                            if b_x[temporary_counter] >= 125 and b_x[temporary_counter] <= 250:
                                A[counter] += 1
                    elif counter == 10:
                        if b_y[temporary_counter] >= 234:
                            if b_x[temporary_counter] >= 250 and b_x[temporary_counter] <= 375:
                                A[counter] += 1
                    else:
                        if b_y[temporary_counter] >= 234:
                            if b_x[temporary_counter] >= 375:
                                A[counter] += 1

        optimum_degerli_bolge_bulma(a_hiz, A, a_x, a_y, A_x, A_y, total_competitor)

        for counter in range(12):
            print("A", counter + 1, "ın değerleri:", A[counter])

        temporary_index = 0
        temporary = 0
        for i in range(12):
            print(weighted_region_coefficient(A[i], a_x, a_y, A_x[i], A_y[i]))
            if weighted_region_coefficient(A[i], a_x, a_y, A_x[i], A_y[i]).real >= temporary:
                temporary = weighted_region_coefficient(A[i], a_x, a_y, A_x[i], A_y[i])
                temporary_index = i
        print(temporary)
        temporary_number = A[0]
        for i in range(12):
            if A[i] >= temporary_number:
                temporary_number = A[i]
        number = temporary_number
        index = temporary_index + 1
        print("kaçıncı alan:", index)
        print("alandaki araç sayısı:", number)

        arama_sayaci = 0

        if index == 1:
            if a_y < 117:  # 1.satır
                if a_x <= 125:  # 1.sütun
                    arama_sayaci += 1
        elif index == 2:
            if a_y < 117:
                if a_x >= 125 and a_x <= 250:  # 2.sütun
                    arama_sayaci += 1
        elif index == 3:
            if a_y < 117:
                if a_x >= 250 and a_x <= 375:  # 3.sütun
                    arama_sayaci += 1
        elif index == 4:
            if a_y < 117:
                if a_x >= 375:  # 4.sütun
                    arama_sayaci += 1
        elif index == 5:
            if a_y < 234 and a_y >= 117:  # 2.satır
                if a_x <= 125:
                    arama_sayaci += 1
        elif index == 6:
            if a_y < 234 and a_y >= 117:
                if a_x >= 125 and a_x <= 250:
                    arama_sayaci += 1
        elif index == 7:
            if a_y < 234 and a_y >= 117:
                if a_x >= 250 and a_x <= 375:
                    arama_sayaci += 1
        elif index == 8:
            if a_y < 234 and a_y >= 117:
                if a_x >= 375:
                    arama_sayaci += 1
        elif index == 9:
            if a_y >= 234:  # 3.satır
                if a_x <= 125:
                    arama_sayaci += 1
        elif index == 10:
            if a_y >= 234:
                if a_x >= 125 and a_x <= 250:
                    arama_sayaci += 1
        elif index == 11:
            if a_y >= 234:
                if a_x >= 250 and a_x <= 375:
                    arama_sayaci += 1
        elif index == 12:
            if a_y >= 234:
                if b_x >= 375:
                    arama_sayaci += 1

        if arama_sayaci == 1:
            time_arama = 4
            b = [0, 0, 0, 0]

            if index == 1:  # kaçıncı büyük bölge olduğu
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] <= 58.5:  # 1.satır
                                if b_x[temporary_counter] <= 62.5:  # 1.sütun
                                    print("a1b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] <= 58.5:  # 1.satır
                                if b_x[temporary_counter] > 62.5 and b_x[temporary_counter] <= 125:  # 2.sütun
                                    print("a1b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 58.5 and b_y[temporary_counter] <= 117:  # 2.satır
                                if b_x[temporary_counter] <= 62.5:  # 1.sütun
                                    print("a1b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 58.5 and b_y[temporary_counter] <= 117:  # 2.satır
                                if b_x[temporary_counter] > 62.5 and b_x[temporary_counter] <= 125:  # 2.sütun
                                    print("a1b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("31,25-29,25")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("93,75-29,25")
                elif index_b == 2:
                    return "3. Bölge"
                    print("31,25-87,75")
                else:
                    print("4.alana gidilecek")
                    print("93,75-87,75")
                b = [0, 0, 0, 0]
            elif index == 2:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] <= 58.5:  # 1.satır
                                if b_x[temporary_counter] > 125 and b_x[temporary_counter] <= 187:  # 1.sütun
                                    print("a2b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] <= 58.5:  # 1.satır
                                if b_x[temporary_counter] > 187 and b_x[temporary_counter] <= 250:  # 2.sütun
                                    print("a2b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 58.5 and b_y[temporary_counter] <= 117:  # 2.satır
                                if b_x[temporary_counter] > 125 and b_x[temporary_counter] <= 187:  # 1.sütun
                                    print("a2b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 58.5 and b_y[temporary_counter] <= 117:  # 2.satır
                                if b_x[temporary_counter] > 187 and b_x[temporary_counter] <= 250:  # 2.sütun
                                    print("a2b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("156-29,25")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("218,5-29,25")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("156-87,75")
                else:
                    print("4.alana gidilecek")
                    print("218,5-87,75")
                b = [0, 0, 0, 0]
            elif index == 3:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] <= 58.5:  # 1.satır
                                if b_x[temporary_counter] > 250 and b_x[temporary_counter] <= 312.5:  # 1.sütun
                                    print("a3b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] <= 58.5:  # 1.satır
                                if b_x[temporary_counter] > 312.5 and b_x[temporary_counter] <= 375:  # 2.sütun
                                    print("a3b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 58.5 and b_y[temporary_counter] <= 117:  # 2.satır
                                if b_x[temporary_counter] > 250 and b_x[temporary_counter] <= 312.5:  # 1.sütun
                                    print("a3b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 58.5 and b_y[temporary_counter] <= 117:  # 2.satır
                                if b_x[temporary_counter] > 312.5 and b_x[temporary_counter] <= 375:  # 2.sütun
                                    print("a3b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("281,25-29,25")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("343,75-29,25")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("281,25-87,75")
                else:
                    print("4.alana gidilecek")
                    print("343,75-87,75")
                b = [0, 0, 0, 0]
            elif index == 4:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] <= 58.5:  # 1.satır
                                if b_x[temporary_counter] > 375 and b_x[temporary_counter] <= 437.5:  # 1.sütun
                                    print("a4b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] <= 58.5:  # 1.satır
                                if b_x[temporary_counter] > 437.5:  # 2.sütun
                                    print("a4b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 58.5 and b_y[temporary_counter] <= 117:  # 2.satır
                                if b_x[temporary_counter] > 375 and b_x[temporary_counter] <= 437.5:  # 1.sütun
                                    print("a4b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 58.5 and b_y[temporary_counter] <= 117:  # 2.satır
                                if b_x[temporary_counter] > 437.5:  # 2.sütun
                                    print("a4b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("406,25-29,25")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("468,75-29,25")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("406,25-87,75")
                else:
                    print("4.alana gidilecek")
                    print("468,75-87,75")
                b = [0, 0, 0, 0]
            elif index == 5:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] > 117 and b_y[temporary_counter] <= 175.5:  # 1.satır
                                if b_x[temporary_counter] <= 62.5:  # 1.sütun
                                    print("a5b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] > 117 and b_y[temporary_counter] <= 175.5:  # 1.satır
                                if b_x[temporary_counter] > 62.5 and b_x[temporary_counter] <= 125:  # 2.sütun
                                    print("a5b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 175.5 and b_y[temporary_counter] <= 234:  # 2.satır
                                if b_x[temporary_counter] <= 62.5:  # 1.sütun
                                    print("a5b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 175.5 and b_y[temporary_counter] <= 234:  # 2.satır
                                if b_x[temporary_counter] > 62.5 and b_x[temporary_counter] <= 125:  # 2.sütun
                                    print("a5b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("31,25-146,25")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("93,75-146,25")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("31,25-204,75")
                else:
                    print("4.alana gidilecek")
                    print("93,75-204,75")
                b = [0, 0, 0, 0]
            elif index == 6:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] > 117 and b_y[temporary_counter] <= 175.5:  # 1.satır
                                if b_x[temporary_counter] > 125 and b_x[temporary_counter] <= 187:  # 1.sütun
                                    print("a6b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] > 117 and b_y[temporary_counter] <= 175.5:  # 1.satır
                                if b_x[temporary_counter] > 187 and b_x[temporary_counter] <= 250:  # 2.sütun
                                    print("a6b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 175.5 and b_y[temporary_counter] <= 234:  # 2.satır
                                if b_x[temporary_counter] > 125 and b_x[temporary_counter] <= 187:  # 1.sütun
                                    print("a6b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 175.5 and b_y[temporary_counter] <= 234:  # 2.satır
                                if b_x[temporary_counter] > 187 and b_x[temporary_counter] <= 250:  # 2.sütun
                                    print("a6b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("156-146,25")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("218,5-146,25")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("156-204,75")
                else:
                    print("4.alana gidilecek")
                    print("218,5-204,75")
                b = [0, 0, 0, 0]
            elif index == 7:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] > 117 and b_y[temporary_counter] <= 175.5:  # 1.satır
                                if b_x[temporary_counter] > 250 and b_x[temporary_counter] <= 312.5:  # 1.sütun
                                    print("a7b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] > 117 and b_y[temporary_counter] <= 175.5:  # 1.satır
                                if b_x[temporary_counter] > 312.5 and b_x[temporary_counter] <= 375:  # 2.sütun
                                    print("a7b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 175.5 and b_y[temporary_counter] <= 234:  # 2.satır
                                if b_x[temporary_counter] > 250 and b_x[temporary_counter] <= 312.5:  # 1.sütun
                                    print("a7b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 175.5 and b_y[temporary_counter] <= 234:  # 2.satır
                                if b_x[temporary_counter] > 312.5 and b_x[temporary_counter] <= 375:  # 2.sütun
                                    print("a7b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("281,25-146,25")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("343,75-146,25")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("281,25-204,75")
                else:
                    print("4.alana gidilecek")
                    print("343,75-204,75")
                b = [0, 0, 0, 0]
            elif index == 8:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] > 117 and b_y[temporary_counter] <= 175.5:  # 1.satır
                                if b_x[temporary_counter] > 375 and b_x[temporary_counter] <= 437.5:  # 1.sütun
                                    print("a8b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] > 117 and b_y[temporary_counter] <= 175.5:  # 1.satır
                                if b_x[temporary_counter] > 437.5:  # 2.sütun
                                    print("a8b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 175.5 and b_y[temporary_counter] <= 234:  # 2.satır
                                if b_x[temporary_counter] > 375 and b_x[temporary_counter] <= 437.5:  # 1.sütun
                                    print("a8b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 175.5 and b_y[temporary_counter] <= 234:  # 2.satır
                                if b_x[temporary_counter] > 437.5:  # 2.sütun
                                    print("a8b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("406,25-146,25")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("468,75-146,25")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("406,25-204,75")
                else:
                    print("4.alana gidilecek")
                    print("468,75-204,75")
                b = [0, 0, 0, 0]
            elif index == 9:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] > 234 and b_y[temporary_counter] <= 292:  # 1.satır
                                if b_x[temporary_counter] <= 62.5:  # 1.sütun
                                    print("a9b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] > 234 and b_y[temporary_counter] <= 292:  # 1.satır
                                if b_x[temporary_counter] > 62.5 and b_x[temporary_counter] <= 125:  # 2.sütun
                                    print("a9b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 292:  # 2.satır
                                if b_x[temporary_counter] <= 62.5:  # 1.sütun
                                    print("a9b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 292:  # 2.satır
                                if b_x[temporary_counter] > 62.5 and b_x[temporary_counter] <= 125:  # 2.sütun
                                    print("a9b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("31,25-263")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("93,75-263")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("31,25-321")
                else:
                    print("4.alana gidilecek")
                    print("93,75-321")
                b = [0, 0, 0, 0]
            elif index == 10:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] > 234 and b_y[temporary_counter] <= 292:  # 1.satır
                                if b_x[temporary_counter] > 125 and b_x[temporary_counter] <= 187:  # 1.sütun
                                    print("a10b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] > 234 and b_y[temporary_counter] <= 292:  # 1.satır
                                if b_x[temporary_counter] > 187 and b_x[temporary_counter] <= 250:  # 2.sütun
                                    print("a10b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 292:  # 2.satır
                                if b_x[temporary_counter] > 125 and b_x[temporary_counter] <= 187:  # 1.sütun
                                    print("a10b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 292:  # 2.satır
                                if b_x[temporary_counter] > 187 and b_x[temporary_counter] <= 250:  # 2.sütun
                                    print("a10b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("156-263")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("218,5-263")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("156-321")
                else:
                    print("4.alana gidilecek")
                    print("218,5-321")
                b = [0, 0, 0, 0]
            elif index == 11:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] > 234 and b_y[temporary_counter] <= 292:  # 1.satır
                                if b_x[temporary_counter] > 250 and b_x[temporary_counter] <= 312.5:  # 1.sütun
                                    print("a11b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] > 234 and b_y[temporary_counter] <= 292:  # 1.satır
                                if b_x[temporary_counter] > 312.5 and b_x[temporary_counter] <= 375:  # 2.sütun
                                    print("a11b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 292:  # 2.satır
                                if b_x[temporary_counter] > 250 and b_x[temporary_counter] <= 312.5:  # 1.sütun
                                    print("a11b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 292:  # 2.satır
                                if b_x[temporary_counter] > 312.5 and b_x[temporary_counter] <= 375:  # 2.sütun
                                    print("a11b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("281,25-263")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("343,75-263")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("281,25-321")
                else:
                    print("4.alana gidilecek")
                    print("343,75-321")
                b = [0, 0, 0, 0]
            else:
                #
                for counter in range(4):
                    for temporary_counter in range(total_competitor):
                        b_x[temporary_counter] += a_hiz * time_arama * np.cos(
                            b_yon[
                                temporary_counter] / 180 * 3.141592653589793)  # bu sürede öbür droneların bulunacağı koordinatlar
                        b_y[temporary_counter] += a_hiz * time_arama * np.sin(
                            b_yon[temporary_counter] / 180 * 3.141592653589793)
                        if counter == 0:
                            if b_y[temporary_counter] > 234 and b_y[temporary_counter] <= 292:  # 1.satır
                                if b_x[temporary_counter] > 375 and b_x[temporary_counter] <= 437.5:  # 1.sütun
                                    print("a12b1 bölgesine eklenecek")
                                    b[0] += 1
                        elif counter == 1:
                            if b_y[temporary_counter] > 234 and b_y[temporary_counter] <= 292:  # 1.satır
                                if b_x[temporary_counter] > 437.5:  # 2.sütun
                                    print("a12b2 bölgesine eklenecek")
                                    b[1] += 1
                        elif counter == 2:
                            if b_y[temporary_counter] > 292:  # 2.satır
                                if b_x[temporary_counter] > 375 and b_x[temporary_counter] <= 437.5:  # 1.sütun
                                    print("a12b3 bölgesine eklenecek")
                                    b[2] += 1
                        else:
                            if b_y[temporary_counter] > 292:  # 2.satır
                                if b_x[temporary_counter] > 437.5:  # 2.sütun
                                    print("a12b4 bölgesine eklenecek")
                                    b[3] += 1
                temporary_b = b[0]
                index_b = 0
                for i in range(4):
                    if b[i] > temporary_b:
                        temporary_b = b[i]
                        index_b = i
                number_b = temporary_b
                print(number_b)
                print(index_b)
                if index_b == 0:
                    print("1.alana gidilecek")
                    print("406,25-263")
                elif index_b == 1:
                    print("2.alana gidilecek")
                    print("468,75-263")
                elif index_b == 2:
                    print("3.alana gidilecek")
                    print("406,25-321")
                else:
                    print("4.alana gidilecek")
                    print("468,75-321")
                b = [0, 0, 0, 0]
        else:
            print("max noktada değil")

tracking = Tracking()