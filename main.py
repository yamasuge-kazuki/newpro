import serial
import time
import math
import threading
import datetime
import csv
import os
import RPi.GPIO as GPIO
from pathlib import Path

# import wiringpi as pi
import BNO055
from micropyGPS import MicropyGPS

# import matplotlib.pyplot as plt
import RPi.GPIO as GPIO
import sys


# 定数　上書きしない
MAG_CONST = 8.9  # 地磁気補正用の偏角
CALIBRATION_MILLITIME = 20 * 1000
TARGET_LAT = 38.26052
TARGET_LNG = 140.8544151
DATA_SAMPLING_RATE = 0.00001
HIGH = 1
LOW = 0
# Pin number
PWMA=18
AIN1=8
AIN2=25
PWMB=19
BIN1=9
BIN2=11


# 変数
end=0.0
start=0.0
acc = [0.0, 0.0, 0.0]
gyro = [0.0, 0.0, 0.0]
mag = [0.0, 0.0, 0.0]
calibBias = [0.0, 0.0, 0.0]
calibRange = [1.0, 1.0, 1.0]
lat = 0.0  # from GPS sensor
lng = 0.0
pres = 0.0
distance = 0
angle = 0.0
azimuth = 0.0
direction = 0.0
frequency = 50
phase = 0
gps_detect = 0
restTime = 0.0
diff_rot = 1
upside_down_Flag = 0  # judge the upside down by acc(bmx)
stuck_GPS_Flag = 0  # judge the stuck by GPS : no obstacle distance_Flag = 0, if CanSat stucked distance_Flag = 1


bmx = BNO055.BNO055()
nowTime = datetime.datetime.now()
fileName = Path("log") / ("testlog_" + nowTime.strftime("%Y-%m%d-%H%M%S") + ".csv")

def main():
    global phase
    global restTime
    global start
    global gps_detect
    global direction
    # Flag

    GPIO.setwarnings(False)
    Setup()
    
    while True:
        try:
            start = time.time()
            print("phase3 : GPS start")
            if distance < 5.0:
                GPIO.output(AIN1,LOW)
                GPIO.output(AIN2,LOW)
                GPIO.output(BIN1,LOW)
                GPIO.output(BIN2,LOW)
                print("goal!")
                direction=360.0
                exit()
        except KeyboardInterrupt:
            direction=360.0
            GPIO.output(AIN1,LOW)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,LOW)
            GPIO.output(BIN2,LOW)
            exit()
            
def currentMilliTime():
    return round(time.time() * 1000)


def Setup():
    bmx.setUp()
    fileName.parent.mkdir(parents=True, exist_ok=True)
    with open(fileName, "a") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "Time",
                "AccX",
                "AccY",
                "AccZ",
                "GyroX",
                "GyroY",
                "GyroZ",
                "MagX",
                "MagY",
                "MagZ",
                "LAT",
                "LNG",
                "ALT",
                "Distance",
                "Azimuth",
                "Angle",
                "Direction",
            ]
        )

    getThread = threading.Thread(target=moveMotor_thread, args=())
    getThread.daemon = True
    getThread.start()

    dataThread = threading.Thread(target=setData_thread, args=())
    dataThread.daemon = True
    dataThread.start()

    gpsThread = threading.Thread(target=GPS_thread, args=())
    gpsThread.daemon = True
    gpsThread.start()

    print("Setup OK")


def getBmxData():  # get BMX data
    global acc
    global gyro
    global mag
    global fall
    acc = bmx.getAcc()
    gyro = bmx.getGyro()
    mag = bmx.getMag()
    # mag[1] = mag[1]
    # mag[2] = mag[2]
    fall = math.sqrt(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2])
    #for i in range(3):
    #    mag[i] = (mag[i] - calibBias[i]) / calibRange[i]




def upside_down():
    global upside_down_Flag
    global acc
    if acc[0] > 0:
        upside_down_Flag = 1


def calcdistance():  # 距離計算用関
    global distance
    EARTH_RADIUS = 6378136.59
    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    distance = math.sqrt(dx * dx + dy * dy)
    # print(f"distance: {distance}")


def calcAngle():  # 角度計算用関数 : north=0 east=90 west = -90
    global angle
    forEAstAngle = 0.0
    EARTH_RADIUS = 6378136.59

    dx = (math.pi / 180) * EARTH_RADIUS * (TARGET_LNG - lng)
    dy = (math.pi / 180) * EARTH_RADIUS * (TARGET_LAT - lat)
    angle = 90 - math.degrees(math.atan2(dy, dx))
    angle %= 360.0
    # if dx == 0 and dy == 0:
    #    forEastAngle = 0.0
    # else:
    #    forEastAngle = (180 / math.pi) * math.atan2(dy, dx)  # arctan
    # angle = forEastAngle - 90
    # if angle < -180:
    #    angle += 360
    # if angle > 180:
    #    angle -= 360
    # angle = -angle
    # print(f"angle: {angle}")


def calcAzimuth():  # 方位角計算用関数
    global azimuth

    azimuth = 90 - math.degrees(math.atan2(mag[1], mag[0]))
    azimuth *= -1
    azimuth % 360  # 上のazimuthはCanSatからみた北の方位
    # print(f"azimuth: {azimuth}")
    # if mag[1] == 0.0:
    #     mag[1] = 0.0000001
    # azimuth = -(180 / math.pi) * math.atan(mag[2] / mag[1])
    # if mag[1] > 0:
    #     azimuth = 90 + azimuth
    # elif mag[1] < 0:
    #     azimuth = -90 + azimuth


def GPS_thread():  # GPSモジュールを読み、GPSオブジェクトを更新する
    global lat
    global lng
    global gps_detect

    s = serial.Serial("/dev/serial0", 9600)
    s.readline()  # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    gps = MicropyGPS(9, "dd")

    while True:
        sentence = s.readline().decode("utf-8")  # GPSデーターを読み、文字列に変換する

        if s.in_waiting > 64:  # バッファを削除
            s.reset_input_buffer()
        if sentence[0] != "$":  # 先頭が'$'でなければ捨てる
            continue
        for (x) in (sentence):  # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
            gps.update(x)
        lat = gps.latitude[0]
        lng = gps.longitude[0]

        if lat > 0:
            gps_detect = 1
        elif lat == 0.0:
            gps_detect = 0
            print("None GNSS value")
        # print(lat)
        # print(lng)

def setData_thread():
    while True:
        getBmxData()
        calcAngle()
        calcAzimuth()
        set_direction()
        calcdistance()
        end=time.time()
        print(f'lat:{lat}')
        print(f'lng:{lng}')
        print(f'azimuth:{azimuth}')
        print(f'angle:{angle}')
        with open(fileName, "a", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    
                    round(end-start,3),
                    round(phase, 1),
                    acc[0],
                    acc[1],
                    acc[2],
                    gyro[0],
                    gyro[1],
                    gyro[2],
                    mag[0],
                    mag[1],
                    mag[2],
                    lat,
                    lng,
                    distance,
                    azimuth,
                    angle,
                    direction
                ]
            )
        time.sleep(DATA_SAMPLING_RATE)


def moveMotor_thread():
    GPIO.setmode(GPIO.BCM)

    GPIO.setup(PWMA, GPIO.OUT)
    GPIO.setup(AIN1, GPIO.OUT)
    GPIO.setup(AIN2, GPIO.OUT)
    GPIO.setup(PWMB, GPIO.OUT)
    GPIO.setup(BIN1, GPIO.OUT)
    GPIO.setup(BIN2, GPIO.OUT)

    M_pwmA = GPIO.PWM(PWMA, frequency)
    M_pwmB = GPIO.PWM(PWMB, frequency)

    M_pwmA.start(100) # right tire pwm
    M_pwmB.start(100)

    
    while True:
        if direction == 360.0:  # stop
            M_pwmA.ChangeDutyCycle(0)
            M_pwmB.ChangeDutyCycle(0)
            GPIO.output(AIN1,LOW)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,LOW)
            GPIO.output(BIN2,LOW) 

        elif direction == 500.0:  # left 
            M_pwmA.ChangeDutyCycle(0)
            M_pwmB.ChangeDutyCycle(100)
            GPIO.output(AIN1,LOW)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW) 

        elif direction == 600.0:  # right 
            M_pwmA.ChangeDutyCycle(100)
            M_pwmB.ChangeDutyCycle(0)
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,LOW)
            GPIO.output(BIN2,LOW) 

        elif direction == -360.0:  #forward
            M_pwmA.ChangeDutyCycle(100)
            M_pwmB.ChangeDutyCycle(100)
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW) 

        elif direction == -400.0:  # rotate  left
            M_pwmA.ChangeDutyCycle(60)
            M_pwmB.ChangeDutyCycle(100)
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW)  
            
        elif direction > 0.0 and direction <= 180.0:  # turn left
            M_pwmA.ChangeDutyCycle(20)
            M_pwmB.ChangeDutyCycle(100)
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW) 

        elif direction < 0.0 and direction >= -180.0:  # turn right
            M_pwmA.ChangeDutyCycle(100)
            M_pwmB.ChangeDutyCycle(20)            
            GPIO.output(AIN1,HIGH)
            GPIO.output(AIN2,LOW)
            GPIO.output(BIN1,HIGH)
            GPIO.output(BIN2,LOW) 



def set_direction():  # -180<direction<180  #rover move to right while direction > 0
    global direction
    global phase
    global upside_down_Flag

    direction=360.0
    direction = azimuth - angle
    direction %= 360
    if (direction > 180):
        direction -= 360
    if abs(direction) < 5.0:
        direction = -360

if __name__ == "__main__":
    main()
