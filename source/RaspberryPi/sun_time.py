#!/usr/bin/env python3
from datetime import datetime, timedelta
import requests
import sys
import serial
import subprocess
import socket
import re
import time
SERVER_IP = "10.10.14.85"
SERVER_PORT = 5000

OPENWEATER_KEY = ""

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((SERVER_IP, SERVER_PORT))

lat, lon = 37.5665, 126.9780
s.sendall(b"[LDH_SUN:PASSWD]")

def get_weather():
    weather_url = f"http://api.openweathermap.org/data/2.5/weather?lat={lat}&lon={lon}&appid={OPENWEATER_KEY}&units=metric&lang=kr"
    weather_data = requests.get(weather_url).json()
    weather_main = weather_data['weather'][0]['main']
    temp = int(weather_data['main']['temp'])

    if weather_main in ["Clear"]:
        status = 0
    elif weather_main in ["Clouds"]:
        status = 1
    elif weather_main in ["Mist","Fog","Haze"]:
        status = 2
    elif weather_main in ["Rain","Drizzle","Thunderstorm"]:
        status = 3
    elif weather_main in ["Snow"]:
        status = 4
    else:
        status = -1

    return status, temp

url = f"https://api.sunrise-sunset.org/json?lat={lat}&lng={lon}&formatted=0"

while True:
    recv_ =s.recv(1024)
    recv_str = recv_.decode().strip()
           
    tokens = re.split(r'[\[\]@]',recv_str)
     
    if tokens[2] == "GETSUN":
        data = requests.get(url).json()

    # UTC 문자열 → datetime 객체
        sunrise_utc = datetime.fromisoformat(data['results']['sunrise'].replace("Z","+00:00"))
        sunset_utc  = datetime.fromisoformat(data['results']['sunset'].replace("Z","+00:00"))

    # 한국 시간대 변환 (+9시간)
        sunrise_kst = sunrise_utc + timedelta(hours=9)
        sunset_kst  = sunset_utc + timedelta(hours=9)
        message = (f"[{tokens[1]}]{tokens[2]}@{sunrise_kst.strftime('%H')}@{sunrise_kst.strftime('%M')}@{sunset_kst.strftime('%H')}@{sunset_kst.strftime('%M')}\n")   
        print(f"{message.strip()}")
        s.send(message.encode())
    
    #time.sleep(60)
    if tokens[2] == "GETWEATHER":
        status, temp = get_weather()
        message = (f"[{tokens[1]}]{tokens[2]}@{status}@{temp}\n")
        print(f"{message.strip()}")
        s.send(message.encode())
    if tokens[2] == "GETTIME":
        now = time.localtime()
        message = (f"[{tokens[1]}]{tokens[2]}@{now.tm_hour}@{now.tm_min}@{now.tm_sec}\n")
        print(f"{message.strip()}")
        s.send(message.encode())
