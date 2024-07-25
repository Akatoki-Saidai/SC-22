import time
import serial
from micropyGPS import MicropyGPS

def main():
    # シリアル通信設定
    print("A")
    uart = serial.Serial('/dev/ttyAMA0', 9600, timeout = 10)
    # gps設定
    my_gps = MicropyGPS(9, 'dd')

    # 10秒ごとに表示
    tm_last = 0
    while True:
        print("C")
        sentence = uart.readline()
        if len(sentence) > 0:
            for x in sentence:
                if 10 <= x <= 126:
                    print("D")
                    stat = my_gps.update(chr(x))
                    if stat:
                        tm = my_gps.timestamp
                        tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                        if (tm_now - tm_last) >= 10:
                            print('=' * 20)
                            print(my_gps.date_string(), tm[0], tm[1], int(tm[2]))
                            print("latitude:", float(my_gps.latitude[0]), ", longitude:", float(my_gps.longitude[0]))

if __name__ == "__main__":
    print("B")
    main()
