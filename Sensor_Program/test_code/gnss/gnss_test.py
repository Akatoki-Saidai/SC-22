import serial
import micropyGPS

gps = micropyGPS.MicropyGPS(9, 'dd') # MicroGPSオブジェクトを生成する。
                                     # 引数はタイムゾーンの時差と出力フォーマット

def rungps(): # GPSモジュールを読み、GPSオブジェクトを更新する
    s = serial.Serial('/dev/serial0', 9600, timeout=10)
    s.readline() # 最初の1行は中途半端なデーターが読めることがあるので、捨てる
    tm_last = 0
    while True:
        sentence = s.readline().decode('utf-8') # GPSデーターを読み、文字列に変換する
        try:
            if sentence[0] != '$': # 先頭が'$'でなければ捨てる
                continue
            for x in sentence: # 読んだ文字列を解析してGPSオブジェクトにデーターを追加、更新する
                gps.update(x)
                if 10 <= x <= 126:
                    stat = gps.update(chr(x))
                    if stat:
                        tm = gps.timestamp
                        tm_now = (tm[0] * 3600) + (tm[1] * 60) + int(tm[2])
                        if (tm_now - tm_last) >= 10:
                            h = gps.timestamp[0] if gps.timestamp[0] < 24 else gps.timestamp[0] - 24
                            print('%2d:%02d:%04.1f' % (h, gps.timestamp[1], gps.timestamp[2]))
                            print('緯度経度: %2.8f, %2.8f' % (gps.latitude[0], gps.longitude[0]))
                            print('海抜: %f' % gps.altitude)
                            print(gps.satellites_used)
                            print('衛星番号: (仰角, 方位角, SN比)')
        except IndexError:
            print(s)
            print('IndexError')
            pass

rungps()
