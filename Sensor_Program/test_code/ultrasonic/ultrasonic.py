import gpiozero
import time

ECHO = 18

echo = gpiozero.DigitalInputDevice(ECHO)

def get_distance(echo):
    time.sleep(0.0002)

    while echo.is_active == False:
        pulse_start = time.time()

    while echo.is_active == True:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = 34300 * (pulse_duration/2)

    return(distance)

while True:
    dist = get_distance(echo)
    print(dist)
