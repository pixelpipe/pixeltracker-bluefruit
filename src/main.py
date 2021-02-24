"""
pixeltracker - A Circuit Playground BLE GPS and Sensor Tracker
Copyright @2021 by pixelchain
"""
import time
from ubxgps import PixelTrackerLite

# GLOBALS
app_Debug = False

# BOARD LED
import digitalio
import board

boardLED = digitalio.DigitalInOut(board.D13)
boardLED.switch_to_output()

# MAIN
if __name__ == '__main__':

    pixeltracker = PixelTrackerLite()

    GPS_LED = 4
    
    # RUN
    if app_Debug:
        # DEBUG RUN
        pixeltracker.run()
    else:
        # RELEASE RUN    
        while True:
            try:            
                pixeltracker.run()
            except Exception as e:
                boardLED.value = True
                print(e)
                time.sleep(1)
