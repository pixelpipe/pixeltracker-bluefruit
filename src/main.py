"""
pixeltracker - A Circuit Playground BLE GPS and Sensor Tracker
Copyright @2021 by pixelchain
"""
import time

# GLOBALS
app_PixeTrackerLite = 0
app_PixeTracker = 1
app_Debug = True

# BOARD LED
import digitalio
import board
boardLED = digitalio.DigitalInOut(board.D13)
boardLED.switch_to_output()

# APP SELECTOR
app = app_PixeTrackerLite

# SELECTIVE APP DEPENDENCIES
if app == app_PixeTracker:    
    from pixeltracker import Pixeltracker
else:
    from ubxgps import PixelTrackerLite

# MAIN
if __name__ == '__main__':

    # SELECT APP
    if app == app_PixeTracker:
        pixeltracker = Pixeltracker()
    else:
        pixeltracker = PixelTrackerLite()

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