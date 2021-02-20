"""
pixeltracker - A Circuit Playground BLE GPS and Sensor Tracker
Copyright @2021 by pixelchain
"""
import time
import peripherals

app_PixeTrackerLite = 0
app_PixeTracker = 1

# Select the application to run
app = app_PixeTrackerLite

# Selective app Dependencies
if app == app_PixeTracker:
    from pixeltracker import Pixeltracker
else:
    from ubxgps import PixelTrackerLite

# Main
if __name__ == '__main__':
    if app == app_PixeTracker:
        pixeltracker = Pixeltracker()
    else:
        pixeltracker = PixelTrackerLite()

    pixeltracker.run()