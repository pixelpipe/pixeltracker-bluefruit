"""
pixeltracker - A Circuit Playground BLE GPS and Sensor Tracker
Copyright @2021 by pixelchain
"""

from pixeltracker import Pixeltracker
import time
import peripherals

if __name__ == '__main__':
    pixeltracker = Pixeltracker()
    pixeltracker.run()    
