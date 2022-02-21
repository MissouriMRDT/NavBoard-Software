from gps import *
import time

gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

while True:
    report = gpsd.next()
    if report['class'] == 'TPV' :
        print (getattr(report, 'lat', 0.0), "\t", getattr(report, 'lon', 0.0), "\t", len(gpsd.satellites))