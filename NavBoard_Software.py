from gps import *
import time
from RoveComm_Python.rovecomm import* 
import py_qmc5883l

# Get rovecomm manifest
manifest = get_manifest()

# Initialize rovecomm
rovecomm_node = RoveComm(11000, ("", manifest["Nav"]["Port"]))

#Initialize GPS
gpsd = gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)

# Initialize Compass
compass = py_qmc5883l.QMC5883L()

LatLon = [0.0, 0.0]
#Pitch, Yaw, Roll
PYR = [0.0, 0.0, 0.0]
NumSat = 0
Bearing = 0

while True:
    
    #Read GPS Data
    report = gpsd.next()
    if report['class'] == 'TPV' :
        LatLon[0] = getattr(report, 'lat', 0.0)
        LatLon[1] = getattr(report, 'lon', 0.0)
        NumSat = len(gpsd.satellites)
    
    Bearing = compass.get_bearing()
    
    #Send RoveComm Packets
    packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["GPSLatLon"]["dataId"], "f", (LatLon[0], LatLon[1]))
    rovecomm_node.write(packet, False)
    
    packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["IMUData"]["dataId"], "f", (PYR[0], PYR[1], PYR[2]))
    rovecomm_node.write(packet, False)
    
    packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["CompassData"]["dataId"], "f", (Bearing,))
    rovecomm_node.write(packet, False)
    
    packet = RoveCommPacket(manifest["Nav"]["Telemetry"]["SatelliteCountData"]["dataId"], "h", (NumSat,))
    rovecomm_node.write(packet, False)
    
        
    #print(LatLon, "\t", NumSat, "\t", Bearing)