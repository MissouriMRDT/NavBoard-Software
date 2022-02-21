import qmc5883l
import math

sensor = qmc5883l.QMC5883L()

while(True):
    m = sensor.get_magnet()
    
    azimuth = math.atan2(m[0], m[1]) * 180/math.pi
    if (azimuth < 0):
        azimuth += 360
    
    print("Magnet: ", m)