import py_qmc5883l

sensor = py_qmc5883l.QMC5883L()


while(True):
    m = sensor.get_magnet()
    
    b = sensor.get_bearing()

    print("Magnet: ", m, "\tBearing: ", b)
