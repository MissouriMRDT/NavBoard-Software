#ifndef _IMUREG
#define _IMUREG

#include "Energia.h"

#define ACT_THS          0B00000100
#define ACT_DUR          0B00000101
#define INT_GEN_CFG_XL   0B00000110
#define INT_GEN_THS_X_XL 0B00000111
#define INT_GEN_THS_Y_XL 0B00001000
#define INT_GEN_THS_Z_XL 0B00001001
#define INT_GEN_DUR_XL   0B00001010
#define REFERENCE_G      0B00001011
#define INT1_CTRL        0B00001100
#define INT2_CTRL        0B00001101

#define WHO_AM_I         0B00001111
#define CTRL_REG1_G      0B00010000
#define CTRL_REG2_G      0B00010001
#define CTRL_REG3_G      0B00010010
#define ORIENT_CFG_G     0B00010011
#define INT_GEN_SRC_G    0B00010100
#define OUT_TEMP_L       0B00010101
#define OUT_TEMP_H       0B00010110
#define STATUS_REG       0B00010111
#define OUT_X_L_G        0B00011000
#define OUT_X_H_G        0B00011001
#define OUT_Y_L_G        0B00011010
#define OUT_Y_H_G        0B00011011
#define OUT_Z_L_G        0B00011100
#define OUT_Z_H_G        0B00011101
#define CTRL_REG4        0B00011110
#define CTRL_REG5_XL     0B00011111
#define CTRL_REG6_XL     0B00100000
#define CTRL_REG7_XL     0B00100001
#define CTRL_REG8        0B00100010
#define CTRL_REG9        0B00100011
#define CTRL_REG10       0B00100100

#define INT_GEN_SRC_XL   0B00100110
#define STATUS_REG0      0B00100111
#define OUT_X_L_XL       0B00101000
#define OUT_X_H_XL       0B00101001
#define OUT_Y_L_XL       0B00101010
#define OUT_Y_H_XL       0B00101011
#define OUT_Z_L_XL       0B00101100
#define OUT_Z_H_XL       0B00101101
#define FIFO_CTRL        0B00101110
#define FIFO_SRC         0B00101111
#define INT_GEN_CFG_G    0B00110000
#define INT_GEN_THS_XH_G 0B00110001
#define INT_GEN_THS_XL_G 0B00110010
#define INT_GEN_THS_YH_G 0B00110011
#define INT_GEN_THS_YL_G 0B00110100
#define INT_GEN_THS_ZH_G 0B00110101
#define INT_GEN_THS_ZL_G 0B00110110
#define INT_GEN_DUR_G    0B00110111

#define OFFSET_X_REG_L_M 0B00000101
#define OFFSET_X_REG_H_M 0B00000110
#define OFFSET_Y_REG_L_M 0B00000111
#define OFFSET_Y_REG_H_M 0B00001000
#define OFFSET_Z_REG_L_M 0B00001001
#define OFFSET_Z_REG_H_M 0B00001010

#define WHO_AM_I_M       0B00001111

#define CTRL_REG1_M      0B00100000
#define CTRL_REG2_M      0B00100001
#define CTRL_REG3_M      0B00100010
#define CTRL_REG4_M      0B00100011
#define CTRL_REG5_M      0B00100100

#define STATUS_REG_M     0B00100111
#define OUT_X_L_M        0B00101000
#define OUT_X_H_M        0B00101001
#define OUT_Y_L_M        0B00101010
#define OUT_Y_H_M        0B00101011
#define OUT_Z_L_M        0B00101100
#define OUT_Z_H_M        0B00101101

#define INT_CFG_M        0B00110000
#define INT_SRC_M        0B00110001
#define INT_THS_L_M      0B00110010
#define INT_THS_H_M      0B00110011

#define Read             0B00000001
#define Write            0B00000000 01101011
#define Address_AG       0B01101011  //address of accelerometer/gyro with SDO_AG connected to Vdd
#define Address_M        0B00011110  //address of magnetometer with SDO_M connected to Vdd

#define ADO 1
#if ADO
#define LSM9DS1XG_ADDRESS 0x6B  //  Device address when ADO = 1
#define LSM9DS1M_ADDRESS  0x1E  //  Address of magnetometer
#define MS5611_ADDRESS    0x77  //  Address of altimeter
#else
#define LSM9DS1XG_ADDRESS 0x6A   //  Device address when ADO = 0
#define LSM9DS1M_ADDRESS  0x1D   //  Address of magnetometer
#define MS5611_ADDRESS    0x77   //  Address of altimeter
#endif  

///////////////////////////////////////////////////////////
#define LSM9DS1XG_ACT_THS	        0x04
#define LSM9DS1XG_ACT_DUR	        0x05
#define LSM9DS1XG_INT_GEN_CFG_XL    0x06
#define LSM9DS1XG_INT_GEN_THS_X_XL  0x07
#define LSM9DS1XG_INT_GEN_THS_Y_XL  0x08
#define LSM9DS1XG_INT_GEN_THS_Z_XL  0x09
#define LSM9DS1XG_INT_GEN_DUR_XL    0x0A
#define LSM9DS1XG_REFERENCE_G       0x0B
#define LSM9DS1XG_INT1_CTRL         0x0C
#define LSM9DS1XG_INT2_CTRL         0x0D
#define LSM9DS1XG_WHO_AM_I          0x0F  // should return 0x68
#define LSM9DS1XG_CTRL_REG1_G       0x10
#define LSM9DS1XG_CTRL_REG2_G       0x11
#define LSM9DS1XG_CTRL_REG3_G       0x12
#define LSM9DS1XG_ORIENT_CFG_G      0x13
#define LSM9DS1XG_INT_GEN_SRC_G     0x14
#define LSM9DS1XG_OUT_TEMP_L        0x15
#define LSM9DS1XG_OUT_TEMP_H        0x16
#define LSM9DS1XG_STATUS_REG        0x17
#define LSM9DS1XG_OUT_X_L_G         0x18
#define LSM9DS1XG_OUT_X_H_G         0x19
#define LSM9DS1XG_OUT_Y_L_G         0x1A
#define LSM9DS1XG_OUT_Y_H_G         0x1B
#define LSM9DS1XG_OUT_Z_L_G         0x1C
#define LSM9DS1XG_OUT_Z_H_G         0x1D
#define LSM9DS1XG_CTRL_REG4         0x1E
#define LSM9DS1XG_CTRL_REG5_XL      0x1F
#define LSM9DS1XG_CTRL_REG6_XL      0x20
#define LSM9DS1XG_CTRL_REG7_XL      0x21
#define LSM9DS1XG_CTRL_REG8         0x22
#define LSM9DS1XG_CTRL_REG9         0x23
#define LSM9DS1XG_CTRL_REG10        0x24
#define LSM9DS1XG_INT_GEN_SRC_XL    0x26
//#define LSM9DS1XG_STATUS_REG        0x27 // duplicate of 0x17!
#define LSM9DS1XG_OUT_X_L_XL        0x28
#define LSM9DS1XG_OUT_X_H_XL        0x29
#define LSM9DS1XG_OUT_Y_L_XL        0x2A
#define LSM9DS1XG_OUT_Y_H_XL        0x2B
#define LSM9DS1XG_OUT_Z_L_XL        0x2C
#define LSM9DS1XG_OUT_Z_H_XL        0x2D
#define LSM9DS1XG_FIFO_CTRL         0x2E
#define LSM9DS1XG_FIFO_SRC          0x2F
#define LSM9DS1XG_INT_GEN_CFG_G     0x30
#define LSM9DS1XG_INT_GEN_THS_XH_G  0x31
#define LSM9DS1XG_INT_GEN_THS_XL_G  0x32
#define LSM9DS1XG_INT_GEN_THS_YH_G  0x33
#define LSM9DS1XG_INT_GEN_THS_YL_G  0x34
#define LSM9DS1XG_INT_GEN_THS_ZH_G  0x35
#define LSM9DS1XG_INT_GEN_THS_ZL_G  0x36
#define LSM9DS1XG_INT_GEN_DUR_G     0x37
//
// Magnetometer registers
#define LSM9DS1M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M   0x0A
#define LSM9DS1M_WHO_AM_I           0x0F  // should be 0x3D
#define LSM9DS1M_CTRL_REG1_M        0x20
#define LSM9DS1M_CTRL_REG2_M        0x21
#define LSM9DS1M_CTRL_REG3_M        0x22
#define LSM9DS1M_CTRL_REG4_M        0x23
#define LSM9DS1M_CTRL_REG5_M        0x24
#define LSM9DS1M_STATUS_REG_M       0x27
#define LSM9DS1M_OUT_X_L_M          0x28
#define LSM9DS1M_OUT_X_H_M          0x29
#define LSM9DS1M_OUT_Y_L_M          0x2A
#define LSM9DS1M_OUT_Y_H_M          0x2B
#define LSM9DS1M_OUT_Z_L_M          0x2C
#define LSM9DS1M_OUT_Z_H_M          0x2D
#define LSM9DS1M_INT_CFG_M          0x30
#define LSM9DS1M_INT_SRC_M          0x31
#define LSM9DS1M_INT_THS_L_M        0x32
#define LSM9DS1M_INT_THS_H_M        0x33

#endif
