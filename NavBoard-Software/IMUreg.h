
#ifndef IMUREG
#define IMUREG

byte ACT_THS              = 0B00000100;
byte ACT_DUR              = 0B00000101;
byte INT_GEN_CFG_XL       = 0B00000110;
byte INT_GEN_THS_X_XL     = 0B00000111;
byte INT_GEN_THS_Y_XL     = 0B00001000;
byte INT_GEN_THS_Z_XL     = 0B00001001;
byte INT_GEN_DUR_XL       = 0B00001010;
byte REFERENCE_G          = 0B00001011;
byte INT1_CTRL            = 0B00001100;
byte INT2_CTRL            = 0B00001101;

byte WHO_AM_I             = 0B00001111;
byte CTRL_REG1_G          = 0B00010000;
byte CTRL_REG2_G          = 0B00010001;
byte CTRL_REG3_G          = 0B00010010;
byte ORIENT_CFG_G         = 0B00010011;
byte INT_GEN_SRC_G        = 0B00010100;
byte OUT_TEMP_L           = 0B00010101;
byte OUT_TEMP_H           = 0B00010110;
byte STATUS_REG           = 0B00010111;
byte OUT_X_L_G            = 0B00011000;
byte OUT_X_H_G            = 0B00011001;
byte OUT_Y_L_G            = 0B00011010;
byte OUT_Y_H_G            = 0B00011011;
byte OUT_Z_L_G            = 0B00011100;
byte OUT_Z_H_G            = 0B00011101;
byte CTRL_REG4            = 0B00011110;
byte CTRL_REG5_XL         = 0B00011111;
byte CTRL_REG6_XL         = 0B00100000;
byte CTRL_REG7_XL         = 0B00100001;
byte CTRL_REG8            = 0B00100010;
byte CTRL_REG9            = 0B00100011;
byte CTRL_REG10           = 0B00100100;

byte INT_GEN_SRC_XL       = 0B00100110;
byte STATUS_REG0          = 0B00100111;
byte OUT_X_L_XL           = 0B00101000;
byte OUT_X_H_XL           = 0B00101001;
byte OUT_Y_L_XL           = 0B00101010;
byte OUT_Y_H_XL           = 0B00101011;
byte OUT_Z_L_XL           = 0B00101100;
byte OUT_Z_H_XL           = 0B00101101;
byte FIFO_CTRL            = 0B00101110;
byte FIFO_SRC             = 0B00101111;
byte INT_GEN_CFG_G        = 0B00110000;
byte INT_GEN_THS_XH_G     = 0B00110001;
byte INT_GEN_THS_XL_G     = 0B00110010;
byte INT_GEN_THS_YH_G     = 0B00110011;
byte INT_GEN_THS_YL_G     = 0B00110100;
byte INT_GEN_THS_ZH_G     = 0B00110101;
byte INT_GEN_THS_ZL_G     = 0B00110110;
byte INT_GEN_DUR_G        = 0B00110111;


byte OFFSET_X_REG_L_M     = 0B00000101;
byte OFFSET_X_REG_H_M     = 0B00000110;
byte OFFSET_Y_REG_L_M     = 0B00000111;
byte OFFSET_Y_REG_H_M     = 0B00001000;
byte OFFSET_Z_REG_L_M     = 0B00001001;
byte OFFSET_Z_REG_H_M     = 0B00001010;

byte WHO_AM_I_M           = 0B00001111;

byte CTRL_REG1_M          = 0B00100000;
byte CTRL_REG2_M          = 0B00100001;
byte CTRL_REG3_M          = 0B00100010;
byte CTRL_REG4_M          = 0B00100011;
byte CTRL_REG5_M          = 0B00100100;

byte STATUS_REG_M         = 0B00100111;
byte OUT_X_L_M            = 0B00101000;
byte OUT_X_H_M            = 0B00101001;
byte OUT_Y_L_M            = 0B00101010;
byte OUT_Y_H_M            = 0B00101011;
byte OUT_Z_L_M            = 0B00101100;
byte OUT_Z_H_M            = 0B00101101;

byte INT_CFG_M            = 0B00110000;
byte INT_SRC_M            = 0B00110001;
byte INT_THS_L_M          = 0B00110010;
byte INT_THS_H_M          = 0B00110011;

byte Read    = 0B00000001;
byte Write   = 0B00000000;
byte Address_AG =   0B01101011;  //address of accelerometer/gyro with SDO_AG connected to Vdd
byte Address_M   =   0B00011110;  //address of magnetometer with SDO_M connected to Vdd

#endif
