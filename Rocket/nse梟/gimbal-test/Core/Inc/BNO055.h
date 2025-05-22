#ifndef __BNO055_H__
#define __BNO055_H__

#include "stdint.h"
#include "stdbool.h"
#include "main.h"

#define BNO055_ADRESS_0 0x28
#define BNO055_ADRESS_1 0x29

//------------------Page0------------------
#define BNO055_MAG_RADIUS_MSB      0x6A
#define BNO055_MAG_RADIUS_LSB      0x69
#define BNO055_ACC_RADIUS_MSB      0x68
#define BNO055_ACC_RADIUS_LSB      0x67
#define BNO055_GYR_OFFSET_Z_MSB    0x66
#define BNO055_GYR_OFFSET_Z_LSB    0x65
#define BNO055_GYR_OFFSET_Y_MSB    0x64
#define BNO055_GYR_OFFSET_Y_LSB    0x63
#define BNO055_GYR_OFFSET_X_MSB    0x62
#define BNO055_GYR_OFFSET_X_LSB    0x61
#define BNO055_MAG_OFFSET_Z_MSB    0x60
#define BNO055_MAG_OFFSET_Z_LSB    0x5F
#define BNO055_MAG_OFFSET_Y_MSB    0x5E
#define BNO055_MAG_OFFSET_Y_LSB    0x5D
#define BNO055_MAG_OFFSET_X_MSB    0x5C
#define BNO055_MAG_OFFSET_X_LSB    0x5B
#define BNO055_ACC_OFFSET_Z_MSB    0x5A
#define BNO055_ACC_OFFSET_Z_LSB    0x59
#define BNO055_ACC_OFFSET_Y_MSB    0x58
#define BNO055_ACC_OFFSET_Y_LSB    0x57
#define BNO055_ACC_OFFSET_X_MSB    0x56
#define BNO055_ACC_OFFSET_X_LSB    0x55
#define BNO055_AXIS_MAP_SIGN       0x42
#define BNO055_AXIS_MAP_CO         0x41
#define BNO055_TEMP_SOURCE         0x40
#define BNO055_SYS_TRIGGER         0x3F
#define BNO055_PWR_MODE            0x3E
#define BNO055_OPR_MODE            0x3D
#define BNO055_UNIT_SEL            0x3B
#define BNO055_SYS_ERR             0x3A
#define BNO055_SYS_STATUS          0x39
#define BNO055_SYS_CLK_STATUS      0x38
#define BNO055_INT_STA             0x37
#define BNO055_ST_RESULT           0x36
#define BNO055_CALIB_STAT          0x35
#define BNO055_TEMP                0x34
#define BNO055_GRV_Data_Z_MSB      0x33
#define BNO055_GRV_Data_Z_LSB      0x32
#define BNO055_GRV_Data_Y_MSB      0x31
#define BNO055_GRV_Data_Y_LSB      0x30
#define BNO055_GRV_Data_X_MSB      0x2F
#define BNO055_GRV_Data_X_LSB      0x2E
#define BNO055_LIA_Data_Z_MBS      0x2D
#define BNO055_LIA_Data_Z_LSB      0x2C
#define BNO055_LIA_Data_Y_MBS      0x2B
#define BNO055_LIA_Data_Y_LSB      0x2A
#define BNO055_LIA_Data_X_MBS      0x29
#define BNO055_LIA_Data_X_LSB      0x28
#define BNO055_QUA_Data_z_MSB      0x27
#define BNO055_QUA_Data_z_LSB      0x26
#define BNO055_QUA_Data_y_MSB      0x25
#define BNO055_QUA_Data_y_LSB      0x24
#define BNO055_QUA_Data_x_MSB      0x23
#define BNO055_QUA_Data_x_LSB      0x22
#define BNO055_QUA_Data_w_MSB      0x21
#define BNO055_QUA_Data_w_LSB      0x20
#define BNO055_EUL_Pitch_MSB       0x1F
#define BNO055_EUL_Pitch_LSB       0x1E
#define BNO055_EUL_Roll_MSB        0x1D
#define BNO055_EUL_Roll_LSB        0x1C
#define BNO055_EUL_Heading_MSB     0x1B
#define BNO055_EUL_Heading_LSB     0x1A
#define BNO055_GYR_DATA_Z_MSB      0x19
#define BNO055_GYR_DATA_Z_LSB      0x18
#define BNO055_GYR_DATA_Y_MSB      0x17
#define BNO055_GYR_DATA_Y_LSB      0x16
#define BNO055_GYR_DATA_X_MSB      0x15
#define BNO055_GYR_DATA_X_LSB      0x14
#define BNO055_MAG_DATA_Z_MSB      0x13
#define BNO055_MAG_DATA_Z_LSB      0x12
#define BNO055_MAG_DATA_Y_MSB      0x11
#define BNO055_MAG_DATA_Y_LSB      0x10
#define BNO055_MAG_DATA_X_MSB      0x0F
#define BNO055_MAG_DATA_X_LSB      0x0E
#define BNO055_ACC_DATA_Z_MSB      0x0D
#define BNO055_ACC_DATA_Z_LSB      0x0C
#define BNO055_ACC_DATA_Y_MSB      0x0B
#define BNO055_ACC_DATA_Y_LSB      0x0A
#define BNO055_ACC_DATA_X_MSB      0x09
#define BNO055_ACC_DATA_X_LSB      0x08
#define BNO055_Page0_Page_ID       0x07
#define BNO055_BL_Rev_ID           0x06
#define BNO055_SW_REV_ID_MSB       0x05
#define BNO055_SW_REV_ID_LSB       0x04
#define BNO055_GYR_ID              0x03
#define BNO055_MAG_ID              0x02
#define BNO055_ACC_ID              0x01
#define BNO055_CHIP_ID             0x00
//-----------------------------------------

//------------------Page1------------------
#define BNO055_GYR_AM_SET          0x1F
#define BNO055_GYR_AM_THRES        0x1E
#define BNO055_GYR_DUR_Z           0x1D
#define BNO055_GYR_HR_Z_SET        0x1C
#define BNO055_GYR_DUR_Y           0x1B
#define BNO055_GYR_HR_Y_SET        0x1A
#define BNO055_GYR_DUR_X           0x19
#define BNO055_GYR_HR_X_SET        0x18
#define BNO055_GYR_INT_SETING      0x17
#define BNO055_ACC_NM_SET          0x16
#define BNO055_ACC_NM_THRE         0x15
#define BNO055_ACC_HG_THRES        0x14
#define BNO055_ACC_HG_DURATION     0x13
#define BNO055_ACC_INT_Settings    0x12
#define BNO055_ACC_AM_THRES        0x11
#define BNO055_INT_EN              0x10
#define BNO055_INT_MSK             0x0F
#define BNO055_GYR_Sleep_Config    0x0D
#define BNO055_ACC_Sleep_Config    0x0C
#define BNO055_GYR_Config_1        0x0B
#define BNO055_GYR_Config_0        0x0A
#define BNO055_MAG_Config          0x09
#define BNO055_ACC_Config          0x08
#define BNO055_Page1_Page_ID       0x07
//-----------------------------------------

//-----------------------------------------
#define BNO055_OPR_MODE_CONFIGMODE      0b0000
#define BNO055_OPR_MODE_ACCONLY         0b0001
#define BNO055_OPR_MODE_MAGONLY         0b0010
#define BNO055_OPR_MODE_GYROONLY        0b0011
#define BNO055_OPR_MODE_ACCMAG          0b0100
#define BNO055_OPR_MODE_ACCGYRO         0b0101
#define BNO055_OPR_MODE_MAGGYRO         0b0110
#define BNO055_OPR_MODE_AMG             0b0111
#define BNO055_OPR_MODE_IMU             0b1000
#define BNO055_OPR_MODE_COMPASS         0b1001
#define BNO055_OPR_MODE_M4G             0b1010
#define BNO055_OPR_MODE_NDOF_FMC_OFF    0b1011
#define BNO055_OPR_MODE_NDOF            0b1100
//19ms required to switch operating modes.
//-----------------------------------------

//-----------------------------------------
#define BNO055_ACC_Config_2G        0b00000000
#define BNO055_ACC_Config_4G        0b00000001
#define BNO055_ACC_Config_8G        0b00000010
#define BNO055_ACC_Config_16G       0b00000011

#define BNO055_ACC_Config_7_81Hz    0b00000000
#define BNO055_ACC_Config_15_63Hz   0b00000100
#define BNO055_ACC_Config_31_25Hz   0b00001000
#define BNO055_ACC_Config_62_5Hz    0b00001100
#define BNO055_ACC_Config_125Hz     0b00010000
#define BNO055_ACC_Config_250Hz     0b00010100
#define BNO055_ACC_Config_500Hz     0b00011000
#define BNO055_ACC_Config_1000Hz    0b00011100

#define BNO055_ACC_Config_Nomal     0b00000000
#define BNO055_ACC_Config_Suspnd    0b00100000
#define BNO055_ACC_Config_LowPwr1   0b01000000
#define BNO055_ACC_Config_Stndby    0b01100000
#define BNO055_ACC_Config_LowPwr2   0b10000000
#define BNO055_ACC_Config_DeepSpd   0b10100000
//-----------------------------------------

//-----------------------------------------
#define BNO055_GYR_Config_0_2000dps     0b00000000
#define BNO055_GYR_Config_0_1000dps     0b00000001
#define BNO055_GYR_Config_0_500dps      0b00000010
#define BNO055_GYR_Config_0_250dps      0b00000011
#define BNO055_GYR_Config_0_125dps      0b00000100
#define BNO055_GYR_Config_0_523Hz       0b00000000
#define BNO055_GYR_Config_0_230Hz       0b00001000
#define BNO055_GYR_Config_0_116Hz       0b00010000
#define BNO055_GYR_Config_0_47Hz        0b00011000
#define BNO055_GYR_Config_0_23Hz        0b00100000
#define BNO055_GYR_Config_0_12Hz        0b00101000
#define BNO055_GYR_Config_0_64Hz        0b00110000
#define BNO055_GYR_Config_0_32Hz        0b00111000

#define BNO055_GYR_Config_1_Normal      0b00000000
#define BNO055_GYR_Config_1_FstPwrUp    0b00000001
#define BNO055_GYR_Config_1_DeepSpd     0b00000010
#define BNO055_GYR_Config_1_Suspnd      0b00000011
#define BNO055_GYR_Config_1_AdvPwrSv    0b00000100
//-----------------------------------------

//-----------------------------------------
#define BNO055_MAG_Config_2Hz       0b00000000
#define BNO055_MAG_Config_6Hz       0b00000001
#define BNO055_MAG_Config_8Hz       0b00000010
#define BNO055_MAG_Config_10Hz      0b00000011
#define BNO055_MAG_Config_15Hz      0b00000100
#define BNO055_MAG_Config_20Hz      0b00000101
#define BNO055_MAG_Config_25Hz      0b00000110
#define BNO055_MAG_Config_30Hz      0b00000111
#define BNO055_MAG_Config_LowPwd    0b00000000
#define BNO055_MAG_Config_Regular   0b00001000
#define BNO055_MAG_Config_EnhcdRglr 0b00010000
#define BNO055_MAG_Config_HiAccrcy  0b00011000
#define BNO055_MAG_Config_Normal    0b00000000
#define BNO055_MAG_Config_Sleep     0b00100000
#define BNO055_MAG_Config_Suspnd    0b01000000
#define BNO055_MAG_Config_ForceMode 0b01100000
//-----------------------------------------

//-----------------------------------------
#define BNO055_UNIT_SEL_ACC_ms2     0b00000000
#define BNO055_UNIT_SEL_ACC_mG      0b00000001

#define BNO055_UNIT_SEL_GYR_Dps     0b00000000
#define BNO055_UNIT_SEL_GYR_Rps     0b00000010

#define BNO055_UNIT_SEL_ANGL_Deg    0b00000000
#define BNO055_UNIT_SEL_ANGL_Rad    0b00000100

#define BNO055_UNIT_SEL_Tmp_C       0b00000000
#define BNO055_UNIT_SEL_Tmp_F       0b00010000

#define BNO055_UNIT_SEL_Fus_Win     0b00000000
#define BNO055_UNIT_SEL_Fus_And     0b10000000

//-------------UNIT SELECT---------------
#define BNO055_UNIT_SEL_ACC BNO055_UNIT_SEL_ACC_ms2
#define BNO055_UNIT_SEL_GYR BNO055_UNIT_SEL_GYR_Dps
#define BNO055_UNIT_SEL_ANGL BNO055_UNIT_SEL_ANGL_Deg
#define BNO055_UNIT_SEL_Tmp BNO055_UNIT_SEL_Tmp_C
#define BNO055_UNIT_SEL_Fus BNO055_UNIT_SEL_Fus_Win
//-----------------------------------------

//-----------------------------------------
#define BNO055_CHIP_ID_DEFAULT 0xa0  //本当は0xA0
//-----------------------------------------

bool BNO055_Initialize_Fusion();
bool BNO055_Initialize_Raw();

void BNO055_SetUnit();

void BNO055_ReadAcc(float*,float*,float*);
void BNO055_ReadLinAcc(float*,float*,float*);
void BNO055_ReadGrvAcc(float*,float*,float*);

void BNO055_ReadMag(float*,float*,float*);

void BNO055_ReadGyr(float*,float*,float*);

void BNO055_ReadAMG(float*,float*,float*,float*,float*,float*,float*,float*,float*);

void BNO055_ReadEul(float*,float*,float*);
void BNO055_ReadQua(float*,float*,float*,float*);

#endif
