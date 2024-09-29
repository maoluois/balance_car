#ifndef __MPU6050_H
#define __MPU6050_H
//#include "mpuiic.h"   												  	  
#include "stm32f1xx_hal.h"
#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t
//#define MPU_ACCEL_OFFS_REG		0X06	// accel_offs register, not mentioned in the datasheet
//#define MPU_PROD_ID_REG			0X0C	// prod id register, not mentioned in the datasheet
#define MPU_SELF_TESTX_REG		0X0D	// self-test register X
#define MPU_SELF_TESTY_REG		0X0E	// self-test register Y
#define MPU_SELF_TESTZ_REG		0X0F	// self-test register Z
#define MPU_SELF_TESTA_REG		0X10	// self-test register A
#define MPU_SAMPLE_RATE_REG		0X19	// sample rate divider
#define MPU_CFG_REG				0X1A	// configuration register
#define MPU_GYRO_CFG_REG		0X1B	// gyroscope configuration register
#define MPU_ACCEL_CFG_REG		0X1C	// accelerometer configuration register
#define MPU_MOTION_DET_REG		0X1F	// motion detection threshold register
#define MPU_FIFO_EN_REG			0X23	// FIFO enable register
#define MPU_I2CMST_CTRL_REG		0X24	// I2C master control register
#define MPU_I2CSLV0_ADDR_REG	0X25	// I2C slave 0 address register
#define MPU_I2CSLV0_REG			0X26	// I2C slave 0 data register
#define MPU_I2CSLV0_CTRL_REG	0X27	// I2C slave 0 control register
#define MPU_I2CSLV1_ADDR_REG	0X28	// I2C slave 1 address register
#define MPU_I2CSLV1_REG			0X29	// I2C slave 1 data register
#define MPU_I2CSLV1_CTRL_REG	0X2A	// I2C slave 1 control register
#define MPU_I2CSLV2_ADDR_REG	0X2B	// I2C slave 2 address register
#define MPU_I2CSLV2_REG			0X2C	// I2C slave 2 data register
#define MPU_I2CSLV2_CTRL_REG	0X2D	// I2C slave 2 control register
#define MPU_I2CSLV3_ADDR_REG	0X2E	// I2C slave 3 address register
#define MPU_I2CSLV3_REG			0X2F	// I2C slave 3 data register
#define MPU_I2CSLV3_CTRL_REG	0X30	// I2C slave 3 control register
#define MPU_I2CSLV4_ADDR_REG	0X31	// I2C slave 4 address register
#define MPU_I2CSLV4_REG			0X32	// I2C slave 4 data register
#define MPU_I2CSLV4_DO_REG		0X33	// I2C slave 4 data out register
#define MPU_I2CSLV4_CTRL_REG	0X34	// I2C slave 4 control register
#define MPU_I2CSLV4_DI_REG		0X35	// I2C slave 4 data in register

#define MPU_I2CMST_STA_REG		0X36	// I2C master status register
#define MPU_INTBP_CFG_REG		0X37	// interrupt/bypass configuration register
#define MPU_INT_EN_REG			0X38	// interrupt enable register
#define MPU_INT_STA_REG			0X3A	// interrupt status register

#define MPU_ACCEL_XOUTH_REG		0X3B	// accelerometer X-axis high byte register
#define MPU_ACCEL_XOUTL_REG		0X3C	// accelerometer X-axis low byte register
#define MPU_ACCEL_YOUTH_REG		0X3D	// accelerometer Y-axis high byte register
#define MPU_ACCEL_YOUTL_REG		0X3E	// accelerometer Y-axis low byte register
#define MPU_ACCEL_ZOUTH_REG		0X3F	// accelerometer Z-axis high byte register
#define MPU_ACCEL_ZOUTL_REG		0X40	// accelerometer Z-axis low byte register

#define MPU_TEMP_OUTH_REG		0X41	// temperature high byte register
#define MPU_TEMP_OUTL_REG		0X42	// temperature low byte register

#define MPU_GYRO_XOUTH_REG		0X43	// gyroscope X-axis high byte register
#define MPU_GYRO_XOUTL_REG		0X44	// gyroscope X-axis low byte register
#define MPU_GYRO_YOUTH_REG		0X45	// gyroscope Y-axis high byte register
#define MPU_GYRO_YOUTL_REG		0X46	// gyroscope Y-axis low byte register
#define MPU_GYRO_ZOUTH_REG		0X47	// gyroscope Z-axis high byte register
#define MPU_GYRO_ZOUTL_REG		0X48	// gyroscope Z-axis low byte register

#define MPU_I2CSLV0_DO_REG		0X63	// I2C slave 0 data out register
#define MPU_I2CSLV1_DO_REG		0X64	// I2C slave 1 data out register
#define MPU_I2CSLV2_DO_REG		0X65	// I2C slave 2 data out register
#define MPU_I2CSLV3_DO_REG		0X66	// I2C slave 3 data out register

#define MPU_I2CMST_DELAY_REG	0X67	// I2C master delay control register
#define MPU_SIGPATH_RST_REG		0X68	// signal path reset register
#define MPU_MDETECT_CTRL_REG	0X69	// motion detection control register
#define MPU_USER_CTRL_REG		0X6A	// user control register
#define MPU_PWR_MGMT1_REG		0X6B	// power management register 1
#define MPU_PWR_MGMT2_REG		0X6C	// power management register 2
#define MPU_FIFO_CNTH_REG		0X72	// FIFO count high byte register
#define MPU_FIFO_CNTL_REG		0X73	// FIFO count low byte register
#define MPU_FIFO_RW_REG			0X74	// FIFO read/write register
#define MPU_DEVICE_ID_REG		0X75	// device ID register

// When AD0 is connected to GND, I2C address is 0X68 (LSB is 0).
// When connected to V3.3, I2C address is 0X69 (LSB is 1).
#define MPU_ADDR				0X68

// Since the module's AD0 is default to GND, the read/write address is 0XD1/0XD0 (if connected to VCC, it is 0XD3/0XD2)
#define MPU_READ    0XD1
#define MPU_WRITE   0XD0

u8 MPU_Init(void); 								// Initialize MPU6050
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf); // I2C write multiple bytes
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf);  // I2C read multiple bytes
u8 MPU_Write_Byte(u8 reg,u8 data);				// I2C write one byte
u8 MPU_Read_Byte(u8 reg);						// I2C read one byte

u8 MPU_Set_Gyro_Fsr(u8 fsr);					// Set gyroscope full-scale range
u8 MPU_Set_Accel_Fsr(u8 fsr);					// Set accelerometer full-scale range
u8 MPU_Set_LPF(u16 lpf);						// Set low-pass filter
u8 MPU_Set_Rate(u16 rate);						// Set sample rate
u8 MPU_Set_Fifo(u8 sens);						// Set FIFO

short MPU_Get_Temperature(void);				// Get temperature
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz); // Get gyroscope values
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az); // Get accelerometer values

#endif