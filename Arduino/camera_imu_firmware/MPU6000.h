// MPU6000 support for AruduIMU V3
#ifndef MPU6000_H
#define MPU6000_H
#include <SPI.h>

#define MPU6000_CHIP_SELECT_PIN 53  // MPU6000 CHIP SELECT

///////////////////////////////////////////////////////////////////////
////////////////////// MPU 6000 registers//////////////////////////////
/////////////////////////////////////////////////////////////////////////

//refer the datasheet RS-MPU-6000-A-00 for details of registers

////stores device id//default value:0x68 ( bit 0 and 7 are reserved, hardcoded to 0) 
//upper 6 bits of the 7-bit 12c address. The lsb of i2c address is determined by value of AD0 pin
#define MPUREG_WHOAMI 0x75 

// Sampling rate divider 
//This controls the rate at which the registers are refreshed, but the user facing registers are updated when the serial is idle
//Sample Rate = Gyroscope Output Rate/1 + SMPLRT_DIV)
//gyro output rate is 8Khz when the DLPF  is diabled and 1Khz when it is enabled
//DLPF_CFG is the digital low pass filter acting on the gyro reading
#define  MPUREG_SMPLRT_DIV 0x19 

//MPU6000 configuration
//This register configures the external Frame Synchronization (FSYNC) pin sampling (bit3:5)
//and the Digital Low Pass Filter (DLPF) setting DLPF_CFG(bit0:2) for both the gyroscopes and accelerometers.
//THE FSYNC is an input pin which can be used to synchronize the data with camera/ other external devices
//EXT_SYNC_SET should be configured to use the FSYNC feature. On ardu1mu v3 this feature cannot be used as there is no connection.
#define MPUREG_CONFIG 0x1A //

//gyroscope configuration
//it is used to trigger gyroscope self-test(bit5:7 x,y z respectively) and configure the gyroscopes full scale range(bit3:4)
//Gyroscope self-test permits users to test the mechanical and electrical portions of the gyroscope
//FS_SEL(bit3:4)= 0(+-250deg/sec) ,1(+-500deg/sec) ,2(+-1000deg/sec), 3(+-2000deg/sec)
//lets say a car has turning radius 5 mts (the circle is 10mts) and it is turning at a very high speed(driver experiences 1g lateral acceleration)
//Hence the limiting velocity is about 0.5 rads/sec or about 30 rads/sec. So it is very safe to select FS_SEL=0;
#define MPUREG_GYRO_CONFIG 0x1B

//accelerometer configuration
//This register is used to trigger accelerometer self test (XA_ST,YA_ST,ZA_ST)(bit5:7) and configure the accelerometer full scale range (AFS_SEL)(bit3:4).
//This register also configures the Digital High Pass Filter (DHPF) , AACCEL_HPF bit0:2)
//AFS_SEL(bit3:4)= 0(+-2g) ,1(+-4g) ,2(+-8g), 3(+-16g)
//car experiences 1 g vertically downward and can experience 1 g lateral acceleration while turning so it might be safe to select AFS_SEL=1;
//ACCEL_HPF:0(Reset),1(5hz),2(2.5hz), 3(1.25Hz), 4(0.63hz), 5(no filtering )
//For now we would want to have no filtering
#define MPUREG_ACCEL_CONFIG 0x1C

//Interrupt pin behavior/Bypass Enable Configuration
//This register configures the behavior of the interrupt signals at the INT pins.
#define MPUREG_INT_PIN_CFG 0x37

//Interrupt enable
//This register enables interrupt generation by interrupt sources.
#define	MPUREG_INT_ENABLE 0x38 

//Accelerometer Measurement(register type: read only)
//value in 2-complement
//sensitivity AFS_SEL(0,1,2,3): 16384 LSB/mg , 8192 LSB/mg, 4096 LSB/mg, 2048 LSB/mg
//AFS_SEL(bit3:4)= 0(+-2g) ,1(+-4g) ,2(+-8g), 3(+-16g)
#define MPUREG_ACCEL_XOUT_H 0x3B
#define MPUREG_ACCEL_XOUT_L 0x3C
#define MPUREG_ACCEL_YOUT_H 0x3D
#define MPUREG_ACCEL_YOUT_L 0x3E 
#define MPUREG_ACCEL_ZOUT_H 0x3F 
#define MPUREG_ACCEL_ZOUT_L 0x40 

//Temperature measurement (register type: read only)
//16 bit signed value
//The scale factor and offset for the temperature sensor are found in the Electrical Specifications table
//scale:340 LSB/celsius, offset: -521 LSB 
#define MPUREG_TEMP_OUT_H 0x41//
#define MPUREG_TEMP_OUT_L 0x42//

//Gyroscope Measurement(register type: read only)
//value in 2-complement
//FS_SEL(0,1,2,3):(131, 65.5, 32.8, 16.4 LSB/deg/sec )
#define MPUREG_GYRO_XOUT_H 0x43 // 
#define	MPUREG_GYRO_XOUT_L 0x44 //
#define MPUREG_GYRO_YOUT_H 0x45 //
#define	MPUREG_GYRO_YOUT_L 0x46 //
#define MPUREG_GYRO_ZOUT_H 0x47 //
#define	MPUREG_GYRO_ZOUT_L 0x48 //

//user control (read write)
//This register allows the user to enable and disable the FIFO buffer, I2C Master Mode, and primary I2C interface
#define MPUREG_USER_CTRL 0x6A //

//MPU Power management (read write)
//bit7=Device reset, bit6=Sleep , bit5=Cycle, bit3=temp_dis, bit0:2=clock select
//This register allows the user to configure the power mode and clock source
//It also provides a bit for resetting the entire device, and a bit for disabling the temperature sensor.
//Device_reset:When set to 1, this bit resets all internal registers to their default values.
//The reset bit automatically clears to 0 once the reset is done.
//Default of clock select is 0(internal 8khz) but it is recommended to use something else
#define	MPUREG_PWR_MGMT_1 0x6B 

//This register allows the user to configure the frequency of wake-ups in Accelerometer Only Low Power Mode.
//This register also allows the user to put individual axes of the accelerometer and gyroscope into standby mode.
#define	MPUREG_PWR_MGMT_2 0x6C

///////////////////////////////////////////////////////////////////////////////
///////////////////Configuration bits  MPU 6000////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//PWR_MGMT_1:bit6. When this bit is 1 this puts the device into sleep
#define BIT_SLEEP                   0x40

//PWR_MGMT_1:bit7. When this bit is set to 1, this bit resets all internal registers to their default values and sets itself back to 0 when done
#define BIT_H_RESET                 0x80

//PWR_MGMT_1:bit0:2 
#define BITS_CLKSEL                 0x07//Stops the clock and keeps the timing generator in reset
#define MPU_CLK_SEL_PLLGYROX        0x01//PLL with X axis gyroscope reference
#define MPU_CLK_SEL_PLLGYROZ        0x03//PLL with Z axis gyroscope reference

//
#define MPU_EXT_SYNC_GYROX          0x02

//GYRO_CONFIG:bit3:4
//full scale of gyro output
//FS_SEL(0,1,2,3):(131, 65.5, 32.8, 16.4 LSB/deg/sec )
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x00// we want max of 250DPS

//CONFIG DLPF_CFG (bits 0:2)

#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00//bandwidth=256Hz and no low pass filter, delay=0.98 ms
#define BITS_DLPF_CFG_188HZ         0x01//bandwidth=188Hz , delay=1.9 ms
#define BITS_DLPF_CFG_98HZ          0x02//bandwidth=98Hz  , delay=2.8 ms
#define BITS_DLPF_CFG_42HZ          0x03//bandwidth=42Hz  , delay=4.8 ms
#define BITS_DLPF_CFG_20HZ          0x04//bandwidth=20Hz  , delay=8.3 ms
#define BITS_DLPF_CFG_10HZ          0x05//bandwidth=10Hz  , delay=13.4 ms
#define BITS_DLPF_CFG_5HZ           0x06//bandwidth=5Hz   , delay=18.6 ms
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07//bandwidth=2100Hz and no low pass filter
#define BITS_DLPF_CFG_MASK          0x07//No filter, Fs=8Mhz

//INT_PIN_CFG INT_RD_CLEAR (bit 4)
//When this bit is equal to 0, interrupt status bits are cleared only by reading INT_STATUS (Register 58)
//When this bit is equal to 1, interrupt status bits are cleared on any read operation.
#define	BIT_INT_ANYRD_2CLEAR	    0x10

//INT_ENABLE DATA_RDY_EN(bit0)
//When set to 1, this bit enables the Data Ready interrupt, 
//which occurs eachtime a write operation to all of the sensor registers has been completed.
#define	BIT_RAW_RDY_EN		    0x01

//MPUREG_USER_CTRL I2C_IF_DIS(bit4)
//When set to 1, this bit disables the primary I2C interface and enables the SPI interface instead.
#define	BIT_I2C_IF_DIS              0x10

// Pin where CAMERA is triggered
#define CAMERA_TRIGGER_PIN 54
// Divider for camera data
#define CAMERA_TRIGGER_FREQUENCY 20

// Sample rate
#define BITS_IMU_SAMPLE_RATE 39 // The actual sample rate 8/(rate + 1)Hz if DLPF is not enabled. Otherwise 1/(rate + 1) Hz

///////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////global variables///////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
volatile bool MPU6000_newdata = false; // Flag that new data has been received
ros::Time irq_time; // Time when the last message was received
int irq_counter = 0; // Counter for triggering Camera
int camera_trigger_start; // When to start triggering camera based on number of IMU messages passed
int camera_trigger_stop; // When to stop triggering. Usually half the number of trigger start messages.

//Sensor variables
float ax=0.0, ay=0.0, az=0.0; // 9.80665/(2^15/4) 8192 LSB/g -max of 4g 1g=9.80665m/s2
float gx=0.0, gy=0.0, gz=0.0; //pi/(180*(2^15/250)) 131 LSB/DPS - max of 250DPS
//unsigned long t_us_imu_now, t_us_imu_prev; //This is updated with micros() when read is called
float temp=0.0,  tempScale_f=1.0;//0.00294118;//340 LSB/celsius, offset: -521 LSB 

//The scales and offsets are set in MPU6000_Init()
float  aScale_f=1.0f, gScale_f=1.0f;

// Gyro offsets
int gxOffset_i=0,gyOffset_i=0,gzOffset_i=0;

#endif
