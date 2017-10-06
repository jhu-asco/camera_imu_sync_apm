#include <SPI.h>
// MPU6000 SPI functions
byte MPU6000_SPI_read(byte reg)
{
  byte dump;
  byte return_value;
  byte addr = reg | 0x80; // Set most significant bit
  digitalWrite(MPU6000_CHIP_SELECT_PIN, LOW);
  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);
  digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);
  return(return_value);
}

void MPU6000_SPI_write(byte reg, byte data)
{
  byte dump;
  digitalWrite(MPU6000_CHIP_SELECT_PIN, LOW);
  dump = SPI.transfer(reg);
  dump = SPI.transfer(data);
  digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);//when the slave select pin is high, it ignores the master
}

// MPU6000 INTERRUPT ON INT 0
void MPU6000_data_int()
{
  MPU6000_newdata = true;
  irq_time = nh.now();
  ++irq_counter;
  if(irq_counter == camera_trigger_start) {
    digitalWrite(CAMERA_TRIGGER_PIN, LOW); 
    irq_counter = 0;
  }
  else if(irq_counter == camera_trigger_stop) {
    digitalWrite(CAMERA_TRIGGER_PIN, HIGH);
  }
}

// MPU6000 Initialization and configuration
void MPU6000_Init(void)
{
    pinMode(CAMERA_TRIGGER_PIN, OUTPUT);
    digitalWrite(CAMERA_TRIGGER_PIN, HIGH);
    
    // MPU6000 chip select setup
    pinMode(MPU6000_CHIP_SELECT_PIN, OUTPUT);
    digitalWrite(MPU6000_CHIP_SELECT_PIN, HIGH);
    
    // SPI initialization
    SPI.begin();
    delay(1);
    
    // Chip reset
    MPU6000_SPI_write(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    delay(100);
    
    // Wake up device and select GyroZ clock (better performance)
    MPU6000_SPI_write(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delay(1);
    
    // Disable I2C bus (recommended on datasheet)
    MPU6000_SPI_write(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
    delay(1);
    
    // SAMPLE RATE
    MPU6000_SPI_write(MPUREG_SMPLRT_DIV,BITS_IMU_SAMPLE_RATE);     // Sample rate = 200Hz    Fsample= 8Khz/(39+1) = 200Hz
    delay(1);
    
    
    MPU6000_SPI_write(MPUREG_CONFIG, BITS_DLPF_CFG_256HZ_NOLPF2);//bandwidth=256Hz and no low pass filter, delay=0.98 ms, Fs=8Khz
    delay(1);

    // Set the camera trigger value based on sampling frequency. If DLPF is enabled, use 1Khz
    camera_trigger_start = 8000/((BITS_IMU_SAMPLE_RATE + 1)*(camera_trigger_frequency));
    camera_trigger_stop = 0.5 * camera_trigger_start;
    
    
    //MPU6000_SPI_write(MPUREG_GYRO_CONFIG,BITS_FS_2000DPS);  // Gyro scale 2000º/s 16.4 LSB/º/s
    MPU6000_SPI_write(MPUREG_GYRO_CONFIG,BITS_FS_250DPS);  // Gyro scale 250º/s i.e. 131 LSB/º/s
    delay(1);
    
    MPU6000_SPI_write(MPUREG_ACCEL_CONFIG,0x08);            // Accel scale 4g (8192LSB/mg)
    delay(1);   
    
    // INT CFG => Interrupt on Data Ready
    MPU6000_SPI_write(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);         // INT: Raw data ready
    delay(1);
    
    MPU6000_SPI_write(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR);  // INT: Clear on any read
    delay(1);
    
    // MPU_INT is connected to INT 0. Enable interrupt on INT0
    attachInterrupt(6,MPU6000_data_int,RISING);
    
    //finding the ZRO of the gyro
    float gxOffset_f=0.0f,gyOffset_f=0.0f,gzOffset_f=0.0f;
    // Average for 2 seconds
    for(int i=0;i<2000;i++)
    {
      if(MPU6000_newdata) {
        cli();
        MPU6000_Read();
        MPU6000_newdata = false;
        sei();
        gxOffset_f = (gxOffset_f*i +gx)/(i+1);
        gyOffset_f = (gyOffset_f*i +gy)/(i+1);
        gzOffset_f = (gzOffset_f*i +gz)/(i+1);
      }
      delay(1);
    }
     gxOffset_i= int(-gxOffset_f);
     gyOffset_i= int(-gyOffset_f);
     gzOffset_i= int(-gzOffset_f);
     
     aScale_f=0.00119710083;//in m/sec^2
     gScale_f=0.000133158;//rads/sec
}

// Read gyros and accel sensors on MPU6000
void MPU6000_Read()
{
  int byte_H;
  int byte_L;

  int reading_i;

  // Read AccelX
  byte_H = MPU6000_SPI_read(MPUREG_ACCEL_XOUT_H);
  byte_L = MPU6000_SPI_read(MPUREG_ACCEL_XOUT_L);
  reading_i = (byte_H<<8)| byte_L;
  ax = float(reading_i)*aScale_f;

  // Read AccelY
  byte_H = MPU6000_SPI_read(MPUREG_ACCEL_YOUT_H);
  byte_L = MPU6000_SPI_read(MPUREG_ACCEL_YOUT_L);
  reading_i = (byte_H<<8)| byte_L;
  ay = float(reading_i)*aScale_f;

  // Read AccelZ
  byte_H = MPU6000_SPI_read(MPUREG_ACCEL_ZOUT_H);
  byte_L = MPU6000_SPI_read(MPUREG_ACCEL_ZOUT_L);
  reading_i = (byte_H<<8)| byte_L;
  az = float(reading_i)*aScale_f;


  // Read GyroX
  byte_H = MPU6000_SPI_read(MPUREG_GYRO_XOUT_H);
  byte_L = MPU6000_SPI_read(MPUREG_GYRO_XOUT_L);
  reading_i = (byte_H<<8)| byte_L;
  gx = float(reading_i + gxOffset_i)*gScale_f;

  // Read GyroY
  byte_H = MPU6000_SPI_read(MPUREG_GYRO_YOUT_H);
  byte_L = MPU6000_SPI_read(MPUREG_GYRO_YOUT_L);
  reading_i = (byte_H<<8)| byte_L;
  gy = float(reading_i + gyOffset_i)*gScale_f;

  // Read GyroZ
  byte_H = MPU6000_SPI_read(MPUREG_GYRO_ZOUT_H);
  byte_L = MPU6000_SPI_read(MPUREG_GYRO_ZOUT_L);
  reading_i = (byte_H<<8)| byte_L;
  gz = float(reading_i + gzOffset_i)*gScale_f;
}
