/*************************************************************************************************************************
*  camera_imu_firmware.ino
*  Author: Subhransu Mishra, Gowtham Garimella
*  email: subhransu.kumar.mishra@gmail.com, ggarime1@jhu.edu

/******************************Include files ***********************************/
#include <SPI.h>
#include <ros.h>

//standard ROS messages
#include <std_msgs/String.h>
#include <camera_imu_sync_apm/ImuTrigger.h>
#include <ros/time.h>  
#include "MPU6000.h"

/******************************Initialize pins ***********************************/
#define CHIP_SELECT_PRESSURE_SENSOR 40                // for Pressure sensor MS5611
#define CHIP_SELECT_MPU6000 53    // IMU MPU6000 CHIP SELECT

/****************************ROS stuff***********************************************/
ros::NodeHandle  nh;
camera_imu_sync_apm::ImuTrigger g_msg_imu;
ros::Publisher g_pub_imu("/vins/imu", &g_msg_imu);
/*******************************************Setup ***********************************************************/
void setup() {
  // Set baudrate to 1Mbps
  nh.getHardware()->setBaud(1000000);
  //Initialize ros nodehandle. 
  nh.initNode();
  nh.advertise(g_pub_imu);
  // Wait for NodeHandle to connect:
  while(!nh.connected()) {
    nh.spinOnce();
  }
  // Camera_trigger frequency is defined in MPU6000.h
  if (! nh.getParam("camera_trigger_frequency", &camera_trigger_frequency)) {
    camera_trigger_frequency = 20;
  }
  pinMode(CHIP_SELECT_PRESSURE_SENSOR,OUTPUT);
  digitalWrite(CHIP_SELECT_PRESSURE_SENSOR,HIGH);  		//Disable Pressure sensor
  pinMode(CHIP_SELECT_MPU6000,OUTPUT);
  digitalWrite(CHIP_SELECT_MPU6000,HIGH);  	//Disable IMU sensor
  MPU6000_Init(); 				//Initialize Imu
}

/*********************************************loop*************************************************/
void loop() 
{
  if(MPU6000_newdata) {
    cli();
    MPU6000_Read();
    MPU6000_newdata = false;//reset
    if(irq_counter == 0)
      g_msg_imu.camera_trigger = true;
    else
      g_msg_imu.camera_trigger = false;
    sei();
    g_msg_imu.header.stamp = irq_time;
    g_msg_imu.header.frame_id = "imu";
    g_msg_imu.accel.x = gx;
    g_msg_imu.accel.y = gy;
    g_msg_imu.accel.z = gz;  
  
    g_msg_imu.gyro.x = ax;
    g_msg_imu.gyro.y = ay;
    g_msg_imu.gyro.z = az;  
    
    g_pub_imu.publish(&g_msg_imu);
    nh.spinOnce();
  }
  delay(1);
}


