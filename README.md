# camera_imu_sync_apm
Provide firmware to create a VINS sensor using an APM2.5 and a pointgrey camera.
=======
Provide firmware to create a VINS sensor using an APM2.5 and a pointgrey camera. The package contains apm firmware to read IMU data and trigger camera synchronously.

# Installation
This package depends on the following packages:
- pointgrey_camera_driver (Download from [here](https://github.com/jhu-asco/pointgrey_camera_driver). Also if necessary, checkout the specific [SHA used](https://github.com/jhu-asco/pointgrey_camera_driver/commit/50839d57648670dee813ef0e82ef12bc64193de5))
- rosserial_node
- rosserial_python

Once the dependencies have been installed, follow these steps
- Build the package using `catkin build`
- Copy the `camera_imu_firmware` sketch to your arduino sketchbook
- Rebuild ros messages in your libraries so as to include the messages from this package ( [Follow this tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup))
- Flash the firmware to Arduino.

# Setting Camera
Connect GPIO0 to A0 pin on APM2.5. Also connect ground of camera to ground.
![Setup](http://ardupilot.org/copter/_images/apm2_analog_pins2.jpg)

(Picture credit: *http://ardupilot.org/copter/docs/common-apm25-and-26-overview.html*)

# Running computer side
Connect the APM board and the camera to the computer. Then run the launch file provided:
```
roslaunch camera_imu_sync_apm imu_camera_combined.launch
```

# Known Bugs
Sometimes the camera does not respond when the launch file is started. The known workaround for now is to start reconfigure, and disable and re-enable `external_trigger`. This only happens occasionally so not sure what the reason is.
