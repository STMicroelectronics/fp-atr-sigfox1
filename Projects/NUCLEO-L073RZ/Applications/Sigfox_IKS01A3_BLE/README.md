## Application Description 

An example application for Asset Tracking using the STM32 Nucleo development board together with X-NUCLEO-S2868A1 or X-NUCLEO-S2868A2 or X-NUCLEO-S2915A1, X-NUCLEO-IDB05A1 or X-NUCLEO-IDB05A2 (optionally), X-NUCLEO-GNSS1A1 and X-NUCLEO-IKS01A2 or X-NUCLEO-IKS01A3 expansion boards is provided in the “Projects” directory. Ready to be built projects are available for multiple IDEs. 

Pre-compiled binary executables are available for both L073RZ and L476RG platforms for Sigfox Radio Control Zone 1.

The user interface is provided via serial port, which needs to be configured with baud rate 115200, 8N1 parameters.

The application collects environmental sensor data (humidity, temperature and pressure) and GNSS position data and sends them over the Sigfox network together in two 7-bytes or 9-bytes Sigfox frames. The last byte is used to indicate the kind of event which triggered the message sending.

The message sending is triggered by either one of the following events:
-	Pressing of the user button
-	Timer expiration. 
-	Threshold crossing events on environmental values
-	Motion detection by the on board accelerometer (wake-up, tilt and orientation detection are possible)

Thresholds can be set using the dedicated ST Asset Tracker mobile app for Android and iOS, and transmitted to the firmware by Bluetooth Low Energy connection. It is also possible to set sensor polling interval and message sending interval, which are set by default to 1 minute and 15 minutes respectively

After the Sigfox message is sent, the STM32 switches to Low-Power state and remains in this state until the next message sending request.

To transmit messages, Sigfox network coverage is needed. Alternatively, a Sigfox Network Emulator Kit can be used.

To see the received data, connect to Sigfox backend (https://backend.sigfox.com) and select the message list for your device. 


## Hardware and Software environment

- This example runs on STM32 Nucleo devices with S2-LP expansion board X-NUCLEO-S2868A1 or X-NUCLEO-S2868A2 or X-NUCLEO-S2915A1, BLE expansion board X-NUCLEO-IDB05A1, GNSS expansion board X-NUCLEO-GNSS1A1 and MEMS sensor board X-NUCLEO-IKS01A2 or X-NUCLEO-IKS01A3

- This example has been tested with STMicroelectronics:
	- NUCLEO-L073RZ RevC board
	- NUCLEO-L476RG RevC board
  and can be easily tailored to any other supported device and development board.

- For Sigfox application demonstration One S2-LP Expansion Board + STM32 Nucleo is programmed as node 

- On the other side, message can be seen on Sigfox Network backend or on a dedicated dashboard

- It operates in all radio control zones.


- Sigfox firmware configurations:

    - Sigfox_IKS01A2     - Configuration to be used with X-NUCLEO-IKS01A2 and without Bluetooth board
    - Sigfox_IKS01A2_BLE - Configuration to be used with X-NUCLEO-IKS01A2 and with Bluetooth board
    - Sigfox_IKS01A3     - Configuration to be used with X-NUCLEO-IKS01A3 and without Bluetooth board
    - Sigfox_IKS01A3_BLE - Configuration to be used with X-NUCLEO-IKS01A3 and with Bluetooth board

- Flash the Node (Nucleo Board mounted with X-NUCLEO-S2868A1 or X-NUCLEO-S2868A2 or X-NUCLEO-S2915A1, X-NUCLEO-IDB05A1 or X-NUCLEO-IDB05A2, X-NUCLEO-GNSS1A1 and X-NUCLEO-IKS01A2 or X-NUCLEO-IKS01A3) as explained.


## How to use it ? 

### Application build and flash

  - Open and build the project with one of the supported development toolchains (see the release note
    for detailed information about the version requirements).
	
  - Program the firmware on the STM32 board: you can copy (or drag and drop) the generated ELF
    file to the USB mass storage location created when you plug the STM32 
    board to your PC. If the host is a Linux PC, the STM32 device can be found in 
    the /media folder with the name e.g. "DIS_L4IOT". For example, if the created mass 
    storage location is "/media/DIS_L4IOT", then the command to program the board 
    with a binary file named "my_firmware.bin" is simply: cp my_firmware.bin 
    /media/DIS_L4IOT. 

   Alternatively, you can program the STM32 board directly through one of the 
   supported development toolchains.
 

### Application first launch

  - Connect the board to your development PC through USB (ST-LINK USB port).
  
  - Open the console through serial terminal emulator (e.g. TeraTerm), select the ST-LINK COM port of your
    board and configure it with 8N1, 115200 bauds, no HW flow control;


### Application runtime 

  - After application startup, Sigfox messages can be sent carrying environmental sensor data.
  
  - The message sending is triggered by either one of the following events:
	-	Pressing of the user button
	-	Timer expiration. By default, the timer is set to 15 minutes but it can be changed in the source code.
	-	Threshold crossing events on environmental values
	-	Motion detection by the on board accelerometer (wake-up, tilt and orientation detection)

### Author

SRA Application Team

### License

Copyright (c) 2022 STMicroelectronics.
All rights reserved.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.
