# mbed_exam2

### 1. How to set up our program
##### (1) move to the directory
  `cd ~/ee2405/mbed_exam2/src/model_deploy`
##### (2) compile the main.cpp
  `sudo mbed compile --source . --source ~/ee2405new/mbed-os-build2/ -m B_L4S5I_IOT01A -t GCC_ARM --profile tflite.json -f`
##### (3) Open the screen 
  `sudo screen /dev/ttyACM0`
##### (4) Open another terminal
##### (5) move to the directory
  `cd ~/ee2405/mbed_exam2/src/model_deploy/wifi_mqtt`
##### (6) compile the mqtt_client.py
  `sudo python3 mqtt_client.py`

### 2. What are the results
##### (1) Compile commend
  `sudo mbed compile --source . --source ~/ee2405new/mbed-os-build2/ -m B_L4S5I_IOT01A -t GCC_ARM --profile tflite.json -f`
##### (2) After compile, open the screen with the commend 
  `sudo screen /dev/ttyACM0`
##### (3) When we type /g/run on the screen, RPC call the gesture_UI and gesture_detect and LED1 will turn on
##### (4) The gesture_UI sense a gesture, the gobal variable shape will be true. 
##### (5) If shape is false, the gesture_detect function will keep calculate and restore the data.
##### (6) If shape is true, which means TF sense a gesture, then the gessture_detect function will call the confirm_shape function to determine which the gesture is.
