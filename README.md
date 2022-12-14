## Requirements

- Ubuntu 18.04
- ROS Melodic
- Python3

## Description 

Node used to record sound from a device using a rospy node. Work with alsaaudio lib in python3.

## Dependencies

-To use this lib, you need to have ROS installed in your computer (http://wiki.ros.org/fr/ROS/Installation).
-In order to get the alsaaudio you need to download the package ```python3-alsaaudio```:
 ```bash
 sudo apt-get install python3-alsaaudio
 ```

## Installation



## Use 

You can change the Id of your device(s) in the launch file (record_launch.launch). Once done, start the nodes by executing the launch file:
```bash
roslaunch record_launch.launch identity:="folderName"
```
The default ```format``` extension is mp3, but you can change it in the launch file with the parameter ```format```.

The default ```microphone_rate``` is 8000 Hz, but it can be changed for the specific value of the specific microphone used. This can be done by changing the parameter ```microphone_rate```.

To check your microphone rate you can do the following command in the terminal:

 ```bash
 aplay -l # in order to see the desired microphone
 arecord -D hw:<card number>,<device>
 ```


To start recording, publish somethig in the service ```/record_audio/start```.
To stop recording, publish something in the service ```/record_audio/stop```.
