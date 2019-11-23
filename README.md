# hack_bot

Summer 2019 intern hack day project: turf painting robot

Demo: GPS Waypoint Navigation and a modified Husky are used to trace the Clearpath logo on the grass


![Alt text](img/hack_cp_logo.png?raw=true "")

All changes made to existing software for this project are listed below.

## Setup

### udev
```
# echo 'SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="00[0-f][0-f]", MODE="0666", SYMLINK+="arduino arduino_$attr{serial}", GROUP="dialout",' > 97-arduino.rules
```
```
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

### rosserial
```rosserial_python``` was installed on the Husky to communicate with the Arduino and relay board. The following lines were added to ```cpr_gps_navigation/cpr_gps_localization/launch/sensors.launch```.
```
<node pkg="rosserial_python" type="serial_node.py" name="serial_node">
    <param name="port" value="/dev/arduino"/>
    <param name="baud" value="9600"/>
</node>
```

### Services
The ```paint_start``` and ```paint_stop``` services called at each goal point were added to ```cpr_gps_navigation/cpr_gps_tasks/scripts```.

### OCU
```MissionServer::findClosestViapoint``` was disabled for this application.

## Extensions
* Temporarily disable painting while turning
* Extend image processing
