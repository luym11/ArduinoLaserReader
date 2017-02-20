# ArduinoLaserReader
Read distance information from SICK sensor

##Sources
用http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World里的代码，注意波特率和数据类型就好   
##Solving "Permission Denied"
    
	$ sudo usermod -a -G dialout <username>
    	$ sudo chmod a+rw /dev/ttyACM0

##Booting
1. roscore
2. rosrun rosserial_python serial_node.py /dev/ttyUSB0  
3. rostopic echo SICK_sensor
4. Then we can use rosbag to record data, rqt to plot
