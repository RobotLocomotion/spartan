# Schunk Driving

The Schunk power cable is a little barrel jack behind the robot. Plug it in when you start, unplug it at the end of the day.

The Schunk control page lives at http://192.170.10.20/ (or whatever the IP of your schunk is). Note the super-useful status panel at the right. If you find that you can't control it, pressing "STOP" and then "ACK" might help.

To control the Schunk from code, your best bet is to use `spartan/modules/spartan/manipulation/schunk_driver.py`. Import and use it in a Python ROS node, and you should be good to go.

