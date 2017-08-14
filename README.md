# Ardrone 2.0

* ## What the project is about

Parrot AR.Drone is a remote controlled flying quadcopter helicopter built by the French company Parrot. The drone is designed to be controlled by mobile or tablet operating systems such as the supported iOS or Android within their respective apps or the unofficial software available for Windows Phone, Samsung BADA and Symbian devices.

The airframe of the AR.Drone, constructed of nylon and carbon fiber parts, measures 57 cm (22 in) across. Two interchangeable hulls were supplied with the airframe, one designed for indoor and one for external flight. The indoor hull is made from EPP foam, and encases the circumference of the blades for protection. The outdoors use hull is made from lightweight plastic, and allows for increased maneuvrability. In total, the AR.Drone has six degrees of freedom, with a miniaturized inertial measurement unit tracking the pitch, roll and yaw for use in stabilisation.

Inside the airframe, a range of sensors assist flight, enabling the interface used by pilots to be simpler, and making advanced flight easier. The onboard computer runs a Linux operating system, and communicates with the pilot through a self-generated Wi-Fi hotspot. The onboard sensors include an ultrasonic altimeter, which is used to provide vertical stabilisation up to 6 m (19 ft 8 in). The rotors are powered by 15 watt, brushless motors powered by an 11.1 Volt lithium polymer battery. This provides approximately 12 minutes of flight time at a speed of 5 m/s (11 mph). Coupled with software on the piloting device, the forward-facing camera allows the drone to build a 3D environment, track objects and drones, and validate shots in augmented reality games.

Our task was the implementation of navigation system for drones, to do it we should have done:
* 	Track a marked object using OpenCV
* 	Build a controller that uses navigation data to set the drones navigation parameters: vertical velocity, and rotation about 3 axis.

Our project based on ROS library and, like any ROS project, it consists of separated files and each of them can be written either in python or in c++. These files call nodes and every node is respond for the part of the work: 

* Controller - makes the drone fly above the circles and follow them, if the target isn't static;
* Computer Vision - detect the target on the camera stream;
* Image Processing - gives all the information about drone status, turn on the frame. 


* ## How to use

All the libraries and soft work only in Ubuntu or Debian. So, firsly install Linux if you don't use it, and than all to let our project compile.

1. Catkin - library to build the ROS project. <br>Documentation - http://wiki.ros.org/catkin. <br>To install execute in command line:

```
sudo apt-get install ros-kinetic-catkin
```

2. ROS - open python/c++  library for robotics. We wrote all code for the "kinetic" version. All the instructions - http://wiki.ros.org/kinetic/Installation/Ubuntu.

3. Ardrone_autonomy - library from the developers of the drone, gives the basic commands to control it - take off, land, hover and so on. <br> Documentation - https://ardrone-autonomy.readthedocs.io/en/latest/installation.html <br> To install just execute in command line:

```
sudo apt-get install ros-kinetic-ardrone-autonomy
```

4. OpenCV – open с++/python library to recognising images, video, objets and stuff like that.<br>
Download - https://sourceforge.net/projects/opencvlibrary/<br>
Documentation - http://docs.opencv.org/3.3.0/<br>

5. Qt library - download the installer on button «Get your open source package» on website https://info.qt.io/download-qt-for-application-development

After that, you need to build the project. Change the directory for that one, where you have "src" folder. In command line execute "catkin_make" in this directory. Now all the changes saved and we can run the project.

To run the project we need to run launch files. It is in the folder named "launch". Now there are 2 files: ardrone_autopilot.launch and enviroment.launch. You need to execute in 2 different windows:

```
roslaunch ardrone_autopilot enviroment.launch
```

```
roslaunch ardrone_autopilot autopilot.launch
```

You'll see the window with all camera stream and two console windows with debug information.<br>

|Buttons: |Info|
|-----|------|
|W, A, S, D | to move the drone yourself;
|T | take off;
|L | land;
|[ or ] | up or down;<br>
|C | change camera;
|M |to turn on the image processing;
|N | autopilot.
|  | <b>Only in go branch:</b>|
|F1| Decrease P coefficient
|F3| Decrease I coefficient
|F5| Decrease D coefficient
|F2| Increase P coefficient
|F4| Increase I coefficient
|F6| Increase D coefficient



* ## Some future recomendations

We've found some interesting libraries, which can make the drone control easier:

* Nodecopter - nodejs library, based on ardrone_autonomy, but with some useful functions, which are nit existing in ardrone_autonomy, easier to use, because it doesn't need many other libraries to work.
<br>Documentation and installation - https://github.com/felixge/node-ar-drone

* tum_ardrone - ROS package that has an already implemented GUI, keyboard controller, camera-based autopilot and PID controller. Also there is a lot of interesting things to work with.
<br>ROS wiki - http://wiki.ros.org/tum_ardrone
<br>GitHub - https://github.com/tum-vision/tum_ardrone


  
