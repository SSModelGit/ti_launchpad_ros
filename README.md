# Connecting a TI LaunchPad to a joystick via ROS
## Introduction
### Purpose and Overview
This was a project developed for the Blinn ENGR 1201 class, with the purpose of utilizing a joystick to control a robot based off the [DSTR platform](http://tstar.us/products/dstr-robot/) driven by a [TI SimpleLink™ Wi-Fi® CC3200 LaunchPad™](http://www.ti.com/tool/cc3200-launchxl). Although other mechanisms for connecting joysticks to boards with wifi shields do exist, [ROS](www.ros.org) was used for its scalability. Currently, the repository contains a general purpose ROS node for connecting to CC3200 LaunchPads with WiFi connectivity - however, this node is usable for all boards that communicate via a WiFi shield and are setup to use UDP socket communication. The repo also contains the drive code for the robot itself, split into two parts, one board control of the robot or two board control, and is further split into drive code for the arm, and drive code for the base.

### Reason for using ROS
ROS is a popular sensor integration and robot control tool that allows for multiple sensors, control algorithms, and microcontrollers/microcomputers to communicate with each other via nodes. Each node can publish and subscribe to several topics, allowing multiple nodes to be communicating and interacting with the same data. By setting up a node that can interact with the LaunchPad, a relay point between the ROS master node and the board is established, allowing for communication between nodes on the computer (including the joy_node) and the board. This opens doors for further integrating sensors, handling more complex control algorithms, controlling multiple boards in an integratable and stable fashion, etc.

### Mechanism

The method of connecting ROS to the LaunchPad was accomplished via UDP sockets - a node on a computer connects to a preset IP address (available as a parameter) and sends information to the preset port (available as a parameter) as a client. The LaunchPad sets up WiFi connections via the shield at the set IP, a UDP socket server at the set port, and waits to recieve information. Within the ROS node, information is sent to the board from within the callbacks, ex. after information from the joy_node is recieved in the subscriber callback, the information is processed and then sent to the board within the same callback routine.

## Getting started
### Prerequisites
Running ROS requires a Linux computer, or a VM running Linux. Compiling and loading the code to the LaunchPad requires the Energia IDE and the board associated drivers. The board itself must come equipped with WiFi capability integrated, or a WiFi shield connected.

### Installing
The following instructions work on Ubuntu 16.04, and forks of this version (ex. Ubuntu Mate 16.04, etc). Copy the contents of ```ubuntu_setup.sh``` into your favorite editor, and save as ```~/ti_ros.sh```. Go to terminal, and run the command ```sh ~/ti_ros.sh```. This will install both ROS Kinetic and ModROS on your system - be wary that depending on the CPU capacity of the machine, not all of the commands of the script will be able successfully execute within one script. Close all open terminals after completing the installation.

## Running the examples
### Compiling the LaunchPad code
Open the LaunchPad files (ending in .ino) in Energia, and upload them to the board. Ensure that the port is well connected - sometimes the port fails to open the first time, and simply requires another attempt at uploading.

### Compiling and executing the ROS node
The setup script would have already compiled the ROS node. If instead the repository was downloaded manually, make sure that the ROS code (in the folder txi_launchpad) is copied into the src folder of the catkin workspace, (into ~/catkin_ws/src/txi_launchpad). Once that is done, run ```catkin_make install```, and then ```catkin_make``` in the terminal to compile the code.

To run the node, first ensure that the board is powered, and the computer is on the same local network as the board. Ensure that the first three digits of the IP of the computer matches that of the board (ex. ```192.168.1.1``` and ```192.168.1.2```). If necessary, change the IP of the computer via ```sudo ifconfig [wifi driver's name] [desired IP]```. To run the node, there are two options: manually setup the nodes and respective parameters, or use the provided launch files. If using two boards to control the robot, use the command ```roslaunch txi_launchpad two_boards.launch```. If using one board, run ```roslaunch cc3200_joy.launch```. Before running the launch file, check the respective launch file's code for the set parameter values for the IP and port numbers - check that the board IP each node points to matches the respective board, and that the port each node points to matches the port the board is listening on.

The nodes are mapped to the Xbox 360 wired controller, and have a 0.50 deadband value - change this as needed. When connected, the LEDs on the board should react - check the LaunchPad code to find the specific reaction.


## Author
* **Shashank Swaminathan** - [SSModelGit](https://github.com/SSModelGit)

## Acknowledgements
This code was modified off of the Simple WiFi Server and Client code from the Arduino example libraries.

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
