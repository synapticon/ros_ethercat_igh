ros_ethercat_igh
----------------

# Overview

This is a generic implementation of a ROS-wrapped EtherCAT Master controller based on the [IgH EtherLab (R)](https://www.etherlab.org/en/what.php) EtherCAT Master Driver for Linux. The ROS EtherCAT master implementation is based on [MotorCortex (TM) Core Library](http://git.vectioneer.com:30000/pub/motorcortex-dist/wikis/home) provided by [Vectioneer](http://www.vectioneer.com/) as an installable Debian package in a binary form.

The maintained IgH driver installer maintained by [Synapticon](https://www.synapticon.com/) can be downloaded from [here](https://github.com/synapticon/Etherlab_EtherCAT_Master)

MotorCortex (TM) Core Debian packages can be downloaded from [here](http://git.vectioneer.com:30000/pub/motorcortex-dist/wikis/home). To control EtherCAT slave devices you requre the following packages to be downloaded matching with your version of Ubuntu OS:

* Nanomsg Ubuntu
* Motorcortex-core Ubuntu 

# Installation procedures
## ROS

To install ROS, please refer to the original ROS installation documentation at [http://www.ros.org/install/](http://www.ros.org/install/)

We have developed and tested this package in Ubuntu 16.04 and ROS Kinetic. Although theoretically other versions should be supported, we recommned if possible to start with this configuration.

## IgH EtherCAT Master

```sh
$ git clone https://github.com/synapticon/Etherlab_EtherCAT_Master.git
$ cd Etherlab_EtherCAT_Master/sncn_installer
$ chmod +x install.sh && ./install.sh
```

After the installation, if you have more than one Ethernet port, you may need to adjust the ethercat configuration file to use your port.

```sh
$ sudo gedit /etc/sysconfig/ethercat
```
Find the line `MASTER0_DEVICE=""` and add your port's MAC address instead the one filled in automatically. 

To start the driver:
```sh
$ sudo service ethercat start
```
or 
```sh
$ sudo /etc/init.d/ethercat start
```

To make sure your device is recognized and listed, please type 

```sh
$ ethercat slaves
```
 Normally, you should have EtherCAT slave devices listed similar to that:
 
 ```
0  0:0  PREOP  +  SOMANET CiA402 Drive
1  0:1  PREOP  +  SOMANET CiA402 Drive
```

## MotorCortex (TM) Core Library

Navigate to the directory containing the downloaded debian packages and then type:

```sh
sudo dpkg -i nanomsg-1.1.4-Linux.deb
sudo dpkg -i motorcortex-core-0.9.7-Linux.deb
```
Alternatively, double-clicking on the package should open the installation dialog.

## This ROS package

The package contains several modules:
* ethercat_master -> EtherCAT master server application
* motion_control -> Set of Python test scripts demonstrating how to control devices and other useful features
* motorcortex_msgs -> Custom messages for client applications
* ros_ethercat_igh -> Metha package

### Building
At this point you should be all set to start using this ROS package to control your devices. If you haven't yet cloned it, please do so into your [catkin configured workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). 
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/synapticon/ros_ethercat_igh.git
$ cd ~/catkin_ws
$ catkin build
```
If you were using `catkin_make` command before, you may need to remove `build` and  `devel` folders in your catkin workspace for `catkin build` command to work. Otherwise continue using `catkin_make`. If with `catkin_make` the build will fail first time because the custom message headers cannot be found, please repeat several times to build. We are working on a solution.

### Configuring
#### Bus topology
The ethercat_master package requires the information about your EtherCAT slave devices topology. 

in `ethercat_master/src/config/io/` you'll find an example of such a script `topology.xml`. You don't need to write it by yourself but use IgH EtherLab (R) tool to generate it:

```sh
$ ethercat xml > topology.xml
```
Now, if you are using Synapticon's [SOMANET Servo Drives](https://www.synapticon.com/products/somanet/somanet-servo-drives) you can just extend the content of the `pdo.xml` to the number of devices you are using. 
Please be cautious about the tags. There are two types of slave devices are supported: 
* Servo Drives: ```<Device Id="0" Name="axis1">``` with the `Id` corresponding to the topology of the bus starting from `0`, and the associated name starting from `axis1` 
* DIO modules ``` <Device Id="2" Name="device1">``` with the `Id` corresponding to the topology of the bus, and the associated name starting from `device1`

#### Linking your application variables to EtherCAT slave devices' Object Dictionary

##### 1. Process Data Objects (PDO) - realtime data

After updating the XML configurations, if you are not using Synapticon devices, you need to register paths on the master application. For this, please edit the `ethercat_master/src/control/Drive.cpp` file. The only code you need to modify there is in the `initPhase1_()` method. 
Let's explain it on the `Position Value` parameter and Process Data Object. Please exemine the line:
```
addParameter("positionValue", ParameterType::INPUT, &driveFeedback_.position_value);
```
We are adding a parameter of a type INPUT (it is a value transfered from the slave to master, so it is considered as input from the master side) named `"positionValue"` and linking it to our internal variable `driveFeedback_.position_value` in this case being a member of the Drive Feedback ROS message (`motorcortex_msgs::DriveIn driveFeedback_`). This procedure creates an internal path ```root/Control/axisX/positionValue```, where `X` in `axisX` is automatically assigned enumeration starting from 1.

Please now navigate to your `pdo.xml` file (`thercat_master/src/config/io/pdo.xml`). There you'll find the following entry:
```
<Pdo Entry="6064:0" Group="In" Name="Position Value">
    <DataType>INT32</DataType>
    <Link>root/Control/axis1/positionValue</Link>
</Pdo>
```
You may spot the already familiar line `<Link>root/Control/axis1/positionValue</Link>` linking your registered `"positionValue"` variable to the PDO Entry `Entry="6064:0"` At this point you should become familiar enough how to repeate the same procedure for all other variables and Process Data Objects you would like to link. 

##### 2. Service Data Objects (SDO) - configuration parameters
The same logic exists for Service Data Objects (SDO). Please navigate to `ethercat_master/src/control/DriveSdo.cpp` file. Let's examine this line:
```
param = addParameter("velocityControllerKp", ParameterType::PARAMETER, &sdoCfg_.velocityControllerCfg.controller_Kp);
    handles_.push_back(param);
```
We are adding a parameter of a type PARAMETER named `"velocityControllerKp"` and linking it to our internal variable `sdoCfg_.velocityControllerCfg.controller_Kp` in this case being a member of the `SDOCfg` service and contained data structure:
```
struct SDOCfg {
    motorcortex_msgs::TorqueControllerCfg torqueControllerCfg;
    motorcortex_msgs::VelocityControllerCfg velocityControllerCfg;
    motorcortex_msgs::PositionControllerCfg positionControllerCfg;
};
```
This procedure creates two internal pathes `root/Control/axisX/SDORead/velocityControllerKp` and `root/Control/axisX/SDOWrite/velocityControllerKp`, where `X` in `axisX` is automatically assigned enumeration starting from 1. This is a result of creating two submodules in `ethercat_master/src/control/Drive.cpp` `create_()` method. One is used for writing the value and one for reading both linked to the same variable `"velocityControllerKp"`:

```
createSubmodule(&drive_sdo_read_, "SDORead");
createSubmodule(&drive_sdo_write_, "SDOWrite");
```

Please now navigate to your `pdo.xml` file (`thercat_master/src/config/io/pdo.xml`). There you'll find the following entry:
```
<Sdo Entry="2011:1" Size="4" Group="SDOs/Velocity Controller" Name="Controller Kp">
    <DataType>FLOAT</DataType>
    <LinkTo>root/Control/axis1/SDORead/velocityControllerKp</LinkTo>
    <LinkFrom>root/Control/axis1/SDOWrite/velocityControllerKp</LinkFrom>
</Sdo>
```
You may spot again the already familiar lines `<LinkTo>root/Control/axis1/SDORead/velocityControllerKp</LinkTo>` and `<LinkFrom>root/Control/axis1/SDOWrite/velocityControllerKp</LinkFrom>`, one is linking the `Sdo Entry="2011:1"` on writing, and one on reading. From this point you should be able to link your other parameters in a similar way.

##### 3. Defining the amount of devices to be used

Once you've configured and linked your variables, you need tel the server application how many instances to control to be created. For this please navigate to `main.cpp` file inside the `ethercat_master/src/` folder. You'll find the following lines:
```
#define NUM_OF_DRIVES 2
#define NUM_OD_DIOS 0
```

Please adjust the number according to the amount of devices you want to control. It is possible to have more devices included in the `topology.xml` and `pdo.xml` but control fewer of them.

##### 4. Rebuild the server application after making the changes
```sh
$ cd ~/catkin_ws
$ catkin build
```
##### 5. Run the `ehtercat_master` server application

For your convinience there is a launch file. Please execute:
```sh
roslaunch motion_control motion_control.launch
```

### Motion control test applications

The `motion_control` package contains a set of python scripts to test basic functionality and to guide you how to develop your own application.

* test_ctrl.py -> main Drives and DIO control application
  * Uses classes drive.py and dio.py
* test_get_SDO.py -> service example to read your configuration parameters over SDOs
* test_set_SDO.py -> service example to write your configuration parameters over SDOs
* test_SDO_get_n_set.py -> service example to read, modify, and then write your configuration parameters over SDOs
* test_restore_params.py -> service example to restore your configuration parameters from saved on a device
* test_save_params.py -> service example to save your configuration parameters to a device

#### Examining `test_ctrl.py`

This is a simple application to make your motors turn in different operational modes and generate pulses on outputs of your DIO module. Let's exemine the code line by line.

```
from __future__ import print_function, division

import rospy
from drive import Drive, OpModesCiA402
from dio import DIO

from motorcortex_msgs.msg import DriveOutList, DriveInList, DigitalInputsList, DigitalOutputsList
```
Here we are importing all the dependancies and our custom ROS messages to be used by the application. The messages are defined in the `motorcortex_msgs` package.

```
# list your slave devices here
drives = [Drive(0), Drive(1)]
dios = [] # for example, [DIO(0)]
```
The first think you need to do is to list your devices. In this example two Drives are listed to be controlled and no DIO module.

```
def drives_feedback_callback(drive_feedback):
    for drive in drives:
        drive.update(drive_feedback.drives_feedback[drive.getId()])

def digital_inputs_callback(digital_inputs):
    if not dios:
        rospy.logerr("The list of DIOs is empty. Please unsubscribe or list the devices")
    else:
        for dio in dios:
            dio.update(digital_inputs.devices_feedback[dio.getId()])
```
Defining callback functions for Drives and DIOs. 

```
# publish control commands
drives_pub = rospy.Publisher('/drive_control', DriveOutList, queue_size = 1)
if dios:
    dios_pub = rospy.Publisher('/digital_outputs', DigitalOutputsList, queue_size = 1)

# subscribe to feedback topics
rospy.Subscriber("/drive_feedback", DriveInList, drives_feedback_callback)
if dios:
    rospy.Subscriber("/digital_inputs", DigitalInputsList, digital_inputs_callback)
```
Creating publishers and subscribers. In this version the DIO module is optional.

```
def safe_off():
    drivesControlMsg = DriveOutList()
    for drive in drives:
        drive.switchOff()
        drivesControlMsg.drive_command.append(drive.encode())

    digitalOutputsControlMsg = DigitalOutputsList()
    if dios:
        for dio in dios:
            digital_outputs = [False] * 12
            dio.setDigitalOutputs(digital_outputs)
            digitalOutputsControlMsg.devices_command.append(dio.encode())

    drives_pub.publish(drivesControlMsg)
    if dios:
        dios_pub.publish(digitalOutputsControlMsg)
```
Operation to be performed on the application termination. For drives we are commanding switch off which is triggered over a quick-stop internally. The outputs of DIO modules are set to zero.

Now we come to the main `controller()` function definition.
```
def controller():
    rospy.init_node('drive_control', anonymous=True)
    rospy.on_shutdown(safe_off)
    r = rospy.Rate(100)#Hz
```
Here we are initializing the ROS node, defining what function to execute when we shutting it down, and defining the control loop rate at 100Hz.

```
    # set your test parameters here
    opMode = OpModesCiA402.CSV.value
    positionIncrement = 1000
    referenceVelocity = 500
    referenceTorque = 50
```
Here you can select you control mode. Options are:
* OpModesCiA402.CSP.value -> Cyclic Synchronous Position
* OpModesCiA402.CSV.value -> Cyclic Synchronous Velocity
* OpModesCiA402.CST.value -> Cyclic Synchronous Torque

Then depending on your prefered control mode you can select target values. Please note, there is no ramp implemented! Sending high torque or velocity references can result in drives overcurrents. Please start with small values.

```
    # switch on here
    while not rospy.is_shutdown():
```
Beginning of the control routine.
```
        # control here
        drivesControlMsg = DriveOutList()
        for drive in drives:
            if drive.hasError():
                print("drive %d has error"%drive.getId())
                drive.resetError()
            else:
                drive.setMode(opMode)
                drive.switchOn()

            if drive.isEnabled():
                if opMode == OpModesCiA402.CSP.value:
                    drive.setPosition(drive.getPosition() + positionIncrement)
                elif opMode == OpModesCiA402.CSV.value:
                    drive.setVelocity(referenceVelocity)
                elif opMode == OpModesCiA402.CST.value:
                    drive.setTorque(referenceTorque)
            else:
                drive.setPosition(drive.getPosition())
                drive.setVelocity(0)
                drive.setTorque(0)

            drivesControlMsg.drive_command.append(drive.encode())
```
The main drives control is happening here. 
