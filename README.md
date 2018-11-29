ros_ethercat_igh
----------------

## Overview

This is a generic implementation of a ROS-wrapped EtherCAT Master controller based on the [IgH EtherLab (R)](https://www.etherlab.org/en/what.php) EtherCAT Master Driver for Linux. The ROS EtherCAT master implementation is based on MotorCortex (TM) Core Library provided by [Vectioneer](http://www.vectioneer.com/) as an installable Debian package in a binary form.

The maintained IgH driver installer maintained by [Synapticon](www.synapticon.com) can be downloaded from [here](https://github.com/synapticon/Etherlab_EtherCAT_Master)

MotorCortex (TM) Core Debian packages can be downloaded from [here](http://git.vectioneer.com:30000/pub/motorcortex-dist/wikis/home)

## Installation procedures
### ROS

To install ROS, please refer to the original ROS installation documentation at [http://www.ros.org/install/](http://www.ros.org/install/)

We have developed and tested this package in Ubuntu 16.04 and ROS Kinetic. Although theoretically other versions should be supported, we recommned if possible to start with this configuration.

### IgH EtherCAT Master

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
$ sudo /et/init.d/ethercat start
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

### MotorCortex (TM) Core Library

Navigate to the directory containing the downloaded debian package and then type:

```sh
sudo dpkg -i xxxx
```
Alternatively, double-clicking on the package should open the installation dialog.

### This ROS package

The package contains several modules:
* ethercat_master -> EtherCAT master server application
* motion_control -> Set of Python test scripts demonstrating how to control devices and other useful features
* motorcortex_msgs -> Custom messages for client applications
* ros_ethercat_igh -> Metha package

#### Building
At this point you should be all set to start using this ROS package to control your devices. If you haven't yet cloned it, please do so into your [catkin configured workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment). 
```sh
$ cd ~/catkin_ws/src
$ git clone https://github.com/synapticon/ros_ethercat_igh.git
$ cd ~/catkin_ws
$ catkin build
```
If you were using `catkin_make` command before, you may need to remove `build` and  `devel` folders in your catkin workspace for `catkin build` command to work. Otherwise continue using `catkin_make`. If with `catkin_make` the build will fail first time because the custom message headers cannot be found, please repeat several times to build. We are working on a solution.

#### Configuring

The ethercat_master package requires the information about your EtherCAT slave devices topology. 

in `ethercat_master/src/config/io/` you'll find an example of such a script `topology.xml`. You don't need to write it by yourself but use IgH EtherLab (R) tool to generate it:

```sh
$ ethercat xml > topology.xml
```
Now, if you are using Synapticon's [SOMANET Servo Drives](https://www.synapticon.com/products/somanet/somanet-servo-drives) you can just extend the content of the `pdo.xml` to the number of devices you are using. 
Please be cautious about the tags. There are two types of slave devices are supported: 
* Servo Drives: ```<Device Id="0" Name="axis1">``` with the `Id` corresponding to the topology of the bus starting from `0`, and the associated name starting from `axis1` 
* DIO modules ``` <Device Id="2" Name="device1">``` with the `Id` corresponding to the topology of the bus, and the associated name starting from `device1`

After updating the XML configurations, if you are not using Synapticon devices, you need to register pathes on the master application. For this, please edit the `ethercat_master/src/control/Drive.cpp` file. The only code you need to modify there is in the `initPhase1_()` method. 
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



