# Controlling RFL robot va ROS



|        Robot Controller         | Linux in Virtual Machine             | Windows               |
| :-----------------------------: | ------------------------------------ | --------------------- |
| ABB Robot Controller Rapid Code | ROS                                  | COMPAS_RRC            |
|                                 | COMPAS_RRC_ROS (ROS to Robot Driver) | COMPAS_FAB (ROSLibPy) |
|                                 | ROS Bridge (ROS to ROSLibPy server)  |                       |

## Installation

### VMWare Workstation 

Install VMWare Workstation (current version 15.5.0 obtained from https://idesnx.ethz.ch/)

1. Open image file ubuntu_18_04.vmx
2. Configure CPU / memory / disk image location
3. Test Run the VM

If Credential Guard conplains:

> This is a conflict that can be simply resolved by temporarily disabling Hyper-V hypervisor running this command in the Windows Command Prompt (as admin):
> `bcdedit /set hypervisorlaunchtype off` 
> Restart is required. Apparently, the side effect is that Hyper-V virtual machines cannot be started after this, and so is Docker. To get Hyper-V back, just turn its hypervisor back on:
> `bcdedit /set hypervisorlaunchtype auto` 

### COMPAS_RRC via ROSLibPy (Windows Side)

COMPAS RRC depends on `compas` and `compas_fab`. Install them.

1. Clone Repo from https://bitbucket.org/ethrfl/compas_rrc
   `git clone https://VictorLeungGKR@bitbucket.org/ethrfl/compas_rrc.git`

2. Install this within working environment

   `pip install -r requirements-dev.txt`
   `invoke add-to-rhino`

### In case of installing the Linux from scratch

https://github.com/gramaziokohler/deep_timber/blob/master/docs/interfaces_linux.md

https://github.com/gramaziokohler/deep_timber/blob/master/docs/interfaces_ros.md

https://github.com/gramaziokohler/deep_timber/blob/master/docs/interfaces_ros_bridge.md

https://github.com/gramaziokohler/compas_rrc_ros

## Running the services

### Linux side

Run VMWare

Click "Power on this VM".

Start new terminal window and start **Compas RRC**:

`roslaunch compas_rrc_driver bringup-1_2.launch robot_ip:=192.168.0.11 robot_streaming_port:=30101 robot_state_port:=30201`

 (robot_ip = ip of the robot controller)

------

Another terminalto start **Ros Bridge**: 

`roslaunch rosbridge_server rosbridge_websocket.launch`

------

Another terminal to to check the ip of this VM on network

Enter `ifconfig` , check the inet ip address.

### Windows side

Activate conda environment

`activate itj`

If another environment is binded to Rhino / GH, need to run 

` python -m compas_rhino.install -v 6.0 `

`python -m compas_rhino.install -p compas_fab `

Run scripts that import compas_rrc library. (RosClient IP should match Virtual Machine IP)

```python
from compas_rrc import *
from compas_fab.backends.ros import RosClient
ros = RosClient('192.168.183.128')
abb = AbbClient(ros)
abb.run()
print('Connected.')
abb.close()
abb.terminate()
```

