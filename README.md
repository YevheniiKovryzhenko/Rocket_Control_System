# Rocket Control System (RCS)

## Hello there 👋

This code was made for the Auburn Rocketry Team when I was the embedded systems team lead. This is a heavy rework of a drone control system, the original [rc_pilot](https://github.com/StrawsonDesign/rc_pilot), developed by Dr. Stawson, Dr. Gaskell, and others.

This is a native C++ altitude control system for active breaking of our rocket, that we developed for USLI and Spaceport America Cup competitions during 2019-2021. This implements a fully autonomous rocket autopilot from the point the system is armed on the launchpad till disarmed on landing. Additionally, we implemented remote control (preflight checklists, arm/disarm, reporting), logging, and a backup companion computer support. 

The state machine and event detection algorithm was successfully tested through multiple launches, and was proven to be capable of detecting all nominal flight events (ignition, liftoff, powered ascend, unpowered ascend, free fall, parachute deployment, and landing) to make sure we do not violate any FAA regulation and active control is only allowed during unpowered phases of flight. This also implements PID control for apogee and course correction (from vertical), but we have only been able to flight test the apogee control part of the algorithm and minimum-snap trajectory guidance. In essence, the vehicle was designed to overshoot the target by some margin (to account for any possible errors) and the RCS was deploying actively deploying airbrakes in flight to compensate and match the predicted apogee with the target apogee. 

This algorithm has been designed to work in pairs with our custom-designed companion module that worked as a redundancy computer (in case anything has failed). The companion module was developed by my team, so feel free to reach out to [Kazybek Mizam](https://github.com/1kzpro) or [Vladimir Eganov](https://github.com/vladimireganov) if you want to learn more about it.

I have not used it for many years now, so some instructions below may no longer work properly. This repository primarily serves as a record of my first introduction to the field of embedded C and hardware integration in general. It is critical to learn and understand the flight code before using it. Make sure you know what you are doing and don't hold me liable for any equipment damage and follow the law!

THIS CODE IS NOT FOR PUBLIC USE. USE AT YOUR OWN RISK. THIS CAN AND WILL GET YOU IN JAIL IF MISUSED.
			CHECK WITH FAA AND LOCAL REGULATIONS

# Installation:
Made to run on Beaglebone Blue (BBB) Linux board with the Debian image from the official website:\
* A very helpful starting point - [BBB getting started page](https://beagleboard.org/getting-started).
* You must install [official BBB image](https://beagleboard.org/latest-images).
* I recommend you familiarize yourself with the [robot control library](http://strawsondesign.com/docs/librobotcontrol/) (start with the manual).

## Project Compilation:
Download the project folder either to your workstation and transfer the files to BBB using scp or [WinSCP (Windows)](https://winscp.net/eng/index.php):
You can place <this_project_name> that you have downloaded from GitHub into (BBB) into
"/home/debian/this_project_name/" directory.
Afterwards, create a build directory <build_folder>:
```bash
mkdir /home/debian/build_folder/
```
	
This would be the location where you want to compile the project to.

### Complie Project:
Go into the root folder of the project:
```bash
cd /home/debian/Rocket_Control_System/
make
```

# Running the project: 
To run the project, you just need to call the main executable with the proper arguments. To see the list of arguments:
```bash
/home/debian/Rocket_Control_System/rc_pilot -h
```
For now, it only requires attaching a proper <SETTINGS_FILE> as:
	
	/home/debian/Rocket_Control_System/bin/rc_pilot -s /home/debian/Rocket_Control_System/Settings/SETTINGS_FILE.json

# Notes:
## Magnetometer (Compass) Issues
If you have just installed a brand new Debian image, you may run into a problem that the compass is not accessible (rc_calibrate_mag exit with error).

The first thing to try is to run the example script for dmp mode with the magnetometer option enabled:
```bash
rc_test_dmp -m -c -t
```
	
This should print out raw compass readings + filtered and fused Tait Bryan angles. Even if calibration was never performed, this should still work...
Try running calibration again:
```bash
rc_calibrate_mag
```
	
If the same issues persist, try to update everything and repeat the steps above.

## Updating everything:
There are cases when you might need to update all the libraries and even Linux Kernel to remove some of the bugs.
First of all, update all the libraries (as a root):
```bash
apt-get update && apt update && apt-get upgrade -y && apt upgrade -y
```
	
Now go and update the scripts for updating Kernel:
```bash
cd /opt/scripts
git pull
```
Update Kernel (as a root):
```bash
tools/update_kernel.sh
```
	
Reboot for the changes to become effective:
```bash
shutdown -r now
```
You may need to update everything once again since you are now running the most recent Linux Kernel and more recent updates are now available:
```bash
apt-get update && apt update && apt-get upgrade -y && apt upgrade -y
```
	
Some packages might want to be updated manually, for example:
```bash
apt upgrade c9-core-installer
```	
Where you need to manually accept (Y) the update. This should be done for all the packages that have been "kept back".

## Modifying the source code:
It is advised to always work on your machine (Windows/MAC OS Desktop/Laptop) to avoid any possible loss of changes on BBB (it can crash). Also, it is much easier to keep track of the new changes if working on your machine and only transferring+compiling on BBB (especially when using Visual Studio or similar Github-enabled environments). If you want to modify the source code from BBB, you need to do it in <this_project_name> folder and repeat the compilation steps using "make".

## Documentation:
You can look into docs/ folder for more information regarding code layout and details regarding the control system, estimation, etc. Also, check out one of our [final readiness reports](https://tigermailauburn-my.sharepoint.com/personal/yzk0058_auburn_edu/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fyzk0058_auburn_edu%2FDocuments%2FCourses%20-%20Aerospace%20Engineering%20%5BAuburn%20University%5D%2FOLD%2FComp%201200%2FAssignments%2F2020_2021_USLI_FRR%2Epdf&parent=%2Fpersonal%2Fyzk0058_auburn_edu%2FDocuments%2FCourses%20-%20Aerospace%20Engineering%20%5BAuburn%20University%5D%2FOLD%2FComp%201200%2FAssignments&ct=1724943886161&or=OWA-NTB-Mail&cid=7998f374-fb00-e90d-0ccb-b0bec2507f74&ga=1) for more details about the rocket, hardware, integration and flight testing!

# Contact
If you have any questions, please feel free to contact me, Yevhenii (Jack) Kovryzhenko, at yzk0058@auburn.edu.

# Credit
This work started during my undergraduate years at Auburn University, under the supervision of Dr. Eldon Triggs. 

I would also like to express my appreciation to [Kazybek Mizam](https://github.com/1kzpro), [Vladimir Eganov](https://github.com/vladimireganov) and the the rest of my team at Auburn University Rocketry Association. Those 3 years have been incredible!

Check out my more recent autopilot-related works like [PX4 Simulink IO Framework](https://github.com/YevheniiKovryzhenko/PX4_SIMULINK_IO_Framework.git), [RRTV TiltWing](https://github.com/YevheniiKovryzhenko/RRTV_TiltWing.git) and [Quadrotor_with_FF_Control](https://github.com/YevheniiKovryzhenko/Quadrotor_with_FF_Control.git). 
