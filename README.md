# This project was developed while no official Linux support was available for the HTC Vive and is therefore deprecated. I started a new project using the newest OpenVR and SteamVR versions as of 12.10.2018 in a [new repository](https://github.com/AndreGilerson/rviz_vive). I see no reason to further use or develop this project, but want to keep this repository around to be on the safe side with my university

# RVIZ Plugin for the HTC Vive

# This Plugin was developed at the [Institute for Mineral Resources Machine Technology, RWTH Aachen University](http://www.imr.rwth-aachen.de/)

![screenshot from 2017-02-02 13-46-32](https://cloud.githubusercontent.com/assets/25487099/22546675/5bb0eef4-e93e-11e6-91fd-0e647c1953b2.png)

## 0.0 Introduction
This Plugins allows [RVIZ](http://wiki.ros.org/rviz) to render to the [HTC Vive](https://www.vive.com/), using [OGRE](http://www.ogre3d.org/). RVIZ is part of the Robot Operating System ([ROS](http://www.ros.org/)), an open source framework designed to simplify many processes in robot software development. RVIZ is a convient visualization tool, part of the main ROS install.
This Plugin takes creates an additional render output in RVIZ, applies lens correction etc. allowing the user to immerse themselves into the simulated world.

As of February 1st, 2017 the Vive does not officially support Linux yet. Altough linux builds of SteamVR and OpenVR are available, theyre not fully functional. Ubuntu does however recognize the Vive as an additional screen. This plugin renders to a window on that screen and uses parts of SteamVR and OpenVr to implement headtracking.

This Plugin was developed as part of a term paper by Andre Gilerson. The term paper was supervised by [Sascha Schade](https://github.com/strongly-typed), in a cooperation of the Insitute for Mineral Resources Machine Technology and the [Institute for Man-Machine Interaction](https://www.mmi.rwth-aachen.de/). The plugin was developed and tested on Ubuntu 16.04, 64 bit, with a GTX 1080, and Nvidia 375.26 graphic drivers.

## 1.0 Known Issues
While using Ubuntu with the Unity desktop environment, the plugin cant Vsync with the Vive unless the Vive window has input focus. This is due to some issues with Compiz. It is therefore recommended to use Ubuntu with GNOME. To install GNOME on your system open up a terminal and *sudo apt-get install ubuntu-gnome-desktop*. Reboot and choose the GNOME desktop environment in the login screen.

## 2.0 Requirements

### 2.1 Dependencies
Approriate drivers for your graphisc card, SDL2 and GLEW. Furthermore git is recommended, for downloading and updating other packages.
* *sudo apt install git libsdl2-dev libglew-dev*

### 2.2 Getting the Robot Operating System (ROS)
To install ROS please follow the instructions provided here [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). This plugin was developed using the *desktop-full* install. After installing ROS, install catkin:
* *sudo apt install catkin*

### 2.3 Getting OpenVR
OpenVR can be cloned from the official [OpenVR Github](https://github.com/ValveSoftware/openvr) repository:
* *git clone https://github.com/ValveSoftware/openvr*
This plugin was developed using OpenVR SDK v1.0.5. Later versions might brake the plugin.
* *cd openvr*
* *git checkout v1.0.5*

### 2.4 Getting SteamVR
Install the steam client (*sudo apt-get install steam*) from the Ubuntu repositories. If you have filtered Internet access, you need to run Steam with the -tcp option. Start Steam from a console *steam -tcp*, login and download SteamVR inside of the Steam Client. 

Then, go to the *Library -> VR*, right click on the SteamVR entry, navigate to *Properties -> Local Files -> Browse Local Files* and note the path. The path is probably */home/upns/.steam/steam/steamapps/common/SteamVR* but may differ.

The plugin was developed using the SteamVR build from January 3rd, 2017. Later versions might brake the plugin. To get older versions of SteamVR, install the Steam command line tool: *sudo apt-get install steamcmd*. Start it with *steamcmd* and use following command:
* *login <your steam user name>*
* *download_depot 250820 250823 1008772584334738762*
This downloads the SteamVR build from January 3rd, 2017. 250820 is the ID of SteamVR, 250823 the ID of the linux depot, and 1008772584334738762 the ID of the correct build. The command line tool will output the path, where it has downloaded the files to, e.g. */home/username/.steam/steamcmd/linux32\steamapps\content\app_250820\depot_250823*. Note that Steam was developed for Windows and did not learn that Unix uses slashes for directories. Copy the files into the SteamVR directory you noted earlier, e.g.
*cp -r /home/username/.steam/steamcmd/linux32/steamapps/content/app_250820/depot_250823/* /home/username/.steam/steam/steamapps/common/SteamVR/*.
Merge and replace existing files.



## 3.0 Build Instructions
The Plugin is built using [catkin](http://wiki.ros.org/catkin). You have to create a catkin workspace first. Open up a terminal and:

* *mkdir -p catkin_ws/src*
* *cd catkin_ws/src*
* *catkin_init_worksspace*

Clone this repository into *catkin_ws/src*. Open the *CMakeList.txt* file inside of the *rviz_vive_plugin* folder and change the Path to the OpenVR repository from part 2.3 in line 10. You can then build the plugin by navigating to *catkin_ws* and executing *catkin_make* in a terminal.

## 4.0 Running the Plugin

A few steps have to be taken before running the Plugin. First of all, Steam supplies some runtime libraries which are used by SteamVR. Some of those are older versions of commenly used libraries, some of which break RVIZ. Luckily SteamVR does not require those specific libraries, and we can therefore create a folder containing only the required libraries, and add that folder to the *LD_LIBRARY_PATH*. The Steam runtime can be found in you Steam installation folder. The exact path is based on your operating system (for me it was *~/.steam/ubuntu12_32/steam-runtime/amd64/lib/x86_64-linux-gnu*). Create a new folder somewhere, note the path, and copy following files from the Steam runtime folder into the folder you just created: 

* libdbus-1.so.3
* libdbus-1.so.3.5.8
* libgpg-error.so.0.8.0
* libncurses.so.5
* libncurses.so.5.9
* libselinux.so.1
* libtinfo.so.5
* libtinfo.so.5.9
* libudev.so.0
* libudev.so.0.13.0
* libwrap.so.0
* libwrap.so.0.7.6
* libz.so.1
* libz.so.1.2.3.4

Now in the *rviz_vive_plugin* folder again, openup the *runtime_setup.sh* , and change the path to OpenVR, the SteamVR installation path (*PATH_TO_STEAM/steamapps/common/SteamVR*), and the Path to the steamruntime folder you just created.

To finally use the plugin, hookup your Vive, startup your ros environment, etc. Open up a new terminal and navigate to your catkin workspace. Then:

* *source devel/setup.zsh* or *source devel/setup.bash* depending on your shell
* *source src/rviz_vive_plugin/runtime_setup.sh*
* If you dont have a roscore started, open up a new terminal and start one with *roscore*
* *chmod a+rw /dev/hidraw* *, to give the necessary permissions to read headtracking data from the Vive

Start RVIZ *rosrun rviz rviz*, then press *Add* in the bottom left corner of the window, and add the *ViveDisplay*. Now the view in RVIZ should be rendered to your HTC Vive.

To make the permission change permament, you will have to create a udev rule. Create a new file named *vive.rules* in */etc/udev/rules.d/*:
* *sudo gedit /etc/udev/rules.d/vive.rules*
Paste following rules inside of it and save:
KERNEL=="hidraw*", ATTRS{idVendor}=="0bb4", MODE="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="28de", MODE="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="0424", MODE="0666"



## 5.0 Credits
This development of this plugin started by rewriting the [oculus_rviz_plugin](http://wiki.ros.org/oculus_rviz_plugins). Dynamically loading parts of the SteamVR library to implement headtracking, is heavily based on the [Vrui VR Toolkit](http://idav.ucdavis.edu/~okreylos/ResDev/Vrui/).

Initial research was done by [Alexander Luehm](https://github.com/luehm), his first results can be viewed [here](https://github.com/luehm/vive_rviz_plugins). The Term paper was supervised by [Sascha Schade](https://github.com/strongly-typed)
