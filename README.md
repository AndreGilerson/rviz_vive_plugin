#RVIZ Plugin for the HTC Vive

#This Plugin was developed at the Institute for Mineral Resources Machine Technology, RWTH Aachen

This Plugins allows RVIZ to render to the HTC Vive, using [OGRE](http://www.ogre3d.org/). This Plugin was developed and tested on Ubuntu 16.04, 64 bit, with a GTX 1080, and Nvidia 375.26 graphic drivers. 

##Known Issues
While using Ubuntu with the Unity desktop environment, the plugin cant Vsync with the Vive unless the Vive window has input focus. This is due to some issues with Compiz. It is therefore recommended to use Ubuntu with GNOME. To install GNOME on your system open up a terminal and *sudo apt-get install ubuntu-gnome-desktop*. Reboot and choose the GNOME desktop environment in the login screen.

##Requirements
* Roboter Operating System (ROS) (tested with desktop-full install [kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu))
* OpenVR (tested with [OpenVR SDK 1.0.5](Roboter Operating System (ROS) [kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)))
* SteamVR (available through the [Steam client](http://store.steampowered.com/about/), tested with Build ID 1525252)


##Build Instructions
The Plugin is build using [catkin](http://wiki.ros.org/catkin). You have to create a catkin workspace first. Open up a terminal and:

* *mkdir -p catkin_ws/src*
* *cd catkin_ws/src*
* *catkin_init_worksspace*

Then place the *rviz_vive_plugin* folder inside of *catkin_ws/src*. Open the *CMakeList.txt* file inside of the *rviz_vive_plugin* folder and change the Path to OpenVR in line 10. You can then build the plugin by navigating to *catkin_ws* and executing *catkin_make*.

##Running the Plugin
A few steps have to be taken before running the Plugin. First of all, Steam supplies some runtime libraries which are used by SteamVR. Some of those are older versions of commenly used libraries, some of which break RVIZ. Luckily SteamVR does not require those specific libraries, and we can therefore create a folder containing only the required libraries, and add that folder to the *LD_LIBRARY_PATH*. The Steam runtime can be found in you Steam installation folder (commenly *~/.local/share/Steam*). The runtime path is based on your operating system. For me it was *PATH_TO_STEAM/ubuntu12_32/steam-runtime/amd64/lib/x86_64-linux-gnu*. Create a new folder somewhere, note the path, and copy following files from the Steam runtime folder: 

* libdbus-1.so3
* libdbus-1.so.3.5.8
* libgpg-error.so.0.8.0
* libgpg-ncurses.so.5
* libncurses.so.5.9
* libselinux.so.1
* libtinfo.so.5
* libtinfo.so.5.9
* libudev.so.0
* libudev.so.0.13.0
* libwrap.so.0
* libwrrap.so.0.7.6
* libz.so.1
* libz.so.1.2.3.4

Now in the *rviz_vive_plugin* folder again, openup the *runtime_setup.sh* , and change the path to OpenVR, the SteamVR installation path (*PATH_TO_STEAM/steamapps/common/SteamVR*), and the Path to the steamruntime folder you just created.

To finally use the plugin, hookup your Vive, startup your ros environment, etc. Open up a new terminal and navigate to your catkin workspace. Then:

* *source devel/setup.zsh* or *source devel/setup.bash* depending on your shell
* *source src/rviz_vive_plugin/runtime_setup.sh*

Start RVIZ *rosrun rviz rviz*, then press *Add* in the bottom left corner of the window, and add the *ViveDisplay*. Now the view in RVIZ should be rendered to your HTC Vive.

##Credits
This development of this plugin started by rewriting the [oculus_rviz_plugin](http://wiki.ros.org/oculus_rviz_plugins). Dynamically loading parts of the SteamVR library to implement headtracking, is strongly based on the [Vrui VR Toolkit](http://idav.ucdavis.edu/~okreylos/ResDev/Vrui/).