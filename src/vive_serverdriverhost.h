/*******
This Plugins allows RVIZ to render to the HTC Vive
Copyright (C) 2017 Andre Gilerson

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*******/



/************************************************************************
This file is heavily influeced (and partly lifted) from the VRUI Toolkit
by Olive Kreylos. Check out http://idav.ucdavis.edu/~okreylos/ResDev/Vrui/
***********************************************************************/

#ifndef VIVE_SERVERDRIVERHOST_H
#define VIVE_SERVERDRIVERHOST_H

#include "openvr_driver.h"
#include "vive_driver.h"
#include <iostream>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

class ServerDriverHost: public vr::IServerDriverHost
{
public:
	ServerDriverHost();
	
	struct DeviceState // Structure describing the current state of a tracked device
	{
		/* Elements: */
		public:
		vr::ITrackedDeviceServerDriver* driver; // Pointer to the driver interface for this tracked device
		bool connected; // Flag if this device is currently connected
		float batteryLevel; // Current battery level from 0.0-1.0; always 1.0 for plugged-in devices
		float ipd; // Currently configured inter-pupillary distance in meters
		
		vr::HmdQuaternion_t _rotation;
		double _translation[3];
		/* Constructors and destructors: */
		DeviceState(void)
			:driver(0),connected(false),
			 batteryLevel(1.0f),ipd(0.0635f)
			{
			}
	};
	
	virtual bool TrackedDeviceAdded(const char* pchDeviceSerialNumber);
	virtual void TrackedDevicePoseUpdated(uint32_t unWhichDevice,const vr::DriverPose_t& newPose);
	virtual void TrackedDevicePropertiesChanged(uint32_t unWhichDevice);
	virtual void VsyncEvent(double vsyncTimeOffsetSeconds);
	virtual void TrackedDeviceButtonPressed(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset);
	virtual void TrackedDeviceButtonUnpressed(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset);
	virtual void TrackedDeviceButtonTouched(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset);
	virtual void TrackedDeviceButtonUntouched(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset);
	virtual void TrackedDeviceAxisUpdated(uint32_t unWhichDevice,uint32_t unWhichAxis,const vr::VRControllerAxis_t& axisState);
	virtual void MCImageUpdated(void);
	virtual vr::IVRSettings* GetSettings(const char* pchInterfaceVersion);
	virtual void PhysicalIpdSet(uint32_t unWhichDevice,float fPhysicalIpdMeters);
	virtual void ProximitySensorState(uint32_t unWhichDevice,bool bProximitySensorTriggered);
	virtual void VendorSpecificEvent(uint32_t unWhichDevice,vr::EVREventType eventType,const vr::VREvent_Data_t& eventData,double eventTimeOffset);
	virtual bool IsExiting(void);
	virtual bool PollNextEvent( vr::VREvent_t *pEvent, uint32_t uncbVREvent );
	
	void Update();
	
	Ogre::Quaternion GetDeviceRotation(uint32_t unWhichDevice);
	Ogre::Vector3 GetDeviceTranslation(uint32_t unWhichDevice);
	
	Ogre::Quaternion GetDeviceZeroRotation(uint32_t unWhichDevice);
	Ogre::Vector3 GetDeviceZeroTranslation(uint32_t unWhichDevice);

	float GetDevicePhsycialIpd(uint32_t unWhichDevice);
	bool IsRdy();
	
	void* _pDriverHandle;
	vr::IServerTrackedDeviceProvider* _pVrTrackedDeviceProvider;
	
	DriverLog* _pDriverLog;
	VRSettings* _pVrSettings;
	
	DeviceState* deviceStates; 
	unsigned int _numConnectedDevices; // Number of currently connected tracked devices
	bool proximitySensor;
	
	vr::HmdQuaternion_t _zeroRotation;
	double _zeroTranslation[3];
	bool _rdy;
};

#endif
