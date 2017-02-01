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


#include "vive_serverdriverhost.h"

#include <dlfcn.h>
#include <cstddef>
#include <cstring>
#include <cstdlib>

extern "C" {
#ifndef DECLSPEC
#if defined(__GNUC__) && __GNUC__ >= 4
	#define DECLSPEC __attribute__ ((visibility("default")))
#else
	#define DECLSPEC
#endif
#endif

#ifndef SDLCALL
#define SDLCALL
#endif

typedef struct
{
	uint32_t format;
	int w;
	int h;
	int refresh_rate;
	void *driverdata;
} SDL_DisplayMode;

typedef struct
{
	int x,y;
	int w, h;
} SDL_Rect;

	
DECLSPEC int SDLCALL SDL_GetNumVideoDisplays(void);
DECLSPEC int SDLCALL SDL_GetCurrentDisplayMode(int displayIndex,SDL_DisplayMode* mode);
DECLSPEC int SDLCALL SDL_GetDisplayBounds(int displayIndex,SDL_Rect* rect);
DECLSPEC const char* SDLCALL SDL_GetDisplayName(int displayIndex);
}




int SDL_GetNumVideoDisplays(void)
{
	return 1;
}

int SDL_GetCurrentDisplayMode(int displayIndex,SDL_DisplayMode* mode)
{
	memset(mode,0,sizeof(SDL_DisplayMode));
	mode->format=0x16161804U;
	mode->w=2160;
	mode->h=1200;
	mode->refresh_rate=89;
	mode->driverdata=0;
	
	return 0;
}

int SDL_GetDisplayBounds(int displayIndex,SDL_Rect* rect)
{
	rect->x=0;
	rect->y=0;
	rect->w=2160;
	rect->h=1200;
	
	return 0;
}

const char* SDL_GetDisplayName(int displayIndex)
{
	return "HTC Vive\"";
}






ServerDriverHost::ServerDriverHost()
{
	deviceStates=new DeviceState[6];
	
	_numConnectedDevices = 0;
	_rdy = 0;

	std::string steamVRPath = std::getenv("steamvr");
	std::string driverPath = steamVRPath + "/drivers/lighthouse/bin/linux64/driver_lighthouse.so";
	std::string driverProviderPath = steamVRPath + "/drivers/lighthouse/bin/linux64";
	_pDriverHandle = dlopen(driverPath.c_str(), RTLD_NOW);
	if(!_pDriverHandle)
	{
		std::cout << "Unable to load SteamVR Lighthouse driver due to:" << dlerror() << std::endl;
	}
	
	typedef void* (*HmdDriverFactoryFunction)(const char* pInterfaceName,int* pReturnCode);
	ptrdiff_t factoryIntermediate=reinterpret_cast<ptrdiff_t>(dlsym(_pDriverHandle,"HmdDriverFactory"));
	HmdDriverFactoryFunction HmdDriverFactory=reinterpret_cast<HmdDriverFactoryFunction>(factoryIntermediate);
	
	if(!HmdDriverFactory)
	{
		std::cout << "Unable to load SteamVR driver factory function due to:" << dlerror() << std::endl;
	}
	
	int error = 0;	
	_pVrTrackedDeviceProvider = reinterpret_cast<vr::IServerTrackedDeviceProvider*>(HmdDriverFactory(vr::IServerTrackedDeviceProvider_Version,&error));
	if(!_pVrTrackedDeviceProvider)
	{
		std::cout << "Unable to retrieve vrTrackedDeviceProvider: " << error << std::endl;
	}
	
	_pDriverLog = new DriverLog();
	_pVrSettings = new VRSettings();
	
	vr::EVRInitError initError=_pVrTrackedDeviceProvider->Init(_pDriverLog,this, "/home", driverProviderPath.c_str());
	if(int(initError))
	{
		std::cout << "Unable to initialize vrTrackedDeviceProvider: " << int(initError) << std::endl;
	}
	
	_pVrTrackedDeviceProvider->LeaveStandby();	
	
	unsigned int numDevices=_pVrTrackedDeviceProvider->GetTrackedDeviceCount();
	for(unsigned int i=0;i<numDevices;++i)
	{
		vr::ITrackedDeviceServerDriver* driver=deviceStates[_numConnectedDevices].driver=_pVrTrackedDeviceProvider->GetTrackedDeviceDriver(i);
		if(driver!=0)
		{
			vr::EVRInitError error=driver->Activate(_numConnectedDevices);
			if(error==vr::VRInitError_None)
			{
				vr::ETrackedPropertyError error=vr::TrackedProp_Success;
				if(driver->GetBoolTrackedDeviceProperty(vr::Prop_DeviceProvidesBatteryStatus_Bool,&error)&&error==vr::TrackedProp_Success)
				{
					deviceStates[_numConnectedDevices].batteryLevel=driver->GetFloatTrackedDeviceProperty(vr::Prop_DeviceBatteryPercentage_Float,&error);
					if(error!=vr::TrackedProp_Success)
						deviceStates[_numConnectedDevices].batteryLevel=0.0f; // If there was an error, assume battery is empty
				}
				else
					deviceStates[_numConnectedDevices].batteryLevel=1.0f; // Devices without battery are assumed fully charged
			}
			else
				std::cout<<"Unable to activate tracked device "<<_numConnectedDevices<<" due to OpenVR error "<<error<<std::endl;
			
			++_numConnectedDevices;
		}
		else
			std::cout<<"Error retrieving tracked device "<<i<<std::endl;
	}
}

bool ServerDriverHost::TrackedDeviceAdded(const char* pchDeviceSerialNumber)
{
	bool result=false;
	
	vr::ITrackedDeviceServerDriver* driver=deviceStates[_numConnectedDevices].driver=_pVrTrackedDeviceProvider->FindTrackedDeviceDriver(pchDeviceSerialNumber);
	if(driver!=0)
	{
		vr::EVRInitError error=driver->Activate(_numConnectedDevices);
		if(error==vr::VRInitError_None)
		{
			vr::ETrackedPropertyError error=vr::TrackedProp_Success;
			if(driver->GetBoolTrackedDeviceProperty(vr::Prop_DeviceProvidesBatteryStatus_Bool,&error)&&error==vr::TrackedProp_Success)
			{
				deviceStates[_numConnectedDevices].batteryLevel=driver->GetFloatTrackedDeviceProperty(vr::Prop_DeviceBatteryPercentage_Float,&error);
				if(error!=vr::TrackedProp_Success)
					deviceStates[_numConnectedDevices].batteryLevel=0.0f;
			}
			else
				deviceStates[_numConnectedDevices].batteryLevel=1.0f;
			std::cout<<"ServerDriverHost: Battery level of device "<<_numConnectedDevices<<" is "<<deviceStates[_numConnectedDevices].batteryLevel*100.0f<<"%"<<std::endl;
			
			result=true;
		}
		else
			std::cout<<"ServerDriverHost: Unable to activate connected device "<<_numConnectedDevices<<" due to OpenVR error "<<error<<std::endl;
		
		++_numConnectedDevices;
	}
	else
		std::cout<<"ServerDriverHost: Error retrieving connected device "<<_numConnectedDevices<<std::endl;
	
	return result;
	}

void ServerDriverHost::TrackedDevicePoseUpdated(uint32_t unWhichDevice,const vr::DriverPose_t& newPose)
{
	DeviceState& state=deviceStates[unWhichDevice];
	
	if(newPose.deviceIsConnected&&newPose.poseIsValid)
	{
		if(!state.connected)
		{
			state.connected=true;
		}
			if(!_rdy)
			{
				_rdy = 1;
				_zeroRotation = newPose.qRotation;
				_zeroTranslation[0] = newPose.vecPosition[0];
				_zeroTranslation[1] = newPose.vecPosition[1];
				_zeroTranslation[2] = newPose.vecPosition[2];
			}
			state._rotation = newPose.qRotation;
			state._translation[0] = newPose.vecPosition[0];
			state._translation[1] = newPose.vecPosition[1];
			state._translation[2] = newPose.vecPosition[2];
	}
	else
	{
		if(state.connected)
		{
			state.connected=false;
		}
	}
}

void ServerDriverHost::TrackedDevicePropertiesChanged(uint32_t unWhichDevice)
{
	DeviceState& ds=deviceStates[unWhichDevice];
	
	vr::ETrackedPropertyError error=vr::TrackedProp_Success;
	if(ds.driver->GetBoolTrackedDeviceProperty(vr::Prop_DeviceProvidesBatteryStatus_Bool,&error)&&error==vr::TrackedProp_Success)
	{
		ds.batteryLevel=ds.driver->GetFloatTrackedDeviceProperty(vr::Prop_DeviceBatteryPercentage_Float,&error);
		if(error!=vr::TrackedProp_Success)
			ds.batteryLevel=0.0f;
	}
	else
		ds.batteryLevel=1.0f;
}

void ServerDriverHost::VsyncEvent(double vsyncTimeOffsetSeconds)
{

}

void ServerDriverHost::TrackedDeviceButtonPressed(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset)
{
	if(unWhichDevice>=1U)
	{
		int baseIndex=(int(unWhichDevice)-1)*5;
		int buttonIndex=eButtonId>=32?eButtonId-29:eButtonId;
	}
}

void ServerDriverHost::TrackedDeviceButtonUnpressed(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset)
{
	if(unWhichDevice>=1U)
	{
		int baseIndex=(int(unWhichDevice)-1)*5;
		int buttonIndex=eButtonId>=32?eButtonId-29:eButtonId;
	}
}

void ServerDriverHost::TrackedDeviceButtonTouched(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset)
{

}

void ServerDriverHost::TrackedDeviceButtonUntouched(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset)
{

}

void ServerDriverHost::TrackedDeviceAxisUpdated(uint32_t unWhichDevice,uint32_t unWhichAxis,const vr::VRControllerAxis_t& axisState)
{
	if(unWhichDevice>=1U)
	{
		int baseIndex=(int(unWhichDevice)-1)*3;
		if(unWhichAxis==0U)
		{
		}
	}
}

void ServerDriverHost::MCImageUpdated(void)
{

}

vr::IVRSettings* ServerDriverHost::GetSettings(const char* pchInterfaceVersion)
{
	if(strcmp(pchInterfaceVersion,vr::IVRSettings_Version)==0)
		return _pVrSettings;
	else
	{
		return 0;
	}
}

void ServerDriverHost::PhysicalIpdSet(uint32_t unWhichDevice,float fPhysicalIpdMeters)
{
	if(deviceStates[unWhichDevice].ipd!=fPhysicalIpdMeters)
	{
		deviceStates[unWhichDevice].ipd = fPhysicalIpdMeters;
	}
}

void ServerDriverHost::ProximitySensorState(uint32_t unWhichDevice,bool bProximitySensorTriggered)
{
	if(proximitySensor!=bProximitySensorTriggered)
	{
		proximitySensor=bProximitySensorTriggered;
	}
}

void ServerDriverHost::VendorSpecificEvent(uint32_t unWhichDevice,vr::EVREventType eventType,const vr::VREvent_Data_t& eventData,double eventTimeOffset)
{

}

bool ServerDriverHost::IsExiting(void)
{
	return true;
}

bool ServerDriverHost::PollNextEvent( vr::VREvent_t *pEvent, uint32_t uncbVREvent )
{
	return false;
}

void ServerDriverHost::Update()
{
	_pVrTrackedDeviceProvider->RunFrame();
}

Ogre::Quaternion ServerDriverHost::GetDeviceRotation(uint32_t unWhichDevice)
{
	return Ogre::Quaternion(deviceStates[unWhichDevice]._rotation.w, deviceStates[unWhichDevice]._rotation.x,
							deviceStates[unWhichDevice]._rotation.y, deviceStates[unWhichDevice]._rotation.z);
}

Ogre::Vector3 ServerDriverHost::GetDeviceTranslation(uint32_t unWhichDevice)
{
	return Ogre::Vector3(deviceStates[unWhichDevice]._translation[0], deviceStates[unWhichDevice]._translation[1],
						deviceStates[unWhichDevice]._translation[2]);
}

Ogre::Quaternion ServerDriverHost::GetDeviceZeroRotation(uint32_t unWhichDevice)
{
	return Ogre::Quaternion(_zeroRotation.w, _zeroRotation.x,
							_zeroRotation.y, _zeroRotation.z);
}

Ogre::Vector3 ServerDriverHost::GetDeviceZeroTranslation(uint32_t unWhichDevice)
{
	return Ogre::Vector3(_zeroTranslation[0], _zeroTranslation[1],
						_zeroTranslation[2]);
}

float ServerDriverHost::GetDevicePhsycialIpd(uint32_t unWhichDevice)
{
	return deviceStates[unWhichDevice].ipd;
}

bool ServerDriverHost::IsRdy()
{
	return _rdy;
}
