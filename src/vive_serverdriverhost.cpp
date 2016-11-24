#include "vive_serverdriverhost.h"

#include <dlfcn.h>
#include <cstddef>
#include <cstring>






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
	mode->format=0x16161804U; // Hard-coded for SDL_PIXELFORMAT_RGB888
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
	return "HTC Vive 5\"";
	}






ServerDriverHost::ServerDriverHost()
{
	deviceStates=new DeviceState[6];
	
	_numConnectedDevices = 0;
	_rdy = 0;
	
	_pDriverHandle = dlopen("/home/upns/.local/share/Steam/steamapps/common/SteamVR/drivers/lighthouse/bin/linux64/driver_lighthouse.so",
						RTLD_NOW);
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
	
	vr::EVRInitError initError=_pVrTrackedDeviceProvider->Init(_pDriverLog,this, "/home/upns/agn", "/home/upns/.local/share/Steam/steamapps/common/SteamVR/drivers/lighthouse/bin/linux64");
	if(int(initError))
	{
		std::cout << "Unable to initialize vrTrackedDeviceProvider: " << int(initError) << std::endl;
	}
	
	_pVrTrackedDeviceProvider->LeaveStandby();
	//TrackedDeviceAdded("");
	
	
	
	unsigned int numDevices=_pVrTrackedDeviceProvider->GetTrackedDeviceCount();
	std::cout<<"OpenVRHost: Activating "<<numDevices<<" tracked devices"<<std::endl;
	for(unsigned int i=0;i<numDevices;++i)
		{
		/* Get a handle for the i-th tracked device: */
		vr::ITrackedDeviceServerDriver* driver=deviceStates[_numConnectedDevices].driver=_pVrTrackedDeviceProvider->GetTrackedDeviceDriver(i);
		if(driver!=0)
			{
			/* Activate the tracked device: */
			std::cout<<"OpenVRHost: Activating tracked device "<<i<<" with ID "<<_numConnectedDevices<<std::endl;
			vr::EVRInitError error=driver->Activate(_numConnectedDevices);
			if(error==vr::VRInitError_None)
				{
				/* Query the configuration of a potential HMD: */
				/* Query the device's battery level: */
				vr::ETrackedPropertyError error=vr::TrackedProp_Success;
				if(driver->GetBoolTrackedDeviceProperty(vr::Prop_DeviceProvidesBatteryStatus_Bool,&error)&&error==vr::TrackedProp_Success)
					{
					deviceStates[_numConnectedDevices].batteryLevel=driver->GetFloatTrackedDeviceProperty(vr::Prop_DeviceBatteryPercentage_Float,&error);
					if(error!=vr::TrackedProp_Success)
						deviceStates[_numConnectedDevices].batteryLevel=0.0f; // If there was an error, assume battery is empty
					}
				else
					deviceStates[_numConnectedDevices].batteryLevel=1.0f; // Devices without battery are assumed fully charged
				std::cout<<"OpenVRHost: Battery level of device "<<_numConnectedDevices<<" is "<<deviceStates[_numConnectedDevices].batteryLevel*100.0f<<"%"<<std::endl;
				}
			else
				std::cout<<"OpenVRHost: Unable to activate tracked device "<<_numConnectedDevices<<" due to OpenVR error "<<error<<std::endl;
			
			++_numConnectedDevices;
			}
		else
			std::cout<<"OpenVRHost: Error retrieving tracked device "<<i<<std::endl;
		}
}

bool ServerDriverHost::TrackedDeviceAdded(const char* pchDeviceSerialNumber)
{
	bool result=false;
	std::cout<<"Adding device with serial number "<<pchDeviceSerialNumber<<std::endl;
	
	/* Get a handle for the new connected device: */
	vr::ITrackedDeviceServerDriver* driver=deviceStates[_numConnectedDevices].driver=_pVrTrackedDeviceProvider->FindTrackedDeviceDriver(pchDeviceSerialNumber);
	if(driver!=0)
	{
		/* Activate the connected device: */
		vr::EVRInitError error=driver->Activate(_numConnectedDevices);
		if(error==vr::VRInitError_None)
		{
			/* Query the device's battery level: */
			vr::ETrackedPropertyError error=vr::TrackedProp_Success;
			if(driver->GetBoolTrackedDeviceProperty(vr::Prop_DeviceProvidesBatteryStatus_Bool,&error)&&error==vr::TrackedProp_Success)
			{
				deviceStates[_numConnectedDevices].batteryLevel=driver->GetFloatTrackedDeviceProperty(vr::Prop_DeviceBatteryPercentage_Float,&error);
				if(error!=vr::TrackedProp_Success)
					deviceStates[_numConnectedDevices].batteryLevel=0.0f; // If there was an error, assume battery is empty
			}
			else
				deviceStates[_numConnectedDevices].batteryLevel=1.0f; // Devices without battery are assumed fully charged
			std::cout<<"ServerDriverHost: Battery level of device "<<_numConnectedDevices<<" is "<<deviceStates[_numConnectedDevices].batteryLevel*100.0f<<"%"<<std::endl;
			
			/* Start using the new device: */
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
			std::cout<<"ServerDriverHost: Device "<<unWhichDevice<<" has been connected"<<std::endl;
			state.connected=true;
		}
			_rdy = 1;
			state._rotation = newPose.qRotation;
			state._translation[0] = newPose.vecPosition[0];
			state._translation[1] = newPose.vecPosition[1];
			state._translation[2] = newPose.vecPosition[2];
	}
	else
	{
		if(state.connected)
		{
			std::cout<<"ServerDriverHost: Device "<<unWhichDevice<<" has been disconnected"<<std::endl;
			state.connected=false;
		}
	}
}

void ServerDriverHost::TrackedDevicePropertiesChanged(uint32_t unWhichDevice)
{
	DeviceState& ds=deviceStates[unWhichDevice];
	
	#if 0
	std::cout<<"ServerDriverHost: Changed properties on device "<<unWhichDevice<<std::endl;
	vr::ETrackedPropertyError propError=vr::TrackedProp_Success;
	int32_t deviceClass=ds.driver->GetInt32TrackedDeviceProperty(vr::Prop_DeviceClass_Int32,&propError);
	if(propError==vr::TrackedProp_Success)
	{
		std::cout<<"ServerDriverHost: Device "<<unWhichDevice<<" is of class "<<deviceClass<<std::endl;
	}
	else
		std::cout<<"ServerDriverHost: Error requesting device property"<<std::endl;
	#endif
	

	/* Query the device's battery level: */
	vr::ETrackedPropertyError error=vr::TrackedProp_Success;
	if(ds.driver->GetBoolTrackedDeviceProperty(vr::Prop_DeviceProvidesBatteryStatus_Bool,&error)&&error==vr::TrackedProp_Success)
	{
		ds.batteryLevel=ds.driver->GetFloatTrackedDeviceProperty(vr::Prop_DeviceBatteryPercentage_Float,&error);
		if(error!=vr::TrackedProp_Success)
			ds.batteryLevel=0.0f; // If there was an error, assume battery is empty
	}
	else
		ds.batteryLevel=1.0f; // Devices without battery are assumed fully charged
		
	std::cout<<"ServerDriverHost: Battery level of device "<<unWhichDevice<<" is " <<std::endl;
}

void ServerDriverHost::VsyncEvent(double vsyncTimeOffsetSeconds)
{
	std::cout<<"ServerDriverHost: Vsync occurred at "<<vsyncTimeOffsetSeconds<<std::endl;
}

void ServerDriverHost::TrackedDeviceButtonPressed(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset)
{
	std::cout<<"Button "<<eButtonId<<" pressed on device "<<unWhichDevice<<std::endl;
	if(unWhichDevice>=1U)
	{
		int baseIndex=(int(unWhichDevice)-1)*5;
		int buttonIndex=eButtonId>=32?eButtonId-29:eButtonId;
	}
}

void ServerDriverHost::TrackedDeviceButtonUnpressed(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset)
{
	std::cout<<"Button "<<eButtonId<<" unpressed on device "<<unWhichDevice<<std::endl;
	if(unWhichDevice>=1U)
	{
		int baseIndex=(int(unWhichDevice)-1)*5;
		int buttonIndex=eButtonId>=32?eButtonId-29:eButtonId;
	}
}

void ServerDriverHost::TrackedDeviceButtonTouched(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset)
{
	std::cout<<"Button "<<eButtonId<<" touched on device "<<unWhichDevice<<std::endl;
}

void ServerDriverHost::TrackedDeviceButtonUntouched(uint32_t unWhichDevice,vr::EVRButtonId eButtonId,double eventTimeOffset)
{
	std::cout<<"Button "<<eButtonId<<" untouched on device "<<unWhichDevice<<std::endl;
}

void ServerDriverHost::TrackedDeviceAxisUpdated(uint32_t unWhichDevice,uint32_t unWhichAxis,const vr::VRControllerAxis_t& axisState)
{
	std::cout<<"Axis "<<unWhichAxis<<" updated on device "<<unWhichDevice<<std::endl;
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
	std::cout<<"ServerDriverHost: MC image updated"<<std::endl;
}

vr::IVRSettings* ServerDriverHost::GetSettings(const char* pchInterfaceVersion)
{
	/* Check if the requested API version matches the compiled-in one: */
	if(strcmp(pchInterfaceVersion,vr::IVRSettings_Version)==0)
		return _pVrSettings;
	else
	{
		std::cout<<"ServerDriverHost: Requested settings API version does not match compiled-in version"<<std::endl;
		return 0;
	}
}

void ServerDriverHost::PhysicalIpdSet(uint32_t unWhichDevice,float fPhysicalIpdMeters)
{
	if(deviceStates[unWhichDevice].ipd!=fPhysicalIpdMeters)
	{
		std::cout<<"ServerDriverHost: Physical IPD on device "<<unWhichDevice<<" set to "<<fPhysicalIpdMeters*1000.0f<<"mm"<<std::endl;
		deviceStates[unWhichDevice].ipd = fPhysicalIpdMeters;
	}
}

void ServerDriverHost::ProximitySensorState(uint32_t unWhichDevice,bool bProximitySensorTriggered)
{
	if(proximitySensor!=bProximitySensorTriggered)
	{
		proximitySensor=bProximitySensorTriggered;
		std::cout<<"ServerDriverHost: Proximity sensor on device "<<unWhichDevice<<(bProximitySensorTriggered?" triggered":" untriggered")<<std::endl;
	}
}

void ServerDriverHost::VendorSpecificEvent(uint32_t unWhichDevice,vr::EVREventType eventType,const vr::VREvent_Data_t& eventData,double eventTimeOffset)
{
	std::cout<<"ServerDriverHost: Vendor-specific event "<<eventType<<" received on device "<<unWhichDevice<<std::endl;
}

bool ServerDriverHost::IsExiting(void)
{
	std::cout<<"ServerDriverHost: IsExiting called"<<std::endl;
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

float ServerDriverHost::GetDevicePhsycialIpd(uint32_t unWhichDevice)
{
	return deviceStates[unWhichDevice].ipd;
}

bool ServerDriverHost::IsRdy()
{
	return _rdy;}