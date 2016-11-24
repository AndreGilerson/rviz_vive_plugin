#include <GL/glew.h>
#include <GL/glu.h>
#include <stdio.h>
#include <cstdlib>

#include "shared/lodepng.h"
#include "shared/Matrices.h"
#include "shared/pathtools.h"

//TODO: proper linux compatibility
#ifdef __linux__
#include "shared/linuxcompathack.h"
#endif

#if defined(POSIX)
#include "unistd.h"
#endif

#include "vive_openvr.h"
//#include "vive_display.h"
#include <OGRE/RenderSystems/GL/OgreGLTextureManager.h>

namespace rviz_vive_plugin
{
ViveOpenVR::ViveOpenVR() : _pHMD(0),
_pRenderModels(0),
_strDriver("No Driver"),
_strDisplay("No Display"),
_validPoseCount(0),
_prevValidPoseCount(0),
_validTracking(false)
{
	
}
	
	
bool ViveOpenVR::Initialize()
{
	/*vr::EVRInitError eError = vr::VRInitError_None;
	_pHMD = vr::VR_Init(&eError, vr::VRApplication_Background);
	
	if(eError != vr::VRInitError_None)
	{
		_pHMD = NULL;
		printf("Unable to init VR runtime: %s \n", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
		return false;
	}
	
	_pRenderModels = (vr::IVRRenderModels *)vr::VR_GetGenericInterface( vr::IVRRenderModels_Version, &eError );
	if(!_pRenderModels)
	{
		_pRenderModels = NULL;
		printf("Unable to init VR RenerModel: %s \n", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
		return false;
	}
	
	char buffer[1024];
	uint32_t bufferLen = _pHMD->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd,
		vr::Prop_TrackingSystemName_String, NULL, 0, NULL);
	_pHMD->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String,
		buffer, bufferLen, NULL);
	_strDriver = buffer;
	
	bufferLen = _pHMD->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd,
													vr::Prop_TrackingSystemName_String, NULL, 0, NULL);
	_pHMD->GetStringTrackedDeviceProperty(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String,
										buffer, bufferLen, NULL);
	_strDisplay = buffer;
	
	_pHMD->GetRecommendedRenderTargetSize(&_width, &_height);
	
	_pCompositor = vr::VRCompositor();
	if(!_pCompositor)
	{
		printf("Compositor initialization failed. See log file for details\n");
		return false;
	}
	*/
	return true;
}

void ViveOpenVR::Update()
{
	vr::VREvent_t event;
	while( _pHMD->PollNextEvent( &event, sizeof( event ) ) );
	{
		switch (event.eventType)
		{
		case vr::VREvent_TrackedDeviceActivated:
		{
			std::cout << "Tracked device activated:" << event.trackedDeviceIndex << "\n";
		}
		break;
		case vr::VREvent_TrackedDeviceDeactivated:
		{
			std::cout << "Trackied device deactivated:" << event.trackedDeviceIndex << "\n";
		}
		break;
		case vr::VREvent_TrackedDeviceUpdated:
		{
			std::cout << "Trackied device updated:" << event.trackedDeviceIndex << "\n";
		}
		break;
		}
	}
	if(_validPoseCount != _prevValidPoseCount)
	{
		if(_validPoseCount > 0) _validTracking = true;
		else _validTracking = false;
		std::cout << "Pose count changed: " << _validPoseCount << "\n";
		_prevValidPoseCount = _validPoseCount;
		_matSeatedPose = GetHeadPose();
	}
}

void ViveOpenVR::UpdatePoses()
{
	if ( !_pHMD )
	return;
	_validPoseCount = 0;
	_pCompositor->WaitGetPoses(_trackedDevices, vr::k_unMaxTrackedDeviceCount, NULL, 0 );
	//_pHMD->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseSeated, 0, _trackedDevices, vr::k_unMaxTrackedDeviceCount);
	for ( int nDevice = 0; nDevice < vr::k_unMaxTrackedDeviceCount; ++nDevice )
	{
		if (_trackedDevices[nDevice].bPoseIsValid)
		{
			_validPoseCount++;
			_matDevicePose[nDevice] = _trackedDevices[nDevice].mDeviceToAbsoluteTracking;
		}
	}
}

vr::HmdMatrix34_t ViveOpenVR::GetHeadPose()
{
	return _matDevicePose[vr::k_unTrackedDeviceIndex_Hmd];
}

vr::HmdMatrix34_t ViveOpenVR::GetZeroPose()
{
	return _matSeatedPose;
}

void ViveOpenVR::SubmitTexture(GLuint textureId, uint32_t eye)
{
	vr::VRTextureBounds_t bounds;
	if(eye)		
	{
		bounds = { 0.0f, 0.2f, 0.2f, 0.0f };
	} else
	{
		bounds = { 0.0f, 0.2f, 0.2f, 0.0f };
	}
	const vr::Texture_t tex = { (void*) textureId, vr::API_OpenGL, vr::ColorSpace_Gamma};
	glBindTexture(GL_TEXTURE_2D, textureId);
	vr::EVRCompositorError err;
	err = _pCompositor->Submit(eye ? vr::Eye_Left : vr::Eye_Right, &tex, &bounds);
	if(err != vr::EVRCompositorError::VRCompositorError_None)
	{
		printf("Error submitting texture: %d \n", err);
	}
}

void ViveOpenVR::RenderFrame()
{
	_pCompositor->PostPresentHandoff();
}

bool ViveOpenVR::InitGL()
{
	return true;
}

bool ViveOpenVR::InitCompositor()
{
	return true;
}


std::string ViveOpenVR::GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError)
{
	uint32_t unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, NULL, 0, peError );
	if( unRequiredBufferLen == 0 )
		return "";

	char *pchBuffer = new char[ unRequiredBufferLen ];
	unRequiredBufferLen = pHmd->GetStringTrackedDeviceProperty( unDevice, prop, pchBuffer, unRequiredBufferLen, peError );
	std::string sResult = pchBuffer;
	delete [] pchBuffer;
	return sResult;
}


}