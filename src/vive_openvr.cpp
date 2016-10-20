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
#include "vive_display.h"
#include <OGRE/RenderSystems/GL/OgreGLTextureManager.h>

namespace rviz_vive_plugin
{
ViveOpenVR::ViveOpenVR() : _pHMD(0),
_pRenderModels(0),
_strDriver("No Driver"),
_strDisplay("No Display")
{
	
}
	
	
bool ViveOpenVR::Initialize()
{
	/*if(SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
	{
		printf("%s - SDL could not initialize! SDL Error: %s\n", __FUNCTION__, SDL_GetError());
		return false;
	}*/
	
	vr::EVRInitError eError = vr::VRInitError_None;
	_pHMD = vr::VR_Init(&eError, vr::VRApplication_Scene);
	
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
	
	if(!vr::VRCompositor())
	{
		printf("Compositor initialization failed. See log file for details\n");
		return false;
	}
	
	_pHMD->CaptureInputFocus();
	vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
	vr::VRCompositor()->WaitGetPoses(m_rTrackedDevicePose, vr::k_unMaxTrackedDeviceCount, NULL, 0);
	
	return true;
}

void ViveOpenVR::Update()
{
	vr::VREvent_t event;
	while( _pHMD->PollNextEvent( &event, sizeof( event ) ) );
}

void ViveOpenVR::SubmitTexture(GLuint textureId, uint32_t eye)
{
	const vr::Texture_t tex = { (void*) textureId, vr::API_OpenGL, vr::ColorSpace_Gamma};
	glBindTexture(GL_TEXTURE_2D, textureId);
	vr::EVRCompositorError err;
	err = vr::VRCompositor()->Submit(eye ? vr::Eye_Left : vr::Eye_Right, &tex);
	if(err != vr::EVRCompositorError::VRCompositorError_None)
	{
		printf("Error submitting texture: %d \n", err);
	}
}

void ViveOpenVR::RenderFrame()
{
	vr::VRCompositor()->PostPresentHandoff();
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