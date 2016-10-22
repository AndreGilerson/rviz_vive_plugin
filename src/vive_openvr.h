#ifndef VIVE_OPENVR_H
#define VIVE_OPENVR_H

#define DEBUGBUILD 0

#include "openvr.h"
#include <string>

namespace rviz_vive_plugin
{

class ViveOpenVR
{
public:
	ViveOpenVR();

	bool Initialize();
	
	std::string GetStrDriver() {return _strDriver;}
	std::string GetStrDisplay() {return _strDisplay;}
	uint32_t GetWidth() {return _width;}
	uint32_t GetHeight() {return _height;}
	
	void Update();
	void UpdatePoses();
	vr::HmdMatrix34_t GetHeadPose();
	vr::HmdMatrix34_t GetZeroPose();
	bool ValidTracking() {return _validTracking;}
	void SubmitTexture(GLuint textureId, uint32_t eye);
	void RenderFrame();

	bool InitGL();
	bool InitCompositor();

protected:
	std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL );
	

	vr::IVRSystem* _pHMD;
	vr::IVRRenderModels* _pRenderModels;
	vr::IVRCompositor* _pCompositor;
	std::string _strDriver;
	std::string _strDisplay;
	
	uint32_t _width;
	uint32_t _height;
	
	vr::TrackedDevicePose_t _trackedDevices[vr::k_unMaxTrackedDeviceCount];
	vr::HmdMatrix34_t _matDevicePose[vr::k_unMaxTrackedDeviceCount];
	vr::HmdMatrix34_t _matSeatedPose;
	uint32_t _validPoseCount;
	uint32_t _prevValidPoseCount;
	bool _validTracking;
};

}

#endif