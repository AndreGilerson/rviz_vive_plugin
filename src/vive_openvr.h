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
	void SubmitTexture(GLuint textureId, uint32_t eye);
	void RenderFrame();
private:
	bool InitGL();
	bool InitCompositor();

	std::string GetTrackedDeviceString( vr::IVRSystem *pHmd, vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL );

	vr::IVRSystem* _pHMD;
	vr::IVRRenderModels* _pRenderModels;
	std::string _strDriver;
	std::string _strDisplay;
	
	uint32_t _width;
	uint32_t _height;
};

}

#endif