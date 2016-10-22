#ifndef VIVE_DISPLAY_H
#define VIVE_DISPLAY_H

#ifndef Q_MOC_RUN
#include "rviz/display.h"
#endif

#include <OGRE/OgreTexture.h>
#include <OGRE/RenderSystems/GL/OgreGLTexture.h>


namespace Ogre
{
class SceneNode;
class RenderWindow;
class Viewport;
class Camera;
class CompositorInstance;
}


namespace rviz
{
class BoolProperty;
class RenderWidget;
class DisplayContext;
class TfFrameProperty;
}

#include <QObject>
#include "vive_openvr.h"

namespace rviz_vive_plugin
{

class ViveDisplay: public rviz::Display
{
Q_OBJECT
public:
	ViveDisplay();
	virtual ~ViveDisplay();
	
	//Overrides from rviz::Display
	virtual void onInitialize();
	virtual void update(float wall_dt, float ros_dt);
	virtual void reset();
protected:
	virtual void onEnable();
	virtual void onDisable();
	
	void updateCamera();
private Q_SLOTS:
	void onScreenCountChanged( int newCount );
	
private:
	bool setupOgre();
	Ogre::Matrix4 MatSteamVRtoOgre4(vr::HmdMatrix34_t matrix);
	Ogre::Matrix4 MatSteamVRtoOgre4(vr::HmdMatrix44_t matrix);

	rviz::RenderWidget* _pRenderWidget;
	rviz::DisplayContext* _pDisplayContext;
	Ogre::SceneNode* _pSceneNode;
	Ogre::SceneNode* _pCameraNode;
	Ogre::SceneManager* _pSceneManager;
	Ogre::RenderWindow* _pRenderWindow;
	Ogre::Viewport* _pViewPorts[2];
	Ogre::Camera* _pCameras[2];
	Ogre::CompositorInstance* _pCompositors[2];
	
	Ogre::TexturePtr _renderTextures[2];
	Ogre::RenderTexture* _pRenderTextures[2];
	
	rviz::TfFrameProperty* _pTfFrameProperty;
	
	ViveOpenVR _vive;
	
	bool _doneSetup;
	Ogre::Matrix4 _prevPose;
	bool _firstPose;
	Ogre::Matrix4 _projMatrix[2];
};

}

#endif
