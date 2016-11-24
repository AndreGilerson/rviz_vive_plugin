#include "vive_display.h"
//#include "vive_openvr.h"

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreCompositorManager.h>
#include <OGRE/OgreCompositorInstance.h>
#include <OGRE/OgreCompositionTargetPass.h>
#include <OGRE/OgreCompositionPass.h>
#include <OGRE/OgreRenderTexture.h>
#include <OGRE/OgreRenderSystem.h>

#include <rviz/properties/tf_frame_property.h>

#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h>

#include <QApplication>
#include <QDesktopWidget>

#include <cstdlib>
#include <dlfcn.h>

namespace rviz_vive_plugin
{	
	
const float g_defaultNearClip = 0.01f;
const float g_defaultFarClip = 1000.0f;
const float g_defaultIPD = 0.064f;
const Ogre::ColourValue g_defaultViewportColour(97 / 255.0f, 97 / 255.0f, 200 / 255.0f);
const float g_defaultProjectionCentreOffset = 0.14529906f;
const float g_defaultDistortion[4] = {1.0f, 0.22f, 0.24f, 0.0f};
const float g_defaultChromAb[4] = {0.996, -0.004, 1.014, 0.0f};
	
ViveDisplay::ViveDisplay() : _pRenderWidget(0),
_pDisplayContext(0),
_pSceneNode(0),
_pSceneManager(0),
_pRenderWindow(0)
{
	_pViewPorts[0] = 0;
	_pViewPorts[1] = 0;
	_pCameras[0] = 0;
	_pCameras[1] = 0;
}

ViveDisplay::~ViveDisplay()
{
	_doneSetup = false;
}

void ViveDisplay::onInitialize()
{
	_pTfFrameProperty = new rviz::TfFrameProperty( "Target Frame", "<Fixed Frame>",
	"Tf frame that the Oculus camera should follow.", this, context_->getFrameManager(), true );
	_pTranslationProperty = new rviz::BoolProperty("Fixed Position", false, 
	"If checked will ignore translation of HTC Vive", this);
	_pOrientationProperty = new rviz::BoolProperty("Fixed Orientation", false,
	"If checked will ignore orientation of HTC Vive", this);
	
	_pSceneManager = scene_manager_; //Random global pointer by RVIZ
	_pDisplayContext = context_;
	_pRenderWidget = new rviz::RenderWidget(rviz::RenderSystem::get());
	_pRenderWidget->setWindowTitle("Vive View");
	_pRenderWidget->setParent(_pDisplayContext->getWindowManager()->getParentWindow());
	_pRenderWidget->setWindowFlags( Qt::Window | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowMaximizeButtonHint );
	_pRenderWidget->setVisible(true);
	//_pRenderWidget->showFullScreen();
	
	_pRenderWindow = _pRenderWidget->getRenderWindow();
	_pRenderWindow->setVisible(true);
	_pRenderWindow->setAutoUpdated(false);
	_pSceneNode = _pSceneManager->getRootSceneNode()->createChildSceneNode();
	
	std::cout << QApplication::desktop()->screenCount() << std::endl;
	int id = 0;
	for(int i = 0; i < QApplication::desktop()->screenCount(); i++)
	{
		std::cout << "Width:" << QApplication::desktop()->screenGeometry(i).width() << std::endl;
		if(QApplication::desktop()->screenGeometry(i).width() > 2000) id = i;
	}
	
	_pViveRenderWidget = new rviz::RenderWidget(rviz::RenderSystem::get());
	_pViveRenderWidget->setGeometry(QApplication::desktop()->screenGeometry(id));
	_pViveRenderWindow = _pViveRenderWidget->getRenderWindow();
	_pViveRenderWidget->showFullScreen();
	//_vive.Initialize();

	setupOgre();
}

void ViveDisplay::update(float wall_dt, float ros_dt)
{
	sev.Update();
	
	Ogre::Vector3 pos;
	Ogre::Quaternion ori;
	
	_pDisplayContext->getFrameManager()->getTransform( _pTfFrameProperty->getStdString(),
													ros::Time(), pos, ori );
    Ogre::Camera *cam = _pDisplayContext->getViewManager()->getCurrent()->getCamera();
    pos = cam->getDerivedPosition();
    ori = cam->getDerivedOrientation();
	
	_pSceneNode->setOrientation(ori);
	_pSceneNode->setPosition(pos);
	
	if(sev.IsRdy())
	{
		pos = sev.GetDeviceTranslation(0);
		ori = sev.GetDeviceRotation(0);
	
		_pCameraNode->setPosition(pos);
		_pCameraNode->setOrientation(ori);
		_pCameraNode->roll(Ogre::Radian(M_PI));
		
		_pCameras[0]->setPosition((- 1) * (sev.GetDevicePhsycialIpd(0))*0.5, 0, 0);
		_pCameras[1]->setPosition((sev.GetDevicePhsycialIpd(0))*0.5, 0, 0);
	}
	
	if(_doneSetup)
	{
		_pRenderTextures[0]->update();
		_pRenderTextures[1]->update();
	}
	_pRenderWindow->update(true);
	_pViveRenderWindow->update(true);
}

void ViveDisplay::reset()
{}

bool ViveDisplay::setupOgre()
{
	Ogre::LogManager::getSingleton().logMessage("Oculus: Setting up Ogre");
	if (_pSceneNode)
		_pCameraNode = _pSceneNode->createChildSceneNode("StereoCameraNode");
	else
		_pCameraNode = _pSceneManager->getRootSceneNode()->createChildSceneNode("StereoCameraNode");
	
	_pCameras[0] = _pSceneManager->createCamera("CameraLeft");
	_pCameras[1] = _pSceneManager->createCamera("CameraRight");
	
	
	/*_renderTextures[0]= Ogre::TextureManager::getSingleton().createManual(
		"RenderTexture1", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,
		_vive.GetWidth(), _vive.GetHeight(), 0, Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET);
	_renderTextures[1] = Ogre::TextureManager::getSingleton().createManual(
		"RenderTexture2", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,
		_vive.GetWidth(), _vive.GetHeight(), 0, Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET);*/
		
		_renderTextures[0]= Ogre::TextureManager::getSingleton().createManual(
		"RenderTexture1", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,
		1080, 1200, 0, Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET);
	_renderTextures[1] = Ogre::TextureManager::getSingleton().createManual(
		"RenderTexture2", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
		Ogre::TEX_TYPE_2D,
		1080, 1200, 0, Ogre::PF_R8G8B8A8, Ogre::TU_RENDERTARGET);
	
	Ogre::MaterialPtr matLeft = Ogre::MaterialManager::getSingleton().getByName("Ogre/Compositor/Oculus");
	
	for (int i = 0; i < 2; ++i)
	{
		_pCameras[i]->detachFromParent();
		_pCameraNode->attachObject(_pCameras[i]);
		
		_pCameras[i]->setNearClipDistance(0.01f);
		_pCameras[i]->setFarClipDistance(1000.0f);
		_pCameras[i]->setAutoAspectRatio(true);
		_pCameras[i]->setPosition((i * 2 - 1) * (g_defaultIPD)*1.5, 0, 0);

		_pViewPorts[i] = _pRenderWindow->addViewport(_pCameras[i], i, 0.5f * i, 0, 0.5f, 1.0f);
		_pViveRenderWindow->addViewport(_pCameras[i], i, 0.5f * i, 0, 0.5f, 1.0f);
		_pViewPorts[i]->setBackgroundColour(g_defaultViewportColour);
		
		_pRenderTextures[i] = _renderTextures[i]->getBuffer()->getRenderTarget();
		_pRenderTextures[i]->addViewport(_pCameras[i]);
		_pRenderTextures[i]->getViewport(0)->setClearEveryFrame(true);
		_pRenderTextures[i]->getViewport(0)->setBackgroundColour(Ogre::ColourValue::Black);
		_pRenderTextures[i]->getViewport(0)->setOverlaysEnabled(false);
		_pRenderTextures[i]->setAutoUpdated(false);
	}
	Ogre::LogManager::getSingleton().logMessage("Oculus: Oculus setup completed successfully");
	_doneSetup = true;
	return true;
}

Ogre::Matrix4 ViveDisplay::MatSteamVRtoOgre4(vr::HmdMatrix34_t matrix)
{
	return Ogre::Matrix4 (
		matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
		matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
		matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
		0.0, 0.0, 0.0, 1.0f
		);
}

Ogre::Matrix4 ViveDisplay::MatSteamVRtoOgre4(vr::HmdMatrix44_t matrix)
{
	return Ogre::Matrix4 (
		matrix.m[0][0], matrix.m[0][1], matrix.m[0][2], matrix.m[0][3],
		matrix.m[1][0], matrix.m[1][1], matrix.m[1][2], matrix.m[1][3],
		matrix.m[2][0], matrix.m[2][1], matrix.m[2][2], matrix.m[2][3],
		matrix.m[3][0], matrix.m[3][1], matrix.m[3][0], matrix.m[3][3]
		);
}

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_vive_plugin::ViveDisplay, rviz::Display)
