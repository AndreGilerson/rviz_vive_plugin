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


#ifndef VIVE_DISPLAY_H
#define VIVE_DISPLAY_H

#ifndef Q_MOC_RUN
#include "rviz/display.h"
#endif

#include <QObject>

#include <OGRE/OgreTexture.h>
#include <OGRE/RenderSystems/GL/OgreGLTexture.h>

#include "openvr_driver.h"

#include "vive_serverdriverhost.h"

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
	
private:
	bool setupOgre();
	Ogre::Matrix4 MatSteamVRtoOgre4(vr::HmdMatrix34_t matrix);
	Ogre::Matrix4 MatSteamVRtoOgre4(vr::HmdMatrix44_t matrix);

	rviz::RenderWidget* _pRenderWidget;
	rviz::RenderWidget* _pViveRenderWidget;
	rviz::DisplayContext* _pDisplayContext;
	Ogre::SceneNode* _pSceneNode;
	Ogre::SceneNode* _pCameraNode;
	Ogre::SceneManager* _pSceneManager;
	Ogre::RenderWindow* _pRenderWindow;
	Ogre::RenderWindow* _pViveRenderWindow;
	Ogre::Camera* _pCameras[2];
	
	Ogre::TexturePtr _renderTextures[2];
	Ogre::RenderTexture* _pRenderTextures[2];
	
	rviz::BoolProperty* _pTranslationProperty;
	rviz::BoolProperty* _pOrientationProperty;
	
	bool _doneSetup;
	
	ServerDriverHost sev;
};

}

#endif
