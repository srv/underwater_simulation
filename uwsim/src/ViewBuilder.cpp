/* 
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Mario Prats
 *     Javier Perez
 */

#include <uwsim/ViewBuilder.h>
#include <uwsim/UWSimUtils.h>
#include <uwsim/TextHUD.h>
#include <uwsim/EventHandler.h>
#include <uwsim/OculusCameraManipulator.h>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/NodeTrackerManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/GUIEventHandler>
#include <osg/Notify>

#include <osgWidget/Util>
#include <osgWidget/WindowManager>
#include <osgWidget/ViewerEventHandlers>
#include <osg/GraphicsContext>

#include <osg/Camera>
#include <osg/ShapeDrawable>

osg::Camera* createHUDCamera( )
{
	osg::ref_ptr<osg::Camera> camera = new osg::Camera;
	camera->setName("ShieldCamera");
	camera->setReferenceFrame( osg::Transform::ABSOLUTE_RF );
	camera->setClearMask( GL_DEPTH_BUFFER_BIT );
	camera->setRenderOrder( osg::Camera::POST_RENDER );
	camera->setViewMatrixAsLookAt(osg::Vec3(0.0f,-5.0f,5.0f), osg::Vec3(),osg::Vec3(0.0f,1.0f,1.0f));
	return camera.release();
}

ViewBuilder::ViewBuilder(ConfigFile &config, SceneBuilder *scene_builder, int *argc, char **argv)
{
  arguments.reset(new osg::ArgumentParser(argc, argv));
  init(config, scene_builder);
}

ViewBuilder::ViewBuilder(ConfigFile &config, SceneBuilder *scene_builder, boost::shared_ptr<osg::ArgumentParser> args)
{
  arguments = args;
  init(config, scene_builder);
}

ViewBuilder::ViewBuilder(ConfigFile &config, SceneBuilder *scene_builder)
{
  int argc = 0;
  char **argv = NULL;
  arguments.reset(new osg::ArgumentParser(&argc, argv));
  init(config, scene_builder);
}

bool ViewBuilder::init(ConfigFile &config, SceneBuilder *scene_builder)
{
  bool disableTextures = false;
  if (arguments->read("--disableTextures"))
    disableTextures = true;

  float reswidth = config.resw, resheight = config.resh;
  while (arguments->read("--resw", reswidth))
    ;
  while (arguments->read("--resh", resheight))
    ;

  bool freeMotion = config.freeMotion;
  if (arguments->read("--freeMotion"))
  {
    freeMotion = true;
  }

  oculus = config.oculus;
  if (arguments->read("--oculus"))
  {
    oculus = true;
  }

  bool windshield = config.windshield;
  if (arguments->read("--windshield"))
  {
    windshield = true;
  }

  fullScreenNum = -1;
  if (!arguments->read("--fullScreen", fullScreenNum) && arguments->read("--fullScreen"))
  {
    fullScreenNum = 0;
  }
  //is screen number is higher than any available screen, set it to the last one
  if (fullScreenNum + 1 >= osg::GraphicsContext::getWindowingSystemInterface()->getNumScreens())
    fullScreenNum = osg::GraphicsContext::getWindowingSystemInterface()->getNumScreens() - 1;
  if (fullScreenNum >= 0)
  {
    osg::GraphicsContext::ScreenSettings settings;
    osg::GraphicsContext::ScreenIdentifier screen_id(fullScreenNum);
    osg::GraphicsContext::getWindowingSystemInterface()->getScreenSettings(screen_id, settings);
    if (settings.width > 0 && settings.height > 0)
    {
      reswidth = settings.width;
      resheight = settings.height;
    }
  }

  OSG_INFO << "Setting main viewer" << std::endl;
  viewer = new osgViewer::Viewer();
  viewer->addEventHandler(new osgViewer::StatsHandler);
  viewer->addEventHandler(scene_builder->getScene()->getOceanSceneEventHandler());
  viewer->addEventHandler(scene_builder->getScene()->getOceanSurface()->getEventHandler());
  osg::ref_ptr < TextHUD > hud = new TextHUD;

  //Initialize viewer
  if (!oculus)
  { 
    //We don't use Oculus with the UWSim.

    //Set main camera position, lookAt, and other params
    if (freeMotion)
    {
      osg::ref_ptr < osgGA::TrackballManipulator > tb = new osgGA::TrackballManipulator;
      tb->setHomePosition(osg::Vec3f(config.camPosition[0], config.camPosition[1], config.camPosition[2]),
                          osg::Vec3f(config.camLookAt[0], config.camLookAt[1], config.camLookAt[2]), osg::Vec3f(0, 0, 1));
      viewer->setCameraManipulator(tb);
    }

    findRoutedNode findRN(config.vehicleToTrack);
    findRN.find(scene_builder->getScene()->getScene());

    osg::ref_ptr < osg::Node > first = findRN.getFirst();
    if (first.valid())
    {
      //target to track found
      osg::ref_ptr < osg::Node > emptyNode = new osg::Node;
      osg::ref_ptr < osgGA::NodeTrackerManipulator > ntm = new osgGA::NodeTrackerManipulator;
      first->asGroup()->addChild(emptyNode);
      ntm->setTrackNode(emptyNode);
      ntm->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);
      ntm->setHomePosition(osg::Vec3d(config.camPosition[0], config.camPosition[1], config.camPosition[2]),
                           osg::Vec3d(config.camLookAt[0], config.camLookAt[1], config.camLookAt[2]),
                           osg::Vec3d(0, 0, 1));
      viewer->setCameraManipulator(ntm);
    }
    else
    {
      //Target object not found, free camera
      osg::ref_ptr < osgGA::TrackballManipulator > tb = new osgGA::TrackballManipulator;
      tb->setHomePosition(osg::Vec3f(config.camPosition[0], config.camPosition[1], config.camPosition[2]),
                          osg::Vec3f(config.camLookAt[0], config.camLookAt[1], config.camLookAt[2]),
                          osg::Vec3f(0, 0, 1));
      viewer->setCameraManipulator(tb);
    }

    viewer->addEventHandler(new osgViewer::HelpHandler);
    viewer->getCamera()->setName("MainCamera");

    //Main camera projection parameters: view angle (fov), aspect ratio, fustrum near and far
    if (config.camNear != -1 && config.camFar != -1)
    {
      OSG_INFO << "Setting custom near/far planes to " << config.camNear << " " << config.camFar << std::endl;
      viewer->getCamera()->setComputeNearFarMode(osgUtil::CullVisitor::DO_NOT_COMPUTE_NEAR_FAR);
      viewer->getCamera()->setProjectionMatrixAsPerspective(config.camFov, config.camAspectRatio, config.camNear,
                                                            config.camFar);
    }
    else
    {
      OSG_INFO << "Setting custom near/far planes to auto" << std::endl;
      viewer->getCamera()->setProjectionMatrixAsPerspective(config.camFov, config.camAspectRatio, 1, 10);
    }

    viewer->setSceneData(scene_builder->getRoot());

    // plnc: from here common for oculus and non-oculus
    OSG_INFO << "Starting window manager..." << std::endl;
    wm = new osgWidget::WindowManager(viewer, reswidth, resheight, 0xF0000000, 0);

    OSG_INFO << "Setting widgets..." << std::endl;
    wm->setPointerFocusMode(osgWidget::WindowManager::PFM_SLOPPY);
    std::vector < osg::ref_ptr<osgWidget::Window> > camWidgets;
    int dispx = 0;
    int ncamwidgets = 0;
    for (unsigned int j = 0; j < scene_builder->iauvFile.size(); j++)
    {
      for (unsigned int i = 0; i < scene_builder->iauvFile[j]->getNumCams(); i++)
      {
        if (scene_builder->iauvFile[j]->camview[i].widget)
        {
          camWidgets.push_back(scene_builder->iauvFile[j]->camview[i].getWidgetWindow());
          camWidgets[ncamwidgets]->setX(dispx);
          camWidgets[ncamwidgets]->setY(0);
          dispx += scene_builder->iauvFile[j]->camview[i].width + 20;
          wm->addChild(camWidgets[ncamwidgets]);
          camWidgets[ncamwidgets]->hide();
          ncamwidgets++;
        }
      }
    }

    viewer->addEventHandler(new SceneEventHandler(camWidgets, hud.get(), scene_builder->getScene(), scene_builder->ROSInterfaces, &config));

    for (unsigned int i = 0; i < scene_builder->realcams.size(); i++)
    {
      wm->addChild(scene_builder->realcams[i]->getWidgetWindow());
    }

    osg::ref_ptr < osg::Group > appgroup = new osg::Group();
    osg::ref_ptr < osg::Camera > appcamera = wm->createParentOrthoCamera();

    appgroup->addChild(appcamera);

    if (scene_builder->getRoot())
      appgroup->addChild(scene_builder->getRoot());

    viewer->addEventHandler(new osgWidget::MouseHandler(wm));
    viewer->addEventHandler(new osgWidget::KeyboardHandler(wm));
    viewer->addEventHandler(new osgWidget::ResizeHandler(wm, appcamera));
    viewer->addEventHandler(new osgWidget::CameraSwitchHandler(wm, appcamera));
    viewer->addEventHandler(new osgViewer::WindowSizeHandler());
    viewer->addEventHandler(new osgGA::StateSetManipulator(viewer->getCamera()->getOrCreateStateSet()));


    //If --disableTextures, force disabling textures
    if (disableTextures)
    {
      for (unsigned int ii = 0; ii < 4; ii++)
      {
    #if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE)
        viewer->getCamera()->getOrCreateStateSet()->setTextureMode(
            ii, GL_TEXTURE_1D, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
    #endif
        viewer->getCamera()->getOrCreateStateSet()->setTextureMode(
            ii, GL_TEXTURE_2D, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
        viewer->getCamera()->getOrCreateStateSet()->setTextureMode(
            ii, GL_TEXTURE_3D, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
        viewer->getCamera()->getOrCreateStateSet()->setTextureMode(
            ii, GL_TEXTURE_RECTANGLE, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
        viewer->getCamera()->getOrCreateStateSet()->setTextureMode(
            ii, GL_TEXTURE_CUBE_MAP, osg::StateAttribute::OVERRIDE | osg::StateAttribute::OFF);
      }
    }

    wm->resizeAllWindows();
    
    viewer->setSceneData(appgroup);

    for (unsigned int j = 0; j < scene_builder->iauvFile.size(); j++)
      for (unsigned int i = 0; i < scene_builder->iauvFile[j]->devices->all.size(); i++)
        scene_builder->iauvFile[j]->devices->all.at(i)->setViewBuilder(this);

    // plnc: to here common for oculus and non-oculus
  }
  else
  {
    // OCULUS RIFT: Open the HMD
    findRoutedNode findRN(config.vehicleToTrack);
    findRN.find(scene_builder->getScene()->getScene());
    osg::ref_ptr < osg::Node > first = findRN.getFirst();
    if (first.valid())
    {
      //target to track found
      osg::ref_ptr < osg::Node > emptyNode = new osg::Node;
      osg::ref_ptr < osgGA::NodeTrackerManipulator > ntm = new osgGA::NodeTrackerManipulator;
      first->asGroup()->addChild(emptyNode);
      ntm->setTrackNode(emptyNode);
      ntm->setTrackerMode(osgGA::NodeTrackerManipulator::NODE_CENTER);
      ntm->setHomePosition(osg::Vec3d(config.camPosition[0], config.camPosition[1], config.camPosition[2]),
                           osg::Vec3d(config.camLookAt[0], config.camLookAt[1], config.camLookAt[2]),
                           osg::Vec3d(0, 0, 1));
      viewer->setCameraManipulator(ntm);
    } else {
      // Create Trackball manipulator
      osg::ref_ptr<osgGA::CameraManipulator> ocm = new osgGA::TrackballManipulator;
      ocm->setHomePosition(osg::Vec3f(config.camPosition[0], config.camPosition[1], config.camPosition[2]),
                           osg::Vec3f(config.camLookAt[0], config.camLookAt[1], config.camLookAt[2]), osg::Vec3f(0, 0, 1));
      //ocm->setByMatrix(scene_builder->iauvFile[0]->baseTransform->getMatrix());
      viewer->setCameraManipulator(ocm);
    }

    // Oculus device
    float nearClip = 0.01f;
    float farClip = 10000.0f;
    float pixelsPerDisplayPixel = 1.0;
    bool useTimewarp = true;
    float worldUnitsPerMetre = 1.0f;
		oculusDevice = new OculusDevice(nearClip, farClip, useTimewarp, pixelsPerDisplayPixel, worldUnitsPerMetre);

    // Get the suggested context traits
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = oculusDevice->graphicsContextTraits();

		// Create a graphic context based on our desired traits
		osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits);
		if (!gc) {
			osg::notify(osg::NOTICE) << "Error, GraphicsWindow has not been created successfully" << std::endl;
			return 1;
		}

    // Attach to window, needed for direct mode
    oculusDevice->attachToWindow(gc);

		if (gc.valid()) {
			gc->setClearColor(osg::Vec4(0.2f, 0.2f, 0.4f, 1.0f));
			gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		}

		viewer->getCamera()->setGraphicsContext(gc);
    viewer->getCamera()->setViewport(0, 0, traits->width, traits->height);

    // Disable automatic computation of near and far plane
    viewer->getCamera()->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
    
    viewer->realize();

    oculusViewer = new OculusViewer(viewer, oculusDevice);
    oculusViewer->addChild(scene_builder->getRoot());
    viewer->setSceneData(oculusViewer);
    
    // Add Oculus Keyboard Handler to only one view
    viewer->addEventHandler(new OculusEventHandler(oculusDevice));
	}

  return true;

}
void ViewBuilder::init()
{
  OSG_INFO << "Creating application..." << std::endl;

  if(!oculus) // plnc
  {
    if (fullScreenNum>=0)
      viewer->setUpViewOnSingleScreen(fullScreenNum);
    else
      viewer->setUpViewInWindow(50, 50, static_cast<int>(wm->getWidth()), static_cast<int>(wm->getHeight()));
  }
}
