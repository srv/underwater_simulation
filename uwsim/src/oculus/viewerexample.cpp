#include <ros/ros.h>

#include <osgDB/ReadFile>
#include <osgGA/TrackballManipulator>

#include "uwsim/oculus/oculusviewer.h"
#include "uwsim/oculus/oculuseventhandler.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "viewer_example");
  
  // use an ArgumentParser object to manage the program arguments.
  osg::ArgumentParser arguments(&argc, argv);

  // read the scene from the list of file specified command line arguments.
  osg::ref_ptr<osg::Node> loadedModel = osgDB::readNodeFiles(arguments);

  // if not loaded assume no arguments passed in, try use default cow model instead.
  if (!loadedModel) loadedModel = osgDB::readNodeFile("/home/ros/cow.osgt");

  // Still no loaded model, then exit
  if (!loadedModel) {
    osg::notify(osg::ALWAYS) << "No model could be loaded and didn't find cow.osgt, terminating.." << std::endl;
    return 0;
  }

  // Create Trackball manipulator
  osg::ref_ptr<osgGA::CameraManipulator> cameraManipulator = new osgGA::TrackballManipulator;
  const osg::BoundingSphere& bs = loadedModel->getBound();

  if (bs.valid()) {
    // Adjust view to object view
    osg::Vec3 modelCenter = bs.center();
    osg::Vec3 eyePos = bs.center() + osg::Vec3(0, bs.radius(), 0);
    cameraManipulator->setHomePosition(eyePos, modelCenter, osg::Vec3(0, 0, 1));
  }

  // Open the HMD
  float nearClip = 0.01f;
  float farClip = 10000.0f;
  float pixelsPerDisplayPixel = 1.0;
  bool useTimewarp = true;
  float worldUnitsPerMetre = 1.0f;
  osg::ref_ptr<OculusDevice> oculusDevice = new OculusDevice(nearClip, farClip, useTimewarp, pixelsPerDisplayPixel, worldUnitsPerMetre);

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

  osg::ref_ptr<osgViewer::Viewer> viewer = new osgViewer::Viewer();
  viewer->getCamera()->setGraphicsContext(gc);
  viewer->getCamera()->setViewport(0, 0, traits->width, traits->height);

  // Disable automatic computation of near and far plane
  viewer->getCamera()->setComputeNearFarMode( osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR );
  viewer->setCameraManipulator(cameraManipulator);
  viewer->realize();
  
  osg::ref_ptr<OculusViewer> oculusViewer = new OculusViewer(viewer, oculusDevice);
  oculusViewer->addChild(loadedModel);
  viewer->setSceneData(oculusViewer);
  // Add statistics handler
  viewer->addEventHandler(new osgViewer::StatsHandler);
  // Add Oculus Keyboard Handler to only one view
  viewer->addEventHandler(new OculusEventHandler(oculusDevice));

  // Start Viewer
  viewer->run();

  ros::spin();
  return 0;
}

