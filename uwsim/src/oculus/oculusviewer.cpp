/*
 * oculusviewer.cpp
 *
 *  Created on: Jun 30, 2013
 *      Author: Jan Ciger & Bj�rn Blissing
 */

#include "uwsim/oculus/oculusviewer.h"

#include "uwsim/oculus/oculusupdateslavecallback.h"

#include <iostream>

/* Public functions */
void OculusViewer::traverse(osg::NodeVisitor& nv)
{
	if (!m_configured) {
		configure();
	}

	osg::Group::traverse(nv);
}

/* Protected functions */
void OculusViewer::configure()
{
	osg::ref_ptr<osg::GraphicsContext> gc =  m_view->getCamera()->getGraphicsContext();
	
	// Attach a callback to detect swap
	m_swapCallback = new OculusSwapCallback(m_device);
	gc->setSwapCallback(m_swapCallback);

	osg::ref_ptr<osg::Camera> camera = m_view->getCamera();
	camera->setName("Main");

	// Add health and safety warning
	m_warning = new OculusHealthAndSafetyWarning();
	m_view->addEventHandler(new OculusWarningEventHandler(m_device.get(), m_warning));
	this->addChild(m_warning->getGraph());
	// Start timer
	m_device->getHealthAndSafetyDisplayState();

	const int textureWidth = m_device->renderTargetWidth()/2;
	const int textureHeight = m_device->renderTargetHeight();
	// master projection matrix
	camera->setProjectionMatrix(m_device->projectionMatrixCenter());
	// Create textures for RTT cameras
	osg::ref_ptr<osg::Texture2D> textureLeft = new osg::Texture2D;
	textureLeft->setTextureSize(textureWidth, textureHeight);
	textureLeft->setInternalFormat(GL_RGBA);
	osg::ref_ptr<osg::Texture2D> textureRight = new osg::Texture2D;
	textureRight->setTextureSize(textureWidth, textureHeight);
	textureRight->setInternalFormat(GL_RGBA);
	// Create RTT cameras and attach textures
	m_cameraRTTLeft = m_device->createRTTCamera(textureLeft, OculusDevice::LEFT, osg::Camera::RELATIVE_RF, gc);
	m_cameraRTTRight = m_device->createRTTCamera(textureRight, OculusDevice::RIGHT, osg::Camera::RELATIVE_RF, gc);
	m_cameraRTTLeft->setName("LeftRTT");
	m_cameraRTTRight->setName("RightRTT");

	// Create warp ortho camera
	osg::ref_ptr<osg::Camera> cameraWarp = m_device->createWarpOrthoCamera(0.0, 1.0, 0.0, 1.0, gc);
	cameraWarp->setName("WarpOrtho");
	cameraWarp->setViewport(new osg::Viewport(0, 0, m_device->screenResolutionWidth(), m_device->screenResolutionHeight()));

	// Create shader program
	osg::ref_ptr<osg::Program> program = m_device->createShaderProgram();

	// Create distortionMesh for each camera
	osg::ref_ptr<osg::Geode> leftDistortionMesh = m_device->distortionMesh(OculusDevice::LEFT, program, 0, 0, textureWidth, textureHeight);
	cameraWarp->addChild(leftDistortionMesh);

	osg::ref_ptr<osg::Geode> rightDistortionMesh = m_device->distortionMesh(OculusDevice::RIGHT, program, 0, 0, textureWidth, textureHeight);
	cameraWarp->addChild(rightDistortionMesh);

	// Add pre draw camera to handle time warp
	cameraWarp->setPreDrawCallback(new WarpCameraPreDrawCallback(m_device));

	// Attach shaders to each distortion mesh
	osg::StateSet* leftEyeStateSet = leftDistortionMesh->getOrCreateStateSet();
	osg::StateSet* rightEyeStateSet = rightDistortionMesh->getOrCreateStateSet();

	m_device->applyShaderParameters(leftEyeStateSet, program.get(), textureLeft.get(), OculusDevice::LEFT);
	m_device->applyShaderParameters(rightEyeStateSet, program.get(), textureRight.get(), OculusDevice::RIGHT);

	// Add RTT cameras as slaves, specifying offsets for the projection
	m_view->addSlave(m_cameraRTTLeft.get(),
		m_device->projectionOffsetMatrixLeft(),
		m_device->viewMatrixLeft(),
		true);
	m_view->getSlave(0)._updateSlaveCallback = new OculusUpdateSlaveCallback(OculusUpdateSlaveCallback::LEFT_CAMERA, m_device.get(), m_swapCallback.get(), m_warning.get());

	m_view->addSlave(m_cameraRTTRight.get(),
		m_device->projectionOffsetMatrixRight(),
		m_device->viewMatrixRight(),
		true);
	m_view->getSlave(1)._updateSlaveCallback = new OculusUpdateSlaveCallback(OculusUpdateSlaveCallback::RIGHT_CAMERA, m_device.get(), m_swapCallback.get(), 0);

	// Use sky light instead of headlight to avoid light changes when head movements
	m_view->setLightingMode(osg::View::SKY_LIGHT);

	// Add warp camera as slave
	m_view->addSlave(cameraWarp, false);
	m_view->setName("Oculus");

	m_configured = true;
}
