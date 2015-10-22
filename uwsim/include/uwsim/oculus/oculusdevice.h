/*
 * oculusdevice.h
 *
 *  Created on: Jul 03, 2013
 *      Author: Bjorn Blissing
 */

#ifndef _OSG_OCULUSDEVICE_H_
#define _OSG_OCULUSDEVICE_H_

// Include the OculusVR SDK
#include <OVR_CAPI.h>

#include <osg/Geode>
#include <osg/Texture2D>

// Forward declaration
class WarpCameraPreDrawCallback;
class OculusSwapCallback;
class EyeRotationCallback;


class OculusDevice : public osg::Referenced {
	friend class WarpCameraPreDrawCallback;
	friend class OculusSwapCallback;
	friend class EyeRotationCallback;
	
	public:
		enum Eye
		{
			LEFT = 0,
			RIGHT = 1,
			COUNT = 2
		};
		OculusDevice(float nearClip, float farClip, bool useTimewarp = true, const float pixelsPerDisplayPixel = 1.0f, const float worldUnitsPerMetre = 1.0f);

		unsigned int screenResolutionWidth() const;
		unsigned int screenResolutionHeight() const;

		unsigned int renderTargetWidth() const;
		unsigned int renderTargetHeight() const;

		osg::Matrix projectionMatrixCenter() const;
		osg::Matrix projectionMatrixLeft() const;
		osg::Matrix projectionMatrixRight() const;

		osg::Matrix projectionOffsetMatrixLeft() const;
		osg::Matrix projectionOffsetMatrixRight() const;

		osg::Matrix viewMatrixLeft() const;
		osg::Matrix viewMatrixRight() const;

		float nearClip() const { return m_nearClip;	}
		float farClip() const { return m_farClip; }
		bool useTimewarp() const { return m_useTimeWarp; }

		void resetSensorOrientation() const;
		void updatePose(unsigned int frameIndex = 0);
		
		osg::Vec3 position() const { return m_position; }
		osg::Quat orientation() const { return m_orientation;  }
		
		osg::Geode* distortionMesh(Eye eye, osg::Program* program, int x, int y, int w, int h, bool splitViewport=false);
		osg::Camera* createRTTCamera(osg::Texture* texture, OculusDevice::Eye eye, osg::Transform::ReferenceFrame referenceFrame, osg::GraphicsContext* gc = 0) const;
		osg::Camera* createWarpOrthoCamera(double left, double right, double bottom, double top, osg::GraphicsContext* gc=0) const;
		osg::Program* createShaderProgram() const;
		void applyShaderParameters(osg::StateSet* stateSet, osg::Program* program, osg::Texture2D* texture, OculusDevice::Eye eye) const;
		bool attachToWindow(osg::ref_ptr<osg::GraphicsContext> gc);
		void toggleMirrorToWindow();
		void toggleLowPersistence();
		void toggleDynamicPrediction();
		osg::GraphicsContext::Traits* graphicsContextTraits() const;
		bool getHealthAndSafetyDisplayState();
		bool tryDismissHealthAndSafetyDisplay();
	protected:
		~OculusDevice(); // Since we inherit from osg::Referenced we must make destructor protected

		int renderOrder(Eye eye) const;
		osg::Matrixf eyeRotationStart(Eye eye) const;
		osg::Matrixf eyeRotationEnd(Eye eye) const;
		osg::Vec2f eyeToSourceUVScale(Eye eye) const;
		osg::Vec2f eyeToSourceUVOffset(Eye eye) const;

		void printHMDDebugInfo();

		void initializeEyeRenderDesc();
		// Note: this function requires you to run the previous function first.
		void calculateEyeAdjustment();
		// Note: this function requires you to run the previous function first.
		void calculateProjectionMatrices();

		void beginFrameTiming(unsigned int frameIndex = 0);
		void endFrameTiming() const;
		void waitTillTime();

		void trySetProcessAsHighPriority() const;
		void applyExtendedModeSettings() const;
		
		static const std::string m_warpVertexShaderSource;
		static const std::string m_warpWithTimewarpVertexShaderSource;
		static const std::string m_warpFragmentShaderSource;

		ovrHmd m_hmdDevice;
		ovrSizei m_resolution;
		ovrSizei m_renderTargetSize;
		ovrEyeRenderDesc m_eyeRenderDesc[2];
		ovrVector2f m_UVScaleOffset[2][2];
		ovrFrameTiming m_frameTiming;
		ovrPosef m_headPose[2];
		ovrMatrix4f m_timeWarpMatrices[2][2];

		osg::Matrixf m_leftEyeProjectionMatrix;
		osg::Matrixf m_rightEyeProjectionMatrix;
		osg::Vec3f m_leftEyeAdjust;
		osg::Vec3f m_rightEyeAdjust;

		const float m_worldUnitsPerMetre; 
		osg::Vec3 m_position;
		osg::Quat m_orientation;

		float m_nearClip;
		float m_farClip;
		bool m_useTimeWarp;
		bool m_directMode;
	private:
		OculusDevice(const OculusDevice&); // Do not allow copy
		OculusDevice& operator=(const OculusDevice&); // Do not allow assignment operator.
};


class WarpCameraPreDrawCallback : public osg::Camera::DrawCallback
{
public:
	explicit WarpCameraPreDrawCallback(osg::ref_ptr<OculusDevice> device) : m_device(device) {}
	virtual void operator()(osg::RenderInfo& renderInfo) const;
protected:
	osg::observer_ptr<OculusDevice> m_device;
};


class OculusSwapCallback : public osg::GraphicsContext::SwapCallback {
public:
	explicit OculusSwapCallback(osg::ref_ptr<OculusDevice> device) : m_device(device), m_frameIndex(0) {}
	void swapBuffersImplementation(osg::GraphicsContext *gc);
	int frameIndex() const { return m_frameIndex; }
private:
	osg::observer_ptr<OculusDevice> m_device;
	int m_frameIndex;
};


class EyeRotationCallback : public osg::Uniform::Callback
{
public:
	enum Mode
	{
		START,
		END
	};
	EyeRotationCallback(const Mode mode, const OculusDevice* device, const OculusDevice::Eye& eye) : m_mode(mode), m_device(device), m_eye(eye) {}
	virtual void operator()	(osg::Uniform* uniform, osg::NodeVisitor* nv);
protected:
	const Mode m_mode;
	const OculusDevice* m_device;
	const OculusDevice::Eye m_eye;
};

#endif /* _OSG_OCULUSDEVICE_H_ */
