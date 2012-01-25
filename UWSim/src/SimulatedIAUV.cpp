/*
 * SimulatedIAUV.cpp
 *
 *  Created on: 03/05/2010
 *      Author: mprats
 */

#include "SimulatorConfig.h"
#include "SimulatedIAUV.h"
#include "URDFRobot.h"
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>

/** Callback for updating the vehicle lamp according to the vehicle position */
class LightUpdateCallback:public osg::NodeCallback {
	osg::Transform *trackNode;	///< Node that the light must track

public:
	LightUpdateCallback(osg::Transform *trackNode)
	{this->trackNode=trackNode;}

	void operator () (osg::Node *node, osg::NodeVisitor *nv) {
		//update light position to track the node
		osg::LightSource *ls=dynamic_cast<osg::LightSource*>(node);
		osg::Light *l=ls->getLight();
		osg::PositionAttitudeTransform *pat=trackNode->asPositionAttitudeTransform();
		osg::Vec3d pos=pat->getPosition();
		l->setPosition( osg::Vec4f(pos.x(),pos.y(),pos.z()-0.5, 1.f) );

		//call to standard callback
		osg::NodeCallback::operator()(node,nv);
	}
};

/*
SimulatedIAUV::SimulatedIAUV(osgOcean::OceanScene *oscene, arm_t armtype) {
	vehicle=new SimulatedVehicle(oscene, "GIRONA500/girona500.osg");

	if (armtype==PA10)
		arm=new SimulatedPA10(oscene);
	else if (armtype==ARM5)
		arm=new SimulatedArmFromURDF5(oscene);

	baseTransform=NULL;
	if(vehicle->baseTransform!=NULL && arm->baseTransform!=NULL) {
		baseTransform=vehicle->baseTransform;
		baseTransform->addChild(arm->baseTransform);

		//Vehicle frame to Arm base frame transform
		osg::Matrix m=arm->baseTransform->getMatrix();
		if (armtype==PA10) {
			m.makeRotate(M_PI,1,0,0);
		} else if (armtype==ARM5) {
		}
		arm->baseTransform->setMatrix(m);
	}
    	camview=NULL;

	//Set-up a lamp attached to the vehicle
	osg::Light *_light=new osg::Light;
	_light->setLightNum(1);
	_light->setAmbient( osg::Vec4d(1.0f, 1.0f, 1.0f, 1.0f ));
	_light->setDiffuse( osg::Vec4d( 1.0, 1.0, 1.0, 1.0 ) );
	_light->setSpecular(osg::Vec4d( 0.1f, 0.1f, 0.1f, 1.0f ) );
	_light->setDirection(osg::Vec3d(0.0, 0.0, -5.0));
	_light->setSpotCutoff(40.0);
	_light->setSpotExponent(10.0);

	lightSource = new osg::LightSource;
	lightSource->setLight(_light);
	lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
	lightSource->setUpdateCallback(new LightUpdateCallback(baseTransform));
}
*/

SimulatedIAUV::SimulatedIAUV(osgOcean::OceanScene *oscene, Vehicle vehicleChars) {
  name=vehicleChars.name;
  baseTransform=new osg::MatrixTransform;
  urdf=new URDFRobot(oscene,vehicleChars); 

  if(urdf->baseTransform!=NULL /* && arm->baseTransform!=NULL*/ ){
    baseTransform->addChild(urdf->baseTransform);
    baseTransform->setName(vehicleChars.name);
  }

    Vcam vcam;
    camview = new VirtualCamera[vehicleChars.Vcams.size()];
    ncams=vehicleChars.Vcams.size();
    //Add virtual cameras in config file
    int cam=0;
    while(vehicleChars.Vcams.size() > 0){
      OSG_INFO << "Adding a virtual camera..." << std::endl;	
      vcam=vehicleChars.Vcams.front();
      vehicleChars.Vcams.pop_front();
      //Camera frame given wrt vehicle origin frame. 
      //Remember that in opengl/osg, the camera frame is a right-handed system with Z going backwards (opposite to the viewing direction) and Y up.
      osg::Transform *vMc=new osg::PositionAttitudeTransform;	
      ((osg::PositionAttitudeTransform*)vMc)->setPosition(osg::Vec3d(vcam.position[0],vcam.position[1],vcam.position[2]));
      ((osg::PositionAttitudeTransform*)vMc)->setAttitude(osg::Quat(vcam.orientation[0],osg::Vec3d(1,0,0),vcam.orientation[1],osg::Vec3d(0,1,0), vcam.orientation[2],osg::Vec3d(0,0,1) ));
      urdf->link[vcam.link]->asGroup()->addChild(vMc);
      camview[cam++].init(vcam.name, vMc, vcam.resw, vcam.resh, vcam.parameters);
      OSG_INFO << "Done adding a virtual camera..." << std::endl;
    }

  
  //Set-up a lamp attached to the vehicle: TODO
/*
  osg::Light *_light=new osg::Light;
  _light->setLightNum(1);
  _light->setAmbient( osg::Vec4d(1.0f, 1.0f, 1.0f, 1.0f ));
  _light->setDiffuse( osg::Vec4d( 1.0, 1.0, 1.0, 1.0 ) );
  _light->setSpecular(osg::Vec4d( 0.1f, 0.1f, 0.1f, 1.0f ) );
  _light->setDirection(osg::Vec3d(0.0, 0.0, -5.0));
  _light->setSpotCutoff(40.0);
  _light->setSpotExponent(10.0);

  lightSource = new osg::LightSource;
  lightSource->setLight(_light);
  lightSource->setLocalStateSetModes(osg::StateAttribute::ON);
  lightSource->setUpdateCallback(new LightUpdateCallback(baseTransform));
*/
}

/*
void SimulatedIAUV::setVirtualCamera(std::string name, osg::Transform* transform, int width, int height) {
    //Set I-AUV virtual camera

    baseTransform->asGroup()->addChild(transform);

    if (camview==NULL)
    	camview=new VirtualCamera(name, transform, width, height);
}
*/


/** Sets the vehicle position. (x,y,z) given wrt to the world frame. (roll,pitch,yaw) are RPY angles in the local frame */
void SimulatedIAUV::setVehiclePosition(double x, double y, double z, double roll, double pitch, double yaw) {
	osg::Matrixd T, Rx, Ry, Rz, transform;
	T.makeTranslate(x,y,z);
	Rx.makeRotate(roll,1,0,0);
	Ry.makeRotate(pitch,0,1,0);
	Rz.makeRotate(yaw,0,0,1);
	transform=Rz*Ry*Rx*T;
	setVehiclePosition(transform);
}


void SimulatedIAUV::setVehiclePosition(osg::Matrixd m){
	baseTransform->setMatrix(m);
}

