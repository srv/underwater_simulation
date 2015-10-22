/* 
 * Copyright (c) 2013 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 * 
 * Contributors:
 *     Juan Carlos García
 */

#include <osgGA/CameraManipulator>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <osg/MatrixTransform>


class OculusCameraManipulator: public osgGA::CameraManipulator
{

	class UnknownTransformType {};

    public:
		OculusCameraManipulator(osg::MatrixTransform *node): CameraManipulator::CameraManipulator(), _mat(node)
	 	{
//			_offset.makeIdentity();
//			_offset.makeRotate(osg::DegreesToRadians(-90.0), osg::Vec3(0,0,1));
//			_offset.preMultRotate(osg::Quat(osg::DegreesToRadians(90.0), osg::Vec3(0,1,0)));
			ocm_sub_ = nh_.subscribe<geometry_msgs::Quaternion>("oculus/orientation", 1, &OculusCameraManipulator::ocmCallback, this);
		};
        virtual bool handle(const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&);
        virtual void setByMatrix(const osg::Matrixd& m);
        virtual osg::Matrixd getMatrix() const;
        virtual void setByInverseMatrix(const osg::Matrixd &m);
        virtual osg::Matrixd getInverseMatrix() const;

    private:         
        osg::MatrixTransform *_mat;
        osg::Matrixd _offset;				//quaternion de oculus
		ros::NodeHandle nh_;
		ros::Subscriber ocm_sub_;
		void ocmCallback(const geometry_msgs::Quaternion::ConstPtr& hmdimu);
};

