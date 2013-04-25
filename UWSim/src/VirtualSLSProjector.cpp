/*
 * VirtualStructuredLightProjector.cpp
 *
 *  Created on: 06/02/2013
 *      Author: Miquel Massot
 *
 */

#include "VirtualSLSProjector.h"
#include "UWSimUtils.h"
#include "osg/BlendFunc"
#include <osgDB/FileNameUtils>
#include <osgOcean/ShaderManager>
#include <osgDB/FileUtils>
#include <osgDB/fstream>
#include <osgDB/Registry>
#include <iostream>
#include <assert.h>

bool VirtualSLSProjector::loadShaderSource(osg::Shader* obj, const std::string& fileName )
{
  std::string fqFileName = osgDB::findDataFile(fileName);
  if( fqFileName.length() == 0 )
  {
    std::cout << "File \"" << fileName << "\" not found." << std::endl;
    return false;
  }
  bool success = obj->loadShaderSourceFromFile( fqFileName.c_str());
  if ( !success  )
  {
    std::cout << "Couldn't load file: " << fileName << std::endl;
    return false;
  }
  else
  {
    std::cout << "File " << fileName << " has been loaded correctly!" << std::endl;
    return true;
  }
}

VirtualSLSProjector::VirtualSLSProjector(){
    osg::ref_ptr<osg::Node> node = new osg::Node;
    osg::ref_ptr<osg::Node> root = new osg::Node;
    std::string name = "SLSprojector";
    std::string image_name = "laser_texture.png";
    double range = 0; //TODO: Not implemented 
    double fov = 60.0;
    bool visible = 1; //TODO: Not implemented
    init(name, root, node, image_name, range, fov, visible);
}

VirtualSLSProjector::VirtualSLSProjector(std::string name, osg::Node *root, osg::Node *node, std::string image_name, double fov, bool visible) {
	double range = 0; 
    init(name, root, node, image_name, range, fov, visible);
}

void VirtualSLSProjector::init(std::string name, osg::Node *root, osg::Node *node, std::string image_name, double range, double fov, bool visible) {
	this->name = name;
    this->fov = fov;
    this->range = range;
    this->node = node;
    this->image_name = image_name;
    this->visible = visible;
    this->lightNum = 0;
    this->textureUnit = 1;
    osg::Vec3 position(0.0f,0.0f,0.0f);
    //osg::Vec3 direction(1.0f, 0.0f, 0.0f);
    osg::Vec3 direction(0.0f, 0.0f, 1.0f);//NEW CONVENTION

    // Add SLS projector!
    this->node->asGroup()->addChild(createSLNode(position, direction, fov, lightNum, textureUnit));

    //Add a switchable frame geometry on the sensor frame
    //osg::ref_ptr<osg::Node> axis=UWSimGeometry::createSwitchableFrame();
    //this->node->asGroup()->addChild(axis);

    project_on(root);	
}

osg::Node* VirtualSLSProjector::createSLNode(const osg::Vec3& position, const osg::Vec3& direction, float angle, unsigned int lightNum, unsigned int textureUnit)
{
    osg::Group* group = new osg::Group;
    
    // create light source.
    osg::LightSource* lightsource = new osg::LightSource;
    osg::Light* light = lightsource->getLight();
    light->setLightNum(lightNum);
    light->setPosition(osg::Vec4(position,1.0f));
    light->setAmbient(osg::Vec4(0.00f,0.00f,0.05f,1.0f));
    light->setDiffuse(osg::Vec4(1.0f,1.0f,1.0f,1.0f));
    light->setConstantAttenuation(0.0f);
    light->setLinearAttenuation(0.0f);
    light->setQuadraticAttenuation(0.0f);
    group->addChild(lightsource);
    
    // create tex gen. 
    //osg::Vec3 up(0.0f,0.0f,1.0f);
    osg::Vec3 up(0.0f,-1.0f,0.0f); //NEW CONVENTION
    up = (direction ^ up) ^ direction;
    up.normalize();
    
    osg::TexGenNode* texgenNode = new osg::TexGenNode;
    texgenNode->setTextureUnit(textureUnit);
    osg::TexGen* texgen = texgenNode->getTexGen();
    texgen->setMode(osg::TexGen::EYE_LINEAR);       
    //                                                 EYE          CENTER         UP
    texgen->setPlanesFromMatrix(osg::Matrixd::lookAt(position, position+direction, up)*
                                osg::Matrixd::perspective(angle,1.0,0.1,100)*
                                osg::Matrixd::translate(1.0,1.0,1.0)*
                                //osg::Matrixd::translate(0.0,0.0,0.0)*
                                osg::Matrixd::scale(0.5,0.5,0.5));
                                //osg::Matrixd::scale(1,1,1));

    group->addChild(texgenNode);
    return group;
}

osg::StateSet* VirtualSLSProjector::createSLDecoratorState(osg::StateSet* stateset, unsigned int lightNum, unsigned int textureUnit)
{
    stateset->setMode(GL_LIGHT0+lightNum, osg::StateAttribute::ON);

    osg::Vec4 ambientColour(0.1f,0.1f,0.1f,0.1f);
    //osg::Vec4 ambientColour(0.05f,0.05f,0.05f,1.0f); 

    // set up spot light texture
    osg::Texture2D* texture = new osg::Texture2D();
    osg::Image* texture_to_project = osgDB::readImageFile(this->image_name);
    assert(texture_to_project);
    texture->setImage(texture_to_project);
    texture->setBorderColor(osg::Vec4(ambientColour));
    texture->setWrap(osg::Texture::WRAP_S,osg::Texture::CLAMP_TO_BORDER); // fa que la textura no es repeteixi continuament
    texture->setWrap(osg::Texture::WRAP_T,osg::Texture::CLAMP_TO_BORDER); // veure: http://lucera-project.blogspot.com.es/2010/06/opengl-wrap.html
    texture->setWrap(osg::Texture::WRAP_R,osg::Texture::CLAMP_TO_BORDER);
    
    stateset->setTextureAttributeAndModes(textureUnit, texture, osg::StateAttribute::ON);
    
    // set up tex gens
    stateset->setTextureMode(textureUnit, GL_TEXTURE_GEN_S, osg::StateAttribute::ON); //Generació automàtica de 
    stateset->setTextureMode(textureUnit, GL_TEXTURE_GEN_T, osg::StateAttribute::ON); //les coordenades de les textures
    stateset->setTextureMode(textureUnit, GL_TEXTURE_GEN_R, osg::StateAttribute::ON); //per mapejar correctament aquestes
    stateset->setTextureMode(textureUnit, GL_TEXTURE_GEN_Q, osg::StateAttribute::ON); //als models en 3D
    
    //TODO: transparency
    //TODO: light
    //TODO: project only on the scene, not in vehicle
    //Notes: a sobre d'un color hi ha textura, a sobre de negre, no.

    osg::BlendFunc *blendFunc = new osg::BlendFunc;
    //blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE); //oceà blanc, linies a la part blanca de la caixa
    blendFunc->setFunction(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA); // oceà blau, igual que abans
    //blendFunc->setFunction(GL_SRC_COLOR,GL_ONE_MINUS_CONSTANT_COLOR); // oceà blau clar, igual
    //blendFunc->setFunction(GL_SRC_COLOR,GL_CONSTANT_COLOR); // ocea negre/blau, igual.
    //stateset->setMode( GL_LIGHTING, osg::StateAttribute::OFF );

    //stateset->setMode( GL_DEPTH_TEST, osg::StateAttribute::OFF ); // actiu per defecte, si el descativo no es dibuxa res a la caixa
    //stateset->setMode( GL_CULL, osg::StateAttribute::OFF );

    //stateset->setMode( GL_BLEND, osg::StateAttribute::ON );
    stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    //stateset->setAttributeAndModes( blendFunc, osg::StateAttribute::ON );
 

    std::cout << "Going to read shaders maybe?" << std::endl;

    // Load shaders!
    /*osg::Program* projProg(new osg::Program);
    osg::Shader* projVertexShader( new osg::Shader(osg::Shader::VERTEX));
    osg::Shader* projFragShader( new osg::Shader(osg::Shader::FRAGMENT));
    projProg->addShader(projVertexShader);
    projProg->addShader(projFragShader);
    loadShaderSource(projVertexShader, "vertex.vert");
    loadShaderSource(projFragShader, "fragment.frag");
    stateset->setAttributeAndModes(projProg, osg::StateAttribute::ON);*/

	static const char model_vertex[]   = "vertex.vert";
	static const char model_fragment[] = "fragment.frag";
	osg::ref_ptr<osg::Program> projProg = osgOcean::ShaderManager::instance().createProgram("robot_shader", model_vertex, model_fragment, model_vertex, model_fragment);
    
    std::cout << projProg->getNumShaders() << std::endl;

    stateset->setAttributeAndModes(projProg, osg::StateAttribute::ON);

    std::cout << "Are shaders read?" << std::endl;

    return stateset;
}

void VirtualSLSProjector::project_on(osg::Node* canvas)
{
    osg::StateSet* stateset = canvas->getOrCreateStateSet();
    canvas->setStateSet(createSLDecoratorState(stateset,lightNum,textureUnit));
    //std::cout << stateset->getActiveTextureUnit() << std::endl;
}
