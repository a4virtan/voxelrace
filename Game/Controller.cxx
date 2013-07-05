#include "Controller.h"
#include "Window.h"

#include <chrono>
#include <thread>
#include <OGRE/OgreImage.h>
#include "PolyVoxCore/CubicSurfaceExtractorWithNormals.h"
#include "PolyVoxCore/MarchingCubesSurfaceExtractor.h"
#include "PolyVoxCore/SurfaceMesh.h"
#include "PolyVoxCore/SimpleVolume.h"

Controller::Controller() :
    ogreRoot(new Ogre::Root()),
    running(true)
{
    ogreRoot->loadPlugin("RenderSystem_GL");
    ogreRoot->loadPlugin("Plugin_ParticleFX");
    ogreRoot->loadPlugin("Plugin_OctreeSceneManager");
    ogreRoot->loadPlugin("Plugin_CgProgramManager");

    ogreRoot->addFrameListener(this);
}

Controller::~Controller() {
}

void Controller::run() {
    if(!ogreRoot->restoreConfig())
        ogreRoot->showConfigDialog();

    ogreRoot->addResourceLocation("data/material", "FileSystem", "material");
    ogreRoot->addResourceLocation("data/mesh", "FileSystem", "meshes");
    ogreRoot->addResourceLocation("data/texture", "FileSystem", "texture");
    ogreRoot->addResourceLocation("data/particle", "FileSystem", "particle");
    ogreRoot->addResourceLocation("data/sound", "FileSystem", "sound");
    ogreRoot->addResourceLocation("data/map", "FileSystem", "map");

    Ogre::RenderWindow* ogreWindow = ogreRoot->initialise(true, "Game");
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    window = std::make_shared<Window>(ogreWindow, this);
    window->init();
    window->onClose.connect(Window::WindowCloseSignalType::slot_type(&Controller::windowClosed, shared_from_this(), _1).track_foreign(shared_from_this()));
    window->onKeyPress.connect(Window::KeyboardSignalType::slot_type(&Controller::keyPressed, shared_from_this(), _1, _2).track_foreign(shared_from_this()));
    window->onMouseMove.connect(Window::MouseMoveSignalType::slot_type(&Controller::mouseMoved, shared_from_this(), _1).track_foreign(shared_from_this()));
    sceneManager.reset(Ogre::Root::getSingleton().createSceneManager("OctreeSceneManager"));
    sceneManager->setAmbientLight(Ogre::ColourValue(0.3, 0.3, 0.3));
    Ogre::MeshManager::getSingleton().setBoundsPaddingFactor(0.0f);
    camera = sceneManager->createCamera("camera");
    camera->setAutoAspectRatio(true);
    camera->setUseMinPixelSize(false);
    camera->setNearClipDistance(0.1f);
    camera->setFarClipDistance(1000.0f);
    window->addViewport(camera);
    camera->setPosition(Ogre::Vector3(0.0f, 10.0f, 200.0f));
    camera->lookAt(Ogre::Vector3(0.0f, 0.0f, 0.0f));

    setupScene();

    try {
        ogreRoot->startRendering();
    } catch(const std::exception& e) {
        std::cerr << "Error in render loop: " << e.what() << std::endl;
    }
}

bool Controller::frameRenderingQueued(const Ogre::FrameEvent& event) {
    onFrameRender(event);
    camera->moveRelative(moveVector);
    std::this_thread::sleep_for(std::chrono::milliseconds(1L));
    return running && window;
}

void Controller::setupScene() {
    std::cout << "setupScene start" << std::endl;

    Ogre::Image heightMap;
    heightMap.load("test3.png", "map");

    std::cout << "filling voxels" << std::endl;
    PolyVox::SimpleVolume<uint8_t> volData(PolyVox::Region(PolyVox::Vector3DInt32(0,0,0), PolyVox::Vector3DInt32(heightMap.getWidth() - 1, heightMap.getHeight() - 1, 255)));
    std::cout << "dimensions - x: " << volData.getWidth() << ", y: " << volData.getHeight() << ", z: " << volData.getDepth() << std::endl;
    for(int x = 0; x < volData.getWidth(); ++x) {
        for(int y = 0; y < volData.getHeight(); ++y) {
            Ogre::ColourValue color = heightMap.getColourAt(x, y, 0);
            int value = (color.r + color.g + color.b) / 3.0f * 255;
            for(int z = 0; z < volData.getDepth(); ++z) {
                if(value > z || z == 0) {
                    volData.setVoxelAt(x, y, z, 255);
                }
            }
        }
    }
    std::cout << "extracting mesh" << std::endl;
    PolyVox::SurfaceMesh<PolyVox::PositionMaterialNormal> mesh;
    //PolyVox::CubicSurfaceExtractorWithNormals<PolyVox::SimpleVolume<uint8_t> > surfaceExtractor(&volData, volData.getEnclosingRegion(), &mesh);
    PolyVox::MarchingCubesSurfaceExtractor<PolyVox::SimpleVolume<uint8_t> > surfaceExtractor(&volData, volData.getEnclosingRegion(), &mesh);
    surfaceExtractor.execute();

    std::cout << "importing mesh" << std::endl;
    Ogre::ManualObject* manual = sceneManager->createManualObject("manual");
    manual->estimateVertexCount(mesh.getVertices().size());
    manual->estimateIndexCount(mesh.getIndices().size());
    manual->begin("grass", Ogre::RenderOperation::OT_TRIANGLE_LIST, "material");

    float minX = mesh.getVertices()[0].position.getX();
    float maxX = mesh.getVertices()[0].position.getX();
    float minY = mesh.getVertices()[0].position.getY();
    float maxY = mesh.getVertices()[0].position.getY();
    for(const auto& vertex : mesh.getVertices()) {
        if(vertex.position.getX() > maxX) maxX = vertex.position.getX();
        if(vertex.position.getX() < minX) minX = vertex.position.getX();
        if(vertex.position.getY() > maxY) maxY = vertex.position.getY();
        if(vertex.position.getY() < minY) minY = vertex.position.getY();
    }
    float rangeX = maxX - minX;
    float rangeY = maxY - minY;

    for(const auto& vertex : mesh.getVertices()) {
        manual->position(vertex.position.getX(), vertex.position.getY(), vertex.position.getZ());
        manual->textureCoord((vertex.position.getX() - minX) / rangeX,
                             (vertex.position.getY() - minY) / rangeY);
        manual->normal(vertex.normal.getX(), vertex.normal.getY(), vertex.normal.getZ());
    }

    for(const auto& index : mesh.getIndices()) {
        manual->index(index);
    }

    manual->end();
    worldNode = sceneManager->getRootSceneNode()->createChildSceneNode("world");
    worldNode->attachObject(manual);

    Ogre::Light* directionalLight = sceneManager->createLight("sun");
    directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
    directionalLight->setDiffuseColour(Ogre::ColourValue::White);
    directionalLight->setSpecularColour(Ogre::ColourValue(0.4f, 0.4f, 0.4f));
    directionalLight->setDirection(Ogre::Vector3(-0.4f, -0.4f, -0.4f));

    std::cout << "setupScene end" << std::endl;
}

void Controller::windowClosed(const Window& window) {
    running = false;
}

void Controller::keyPressed(const OIS::KeyEvent& event, bool state) {
    if(event.key == OIS::KeyCode::KC_ESCAPE) {
        running = false;
    } else if(event.key == OIS::KeyCode::KC_W) {
        moveVector = Ogre::Vector3(0.0f, 0.0f, state ? -0.1f : 0.0f);
    } else if(event.key == OIS::KeyCode::KC_S) {
        moveVector = Ogre::Vector3(0.0f, 0.0f, state ? 0.1f : 0.0f);
    } else if(event.key == OIS::KeyCode::KC_A) {
        moveVector = Ogre::Vector3(state ? -0.1f : 0.0f, 0.0f, 0.0f);
    } else if(event.key == OIS::KeyCode::KC_D) {
        moveVector = Ogre::Vector3(state ? 0.1f : 0.0f, 0.0f, 0.0f);
    }
}

void Controller::mouseMoved(const OIS::MouseEvent& event) {
    camera->yaw(Ogre::Degree(-event.state.X.rel * 0.1f));
    camera->pitch(Ogre::Degree(-event.state.Y.rel * 0.1f));
}
