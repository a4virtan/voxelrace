#include "Controller.h"
#include "Window.h"

#include <chrono>
#include <thread>
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
    Ogre::RenderWindow* ogreWindow = ogreRoot->initialise(true, "Game");
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    window = std::make_shared<Window>(ogreWindow, this);
    window->init();
    window->onClose.connect(Window::WindowCloseSignalType::slot_type(&Controller::windowClosed, shared_from_this(), _1).track_foreign(shared_from_this()));
    window->onKeyPress.connect(Window::KeyboardSignalType::slot_type(&Controller::keyPressed, shared_from_this(), _1).track_foreign(shared_from_this()));
    sceneManager.reset(Ogre::Root::getSingleton().createSceneManager("OctreeSceneManager"));
    sceneManager->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
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
    std::this_thread::sleep_for(std::chrono::milliseconds(1L));
    return running && window;
}

void Controller::setupScene() {
    std::cout << "setupScene start" << std::endl;
    PolyVox::SimpleVolume<uint8_t> volData(PolyVox::Region(PolyVox::Vector3DInt32(0,0,0), PolyVox::Vector3DInt32(63, 63, 63)));
    for(int x = 0; x < volData.getWidth(); x += 5) {
        for(int y = 0; y < volData.getHeight(); y++) {
            volData.setVoxelAt(x, y, 0, 255);;
        }
    }
    PolyVox::SurfaceMesh<PolyVox::PositionMaterialNormal> mesh;
    PolyVox::CubicSurfaceExtractorWithNormals<PolyVox::SimpleVolume<uint8_t> > surfaceExtractor(&volData, volData.getEnclosingRegion(), &mesh);
    surfaceExtractor.execute();

    Ogre::ManualObject* manual = sceneManager->createManualObject("manual");
    manual->begin("BaseWhite", Ogre::RenderOperation::OT_LINE_LIST);

    for(const auto& vertex : mesh.getVertices()) {
        manual->position(vertex.position.getX(), vertex.position.getY(), vertex.position.getZ());
    }
    for(const auto& index : mesh.getIndices()) {
        manual->index(index);
    }

    manual->end();
    worldNode = sceneManager->getRootSceneNode()->createChildSceneNode("world");
    worldNode->attachObject(manual);

    std::cout << "setupScene end" << std::endl;
}

void Controller::windowClosed(const Window& window) {
    running = false;
}

void Controller::keyPressed(const OIS::KeyEvent& event) {
    if(event.key == OIS::KeyCode::KC_ESCAPE) {
        running = false;
    }
}
