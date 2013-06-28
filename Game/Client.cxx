#include "Client.h"

#include <boost/thread.hpp>
#include "PolyVoxCore/CubicSurfaceExtractorWithNormals.h"
#include "PolyVoxCore/MarchingCubesSurfaceExtractor.h"
#include "PolyVoxCore/SurfaceMesh.h"
#include "PolyVoxCore/SimpleVolume.h"

Client::Client() :
    ogreRoot(new Ogre::Root())
{
    ogreRoot->loadPlugin("RenderSystem_GL");
    ogreRoot->loadPlugin("Plugin_ParticleFX");
    ogreRoot->loadPlugin("Plugin_OctreeSceneManager");
    ogreRoot->loadPlugin("Plugin_CgProgramManager");

    ogreRoot->addFrameListener(this);
}

Client::~Client() {
}

void Client::run() {
    if(!ogreRoot->restoreConfig())
        ogreRoot->showConfigDialog();
    Ogre::RenderWindow* window = ogreRoot->initialise(true, "Game");
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

    setupScene();

    try {
        ogreRoot->startRendering();
    } catch(const std::exception& e) {
        std::cerr << "Error in render loop: " << e.what() << std::endl;
    }
}

bool Client::frameRenderingQueued(const Ogre::FrameEvent& event) {
    boost::this_thread::sleep_for(boost::chrono::nanoseconds(10000000L));
    return true;
}

void Client::setupScene() {
    std::cout << "setupScene start" << std::endl;
    PolyVox::SimpleVolume<uint8_t> volData(PolyVox::Region(PolyVox::Vector3DInt32(0,0,0), PolyVox::Vector3DInt32(63, 63, 63)));
    for (int y = 0; y < volData.getHeight(); y++) {
        volData.setVoxelAt(0, y, 0, 255);
    }
    PolyVox::SurfaceMesh<PolyVox::PositionMaterialNormal> mesh;
    PolyVox::CubicSurfaceExtractorWithNormals<PolyVox::SimpleVolume<uint8_t> > surfaceExtractor(&volData, volData.getEnclosingRegion(), &mesh);
    surfaceExtractor.execute();
    std::cout << "setupScene end" << std::endl;
}
