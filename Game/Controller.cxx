#include "Controller.h"
#include "Window.h"

#include <chrono>
#include <thread>
#include <OGRE/OgreImage.h>
#include <OGRE/OgreEntity.h>
#include "PolyVoxCore/CubicSurfaceExtractorWithNormals.h"
#include "PolyVoxCore/MarchingCubesSurfaceExtractor.h"
#include "PolyVoxCore/SurfaceMesh.h"
#include "PolyVoxCore/SimpleVolume.h"

Controller::Controller() :
    ogreRoot(new Ogre::Root()),
    running(true),
    collisionHeightMap(0)
{
    ogreRoot->loadPlugin("RenderSystem_GL");
    ogreRoot->loadPlugin("Plugin_ParticleFX");
    ogreRoot->loadPlugin("Plugin_OctreeSceneManager");
    ogreRoot->loadPlugin("Plugin_CgProgramManager");
}

Controller::~Controller() {
    for(int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
            delete body->getMotionState();
        }
        dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }

    for (int i = 0; i < collisionShapes.size(); ++i) {
        delete collisionShapes[i];
    }

    delete collisionHeightMap;
}

void Controller::init() {
    if(!ogreRoot->restoreConfig())
        ogreRoot->showConfigDialog();

    ogreRoot->addFrameListener(this);

    ogreRoot->addResourceLocation("data/material", "FileSystem", "material");
    ogreRoot->addResourceLocation("data/mesh", "FileSystem", "mesh");
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
    camera->setPosition(Ogre::Vector3(0.0f, 0.0f, 300.0f));
    camera->lookAt(Ogre::Vector3(0.0f, 0.0f, 0.0f));

    collisionConfiguration.reset(new btDefaultCollisionConfiguration());
    collisionDispatcher.reset(new btCollisionDispatcher(collisionConfiguration.get()));
    btVector3 worldMin(-1000, -1000, -1000);
    btVector3 worldMax(1000, 1000, 1000);
    overlappingPairCache.reset(new btAxisSweep3(worldMin, worldMax));
    constraintSolver.reset(new btSequentialImpulseConstraintSolver());
    dynamicsWorld.reset(new btDiscreteDynamicsWorld(collisionDispatcher.get(), overlappingPairCache.get(), constraintSolver.get(), collisionConfiguration.get()));
    btVector3 gravity(0.0f, 0.0f, -9.8f);
    dynamicsWorld->setGravity(gravity);

    setupScene();
}

void Controller::run() {
    lastStep = std::chrono::system_clock::now();
    try {
        ogreRoot->startRendering();
    } catch(const std::exception& e) {
        std::cerr << "Error in render loop: " << e.what() << std::endl;
    }
}

bool Controller::frameRenderingQueued(const Ogre::FrameEvent& event) {
    onFrameRender(event);
    camera->moveRelative(moveVector);
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    double dt = static_cast<float>(std::chrono::duration_cast<std::chrono::milliseconds>(now - lastStep).count()) / 1000.0f;
    dynamicsWorld->stepSimulation(dt, 5);
    lastStep = now;
    for(int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
            Ogre::SceneNode *node = static_cast<Ogre::SceneNode*>(body->getUserPointer());
            if(node) {
                btTransform transform;
                body->getMotionState()->getWorldTransform(transform);
                btVector3 point = transform.getOrigin();
                node->setPosition(Ogre::Vector3((float)point[0], (float)point[1], (float)point[2]));
                //std::cout << point[0] << ", " << point[1] << ", " << point[2] << std::endl;
            }
        }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1L));
    return running && window;
}

void Controller::setupScene() {
    std::cout << "setupScene start" << std::endl;

    Ogre::Image heightMap;
    heightMap.load("test3.png", "map");

    std::cout << "filling voxels" << std::endl;
    PolyVox::SimpleVolume<uint8_t> volData(PolyVox::Region(PolyVox::Vector3DInt32(0,0,0), PolyVox::Vector3DInt32(heightMap.getWidth() - 1, heightMap.getHeight() - 1, 255)));
    collisionHeightMap = new uint8_t[volData.getWidth() * volData.getHeight()];
    std::cout << "dimensions - x: " << volData.getWidth() << ", y: " << volData.getHeight() << ", z: " << volData.getDepth() << std::endl;
    for(int x = 0; x < volData.getWidth(); ++x) {
        for(int y = 0; y < volData.getHeight(); ++y) {
            Ogre::ColourValue color = heightMap.getColourAt(x, y, 0);
            int value = (color.r + color.g + color.b) / 3.0f * 255;
            collisionHeightMap[x * volData.getWidth() + y] = value;
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
    float minZ = mesh.getVertices()[0].position.getZ();
    float maxZ = mesh.getVertices()[0].position.getZ();
    for(const auto& vertex : mesh.getVertices()) {
        if(vertex.position.getX() > maxX) maxX = vertex.position.getX();
        if(vertex.position.getX() < minX) minX = vertex.position.getX();
        if(vertex.position.getY() > maxY) maxY = vertex.position.getY();
        if(vertex.position.getY() < minY) minY = vertex.position.getY();
        if(vertex.position.getZ() > maxZ) maxZ = vertex.position.getZ();
        if(vertex.position.getZ() < minZ) minZ = vertex.position.getZ();
    }
    float rangeX = maxX - minX;
    float rangeY = maxY - minY;
    float rangeZ = maxZ - minZ;

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

    std::cout << "attaching mesh" << std::endl;

    worldNode = sceneManager->getRootSceneNode()->createChildSceneNode("world");
    worldNode->attachObject(manual);

    std::cout << "adding sun" << std::endl;

    Ogre::Light* directionalLight = sceneManager->createLight("sun");
    directionalLight->setType(Ogre::Light::LT_DIRECTIONAL);
    directionalLight->setDiffuseColour(Ogre::ColourValue::White);
    directionalLight->setSpecularColour(Ogre::ColourValue(0.4f, 0.4f, 0.4f));
    directionalLight->setDirection(Ogre::Vector3(-0.4f, -0.4f, -0.4f));

    std::cout << "creating terrain collision shape" << std::endl;

    btHeightfieldTerrainShape* heightfieldShape = new btHeightfieldTerrainShape(volData.getWidth(), volData.getHeight(),
                                                                                collisionHeightMap,
                                                                                255, 2, false, true);
    heightfieldShape->setUseDiamondSubdivision(true);
    heightfieldShape->setLocalScaling(btVector3(1, 1, 1));
    collisionShapes.push_back(heightfieldShape);
    newRigidBody(heightfieldShape, 0.0f, btVector3(minX + rangeX / 2.0f, minY + rangeY / 2.0f, 255), worldNode);

    camera->setPosition(Ogre::Vector3(minX + rangeX / 2, minY + rangeY / 2, 500.0f));
    camera->lookAt(Ogre::Vector3(minX + rangeX / 2, minY + rangeY / 2, 0));

    std::cout << "adding cube" << std::endl;

    Ogre::MeshPtr cubeMesh = Ogre::MeshManager::getSingleton().load("cube.mesh", "mesh");
    Ogre::Entity* cubeEntity = sceneManager->createEntity("cube.entity", cubeMesh);
    Ogre::SceneNode* cubeNode = worldNode->createChildSceneNode("cube.node");
    cubeNode->attachObject(cubeEntity);
    cubeNode->setScale(0.1f, 0.1f, 0.1f);

    btBoxShape* cubeShape = new btBoxShape(btVector3(5.0f, 5.0f, 5.0f));
    collisionShapes.push_back(cubeShape);
    btRigidBody* cubeBody = newRigidBody(cubeShape, 5.0f, btVector3(minX + rangeX / 2 + 10, minY + rangeY / 2 + 10, 200), cubeNode);

    cubeEntity->setUserAny(Ogre::Any(cubeBody));

    std::cout << "setupScene end" << std::endl;
}

btRigidBody* Controller::newRigidBody(btCollisionShape* shape, float mass, btVector3 origin, Ogre::SceneNode* node) {
    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(origin);
    btVector3 localInertia(0, 0, 0);
    if(mass)
        shape->calculateLocalInertia(mass, localInertia);
    btDefaultMotionState* motionState = new btDefaultMotionState(tr);
    btRigidBody::btRigidBodyConstructionInfo info(mass, motionState, shape, localInertia);
    btRigidBody* body = new btRigidBody(info);
    body->setContactProcessingThreshold(BT_LARGE_FLOAT);
    body->setUserPointer(node);
    dynamicsWorld->addRigidBody(body);
    return body;
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
