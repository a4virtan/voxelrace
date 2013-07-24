#pragma once

#include "Window.h"
#include <memory>
#include <chrono>
#include <boost/signals2/signal.hpp>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreMeshManager.h>
#include <OGRE/OgreManualObject.h>
#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>

class Controller : public Ogre::FrameListener,
                   public std::enable_shared_from_this<Controller> {
public:
    Controller();
    ~Controller();

    void init();

    void run();

    typedef boost::signals2::signal<void (const Ogre::FrameEvent&)> FrameRenderSignalType;

    FrameRenderSignalType onFrameRender;

private:
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& event);

    void windowClosed(const Window& window);

    void keyPressed(const OIS::KeyEvent& event, bool state);

    void mouseMoved(const OIS::MouseEvent& event);

    void setupScene();

    std::unique_ptr<Ogre::Root> ogreRoot;
    std::unique_ptr<Ogre::SceneManager> sceneManager;
    Ogre::SceneNode* worldNode;
    Ogre::Camera* camera;
    std::shared_ptr<Window> window;
    bool running;
    Ogre::Vector3 moveVector;
    std::chrono::time_point<std::chrono::system_clock> lastStep;

    std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration;
    std::unique_ptr<btCollisionDispatcher> collisionDispatcher;
    std::unique_ptr<btAxisSweep3> overlappingPairCache;
    std::unique_ptr<btSequentialImpulseConstraintSolver> constraintSolver;
    std::unique_ptr<btDynamicsWorld> dynamicsWorld;
    btAlignedObjectArray<btCollisionShape*> collisionShapes;
    uint8_t* collisionHeightMap;
};
