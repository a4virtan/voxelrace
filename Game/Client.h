#pragma once

#include <memory>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreRenderWindow.h>

class Client : public Ogre::FrameListener {
public:
    Client();
    ~Client();

    void run();

private:
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& event);

    void setupScene();

    std::unique_ptr<Ogre::Root> ogreRoot;
};
