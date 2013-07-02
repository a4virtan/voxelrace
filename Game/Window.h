#pragma once

#include <memory>
#include <boost/signals2/signal.hpp>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreWindowEventUtilities.h>
#include <OGRE/OgreCamera.h>
#include <OISEvents.h>
#include <OISInputManager.h>
#include <OISKeyboard.h>
#include <OISJoyStick.h>
#include <OISMouse.h>

class Controller;

class Window : public std::enable_shared_from_this<Window>,
               public Ogre::WindowEventListener,
               public OIS::KeyListener,
               public OIS::MouseListener,
               public OIS::JoyStickListener
{
public:
    Window(Ogre::RenderWindow* window, Controller* controller);

    virtual ~Window();

    void init();

    Ogre::RenderWindow* getHandle();

    void addViewport(Ogre::Camera* camera);

    typedef boost::signals2::signal<void (const OIS::MouseEvent&)> MouseMoveSignalType;
    typedef boost::signals2::signal<void (const OIS::MouseEvent&, OIS::MouseButtonID, bool)> MouseButtonSignalType;
    typedef boost::signals2::signal<void (const OIS::KeyEvent&, bool)> KeyboardSignalType;
    typedef boost::signals2::signal<void (const OIS::JoyStickEvent&, int)> JoystickSignalType;
    typedef boost::signals2::signal<void (const OIS::JoyStickEvent&, int, bool)> JoystickButtonSignalType;
    typedef boost::signals2::signal<void (const Window&)> WindowCloseSignalType;

    MouseMoveSignalType onMouseMove;
    MouseButtonSignalType onMousePress;
    KeyboardSignalType onKeyPress;
    JoystickButtonSignalType onJoystickButtonPress;
    JoystickSignalType onJoystickAxisMove;
    WindowCloseSignalType onClose;

private:
    Window();

    // OIS::KeyListener
    virtual bool keyPressed(const OIS::KeyEvent& arg);
    virtual bool keyReleased(const OIS::KeyEvent& arg);

    // OIS::MouseListener
    virtual bool mouseMoved(const OIS::MouseEvent& arg);
    virtual bool mousePressed(const OIS::MouseEvent& arg, OIS::MouseButtonID id);
    virtual bool mouseReleased(const OIS::MouseEvent& arg, OIS::MouseButtonID id);

    // OIS::JoyStickListener
    virtual bool buttonPressed(const OIS::JoyStickEvent& arg, int button);
    virtual bool buttonReleased(const OIS::JoyStickEvent& arg, int button);
    virtual bool axisMoved(const OIS::JoyStickEvent& arg, int axis);

    // Ogre::WindowEventListener
    virtual void windowResized(Ogre::RenderWindow* window);
    virtual void windowClosed(Ogre::RenderWindow* window);

    void step(const Ogre::FrameEvent& event);

    Ogre::RenderWindow* window;
    Controller* controller;
    OIS::InputManager* inputManager;
    OIS::Keyboard* keyboard;
    OIS::JoyStick* joystick;
    OIS::Mouse* mouse;
};
