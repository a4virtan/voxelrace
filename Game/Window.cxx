#include "Window.h"
#include "Controller.h"
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreViewport.h>

//@todo add better support to connect to multiple devices of same type
Window::Window(Ogre::RenderWindow* window, Controller* controller) :
    window(window),
    controller(controller),
    keyboard(0),
    joystick(0),
    mouse(0)
{
    size_t windowHnd = 0;
    window->getCustomAttribute("WINDOW", &windowHnd);
    inputManager = OIS::InputManager::createInputSystem(windowHnd);

    if(inputManager->getNumberOfDevices(OIS::OISKeyboard))
    {
        keyboard = dynamic_cast<OIS::Keyboard*>(inputManager->createInputObject(OIS::OISKeyboard, true));
        keyboard->setEventCallback(this);
    }
    if(inputManager->getNumberOfDevices(OIS::OISMouse))
    {
        mouse = dynamic_cast<OIS::Mouse*>(inputManager->createInputObject(OIS::OISMouse, true));
        mouse->setEventCallback(this);
    }
    if(inputManager->getNumberOfDevices(OIS::OISJoyStick))
    {
        joystick = dynamic_cast<OIS::JoyStick*>(inputManager->createInputObject(OIS::OISJoyStick, true));
        joystick->setEventCallback(this);
    }

    //Set initial mouse clipping size
    windowResized(window);

    //Register as a Window listener
    Ogre::WindowEventUtilities::addWindowEventListener(window, this);
}

Window::~Window()
{
    if(mouse)
        inputManager->destroyInputObject(mouse);
    if(keyboard)
        inputManager->destroyInputObject(keyboard);
    if(joystick)
        inputManager->destroyInputObject(joystick);
    OIS::InputManager::destroyInputSystem(inputManager);
    Ogre::Root::getSingleton().destroyRenderTarget(window);
}

void Window::init()
{
    controller->onFrameRender.connect(Controller::FrameRenderSignalType::slot_type(&Window::step, this, _1).track_foreign(shared_from_this()));
}

Ogre::RenderWindow* Window::getHandle()
{
    return window;
}

void Window::step(const Ogre::FrameEvent& event)
{
    if(keyboard)
        keyboard->capture();
    if(mouse)
        mouse->capture();
    if(joystick)
        joystick->capture();
}

bool Window::keyReleased(const OIS::KeyEvent& arg)
{
    onKeyRelease(arg);
    return true;
}

bool Window::keyPressed(const OIS::KeyEvent& arg)
{
    onKeyPress(arg);
    return true;
}

bool Window::mouseMoved(const OIS::MouseEvent& arg)
{
    onMouseMove(arg);
    return true;
}

bool Window::mousePressed(const OIS::MouseEvent& arg, OIS::MouseButtonID id)
{
    onMousePress(arg, id);
    return true;
}

bool Window::mouseReleased(const OIS::MouseEvent &arg, OIS::MouseButtonID id)
{
    onMouseRelease(arg, id);
    return true;
}

bool Window::buttonPressed(const OIS::JoyStickEvent& arg, int button)
{
    onJoystickButtonPress(arg, button);
    return true;
}

bool Window::buttonReleased(const OIS::JoyStickEvent& arg, int button)
{
    onJoystickButtonRelease(arg, button);
    return true;
}

bool Window::axisMoved(const OIS::JoyStickEvent& arg, int axis)
{
    onJoystickAxisMove(arg, axis);
    return true;
}

void Window::windowResized(Ogre::RenderWindow* window)
{
    unsigned int width, height, depth;
    int left, top;
    window->getMetrics(width, height, depth, left, top);

    if(mouse)
    {
        const OIS::MouseState& ms = mouse->getMouseState();
        ms.width = width;
        ms.height = height;
    }
}

void Window::windowClosed(Ogre::RenderWindow* window)
{
    onClose(*this);
}

void Window::addViewport(Ogre::Camera* camera)
{
    Ogre::Viewport* viewport = window->addViewport(camera);
    viewport->setBackgroundColour(Ogre::ColourValue(0, 0, 0));
    camera->setAspectRatio(Ogre::Real(viewport->getActualWidth()) / Ogre::Real(viewport->getActualHeight()));
}
