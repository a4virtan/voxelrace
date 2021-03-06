#include "Controller.h"

#include <memory>

int main(int argc, char **argv)
{
    std::shared_ptr<Controller> controller = std::make_shared<Controller>();
    controller->init();
    controller->run();
}
