#pragma once
#include "../include/Map.h"
#include <memory>
class GLDebug {
public:
    GLDebug();
    virtual ~GLDebug();
    static void init(int argc, char** argv);
    static void disp();
    static void idle();
    static void drawMap(std::shared_ptr<Map> m);
};