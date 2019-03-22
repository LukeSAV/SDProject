#pragma once

#include "../include/Map.h"

#include <memory>

class LocalOp {
public:
    LocalOp();
    virtual ~LocalOp();
    static std::shared_ptr<Map> m;
    static void addMap(int end_x_index, int end_y_index);
};