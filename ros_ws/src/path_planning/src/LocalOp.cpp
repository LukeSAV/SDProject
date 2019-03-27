#include "../include/LocalOp.h"
#include "../include/Map.h"
#include <memory>

std::shared_ptr<Map> LocalOp::m;

LocalOp::LocalOp() {

}
 
LocalOp::~LocalOp() {

}

void LocalOp::addMap(int end_x_index, int end_y_index) {
    LocalOp::m = std::make_shared<Map>(end_x_index, end_y_index);
}
