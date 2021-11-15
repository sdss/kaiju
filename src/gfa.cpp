#include <iostream>
#include "gfa.h"


GFA::GFA(int id, std::array<vec3, 2> collisionSegWokXYZ, double collisionBuffer)
    : id(id), collisionSegWokXYZ(collisionSegWokXYZ), collisionBuffer(collisionBuffer) {
}

