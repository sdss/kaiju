#include <iostream>
#include "fiducial.h"


Fiducial::Fiducial(int id, vec3 xyzWok, double collisionBuffer)
    : id(id), xyzWok(xyzWok), collisionBuffer(collisionBuffer) {
    x = xyzWok[0];
    y = xyzWok[1];
}

