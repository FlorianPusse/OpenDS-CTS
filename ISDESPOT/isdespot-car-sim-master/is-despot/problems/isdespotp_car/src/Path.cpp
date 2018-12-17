#include "Path.h"
#include<iostream>
#include "math_utils.h"
using namespace std;

int Path::nearest(COORD pos) {
    auto& path = *this;
    double dmin = COORD::EuclideanDistanceSimple(pos, path[0]);
    int imin = 0;
    for(int i=0; i<path.size(); i++) {
        double d = COORD::EuclideanDistanceSimple(pos, path[i]);
        if(dmin > d) {
            dmin = d;
            imin = i;
        }
    }
    return imin;
}

int Path::forward(int i, double len) const {
    auto& path = *this;

    i += int(len / ModelParams::PATH_STEP);
    if(i > path.size()-1) {
        i = (int) path.size()-1;
    }
    return i;
}