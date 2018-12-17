#ifndef COORD_H_DESPOT
#define COORD_H_DESPOT

#include <stdlib.h>
#include <assert.h>
#include <ostream>
#include <math.h>

struct COORD
{
  double x, y, theta;
  
  COORD() {}

  COORD(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta){}

  COORD(double _x, double _y) : x(_x), y(_y), theta(-1) {}

  bool Valid() const {
    return x >= 0 && y >= 0;
  }

  bool operator==(COORD rhs) const {
    return x == rhs.x && y == rhs.y;
  }

  bool operator<(const COORD& other) const {
    return x < other.x || (x == other.x && y < other.y);
  }
  
  bool operator!=(COORD rhs) const {
    return x != rhs.x || y != rhs.y;
  }

  void operator+=(COORD offset) {
    x += offset.x;
    y += offset.y;
  }

  COORD operator+(COORD rhs) const {
    return COORD(x + rhs.x, y + rhs.y);
  }
  
  COORD operator*(int mul) const
  {
    return COORD(x * mul, y * mul);
  }

  
  static double EuclideanDistance(COORD lhs, COORD rhs);
  static double ManhattanDistance(COORD lhs, COORD rhs);
  static double EuclideanDistanceSimple(COORD lhs, COORD rhs);
};

inline double COORD::EuclideanDistance(COORD lhs, COORD rhs) {
  return sqrt(pow(lhs.x - rhs.x,2) + pow(lhs.y - rhs.y,2));
}

inline double COORD::EuclideanDistanceSimple(COORD lhs, COORD rhs) {
  return (lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y);
}

inline double COORD::ManhattanDistance(COORD lhs, COORD rhs) {
  return fabs(lhs.x - rhs.x) + fabs(lhs.y - rhs.y);
}

inline std::ostream& operator<<(std::ostream& ostr, COORD& COORD) {
  ostr << "(" << COORD.x << ", " << COORD.y << ")";
  return ostr;
}

// COORD_H

#endif
