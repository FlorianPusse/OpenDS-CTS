#include <iostream>
#include "param.h"
#include <utility>
#include <cmath>
#include <cassert>
#include "coord.h"

using namespace std;

/**
 *    A-----------N-----------B
 *    |           ^           |
 *    |           |           |
 *    |     |-L<--H-----|     |
 *    |     |           |     |
 *    |     |           |     |
 *    |     |           |     |
 *    |     |           |     |
 *    |     |           |     |
 *    |     |           |     |
 *    |     |           |     |
 *    |     |M          |     |
 *    |     |           |     |
 *    |     |           |     |
 *    |     |-----------|     |
 *    |                       |
 *    |                       |
 *    D-----------------------C
 *
 * H: center of the head of the car
 * N: a point right in front of the car
 * L: a point to the left/right of H
 *
 * A point M is inside the safety zone ABCD iff
 *   ((0 <= HM . HN & (HM . HN)^2 <= (HN . HN) * front_margin^2) || (0 => HM . HN & (HM . HN)^2 < (HN . HN) * back_margin^2))
 *   && (HM . HL)^2 <= (HL . HL) * side_margin^2
 */


bool InRectangle(double HNx, double HNy, double HMx, double HMy, double front_margin, double back_margin, double side_margin) {
	double HLx = - HNy, // direction after 90 degree anticlockwise rotation
				 HLy = HNx;

	double HM_HN = HMx * HNx + HMy * HNy, // HM . HN
				 HN_HN = HNx * HNx + HNy * HNy; // HN . HN
	if (HM_HN >= 0 && HM_HN * HM_HN > HN_HN * front_margin * front_margin)
		return false;
	if (HM_HN <= 0 && HM_HN * HM_HN > HN_HN * back_margin * back_margin)
		return false;

	double HM_HL = HMx * HLx + HMy * HLy, // HM . HL
				 HL_HL = HLx * HLx + HLy * HLy; // HL . HL
	return HM_HL * HM_HL <= HL_HL * side_margin * side_margin;
}

void rotatePosition(const double centerX, const double centerZ, const double theta, const double x, const double z, double& resX, double& resZ){
    double tempX = x - centerX;
    double tempZ = z - centerZ;

    double sin_theta = ModelParams::lookupTable->sin(theta);
    double cos_theta = ModelParams::lookupTable->cos(theta);

    double rotatedX = tempX * cos_theta - tempZ * sin_theta;
    double rotatedY = tempX * sin_theta + tempZ * cos_theta;

    resX = rotatedX + centerX;
    resZ = rotatedY + centerZ;
}

bool inCollision(const double Mx, const double My, const COORD& car_pos) {
    const double &carX = car_pos.x;
    const double &carY = car_pos.y;
    const double &carTheta = car_pos.theta;

    double Hx = -1;
    double Hy = -1;

    rotatePosition(carX,carY,carTheta,carX + ModelParams::CAR_LENGTH / 2.0, carY, Hx, Hy);

    double Nx = Hx + (Hx - carX); // Move point further in direction of head of the car
    double Ny = Hy + (Hy - carY); // Move point further in direction of head of the car

	double HNx = Nx - Hx;
	double HNy = Ny - Hy;
	double HMx = Mx - Hx;
	double HMy = My - Hy;

	double side_margin = ModelParams::CAR_WIDTH / 2.0 + ModelParams::COLLISION_SIDE_DISTANCE;
	double front_margin = ModelParams::COLLISION_DISTANCE;
	double back_margin = ModelParams::CAR_LENGTH + 0.1;

	return InRectangle(HNx, HNy, HMx, HMy, front_margin, back_margin, side_margin);
}