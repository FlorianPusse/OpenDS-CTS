#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

class SinCosLookupTable {
    const double discretization = 0.005;

    double* cos_table = new double[(int) ceil((2 * M_PI) / discretization)];
    double* sin_table = new double[(int) ceil((2 * M_PI) / discretization)];

public:
    SinCosLookupTable() {
        for (int i = 0; i*discretization < 2 * M_PI; i++) {
            cos_table[i] = std::cos(i*discretization);
            sin_table[i] = std::sin(i*discretization);
        }
    }
    ~SinCosLookupTable(){
        delete[] cos_table;
        delete[] sin_table;
    }

    double sin(double radAngle) {
        if(radAngle < 0){
            radAngle += 2*M_PI;
        }

        return sin_table[(int) (fmod(radAngle, 2* M_PI) / discretization)];
    }

    double cos(double radAngle) {
        if(radAngle < 0){
            radAngle += 2*M_PI;
        }

        return cos_table[(int) (fmod(radAngle, 2* M_PI) / discretization)];
    }
};
