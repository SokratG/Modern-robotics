#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <exception>
#include <time.h>
#include <stdlib.h>
#include <sstream>
#include <iomanip>
#include <cmath>


#define RAND_SEED() srand(time(NULL))
#define RAND(min, max) (min + rand() % (max - min + 1))

#ifdef _USE_MATH
    #define M_PI        3.14159265358979323846
    #define M_PI_2      1.57079632679489661923
#endif

using std::cout;
using std::endl;
using std::string;
using std::ofstream;
using std::exception;
using std::stringstream;

double round_up(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::ceil(value * multiplier) / multiplier;
}

double round_down(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::floor(value * multiplier) / multiplier;
}

double near(double value, int decimal_places) {
    const double multiplier = std::pow(10.0, decimal_places);
    return std::round(value * multiplier) / multiplier;
}