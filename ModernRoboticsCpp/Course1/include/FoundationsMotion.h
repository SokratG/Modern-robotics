#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <exception>
#include <time.h>
#include <stdlib.h>
#include <sstream>
#include <iomanip>


#define RAND_SEED() srand(time(NULL))
#define RAND(min, max) (min + rand() % (max - min + 1))

using std::cout;
using std::endl;
using std::string;
using std::ofstream;
using std::exception;
using std::stringstream;

