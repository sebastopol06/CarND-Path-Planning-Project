#ifndef PREDICTIONS_H
#define PREDICTIONS_H

#include <math.h>
#include <iostream>
#include <vector>
#include <map>

#include "utility.h"

std::map<int, t_traj> generate_predictions(t_traj &sensor_fusion, double car_s, double car_d, int horizon);

#endif // PREDICTIONS_H
