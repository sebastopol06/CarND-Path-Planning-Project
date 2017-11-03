#ifndef COST_H
#define COST_H

#include <math.h>
#include <iostream>
#include <vector>
#include <iterator>
#include <map>
#include <cmath>

#include "params.h"
#include "utility.h"

// TARGET: lane, velocity, time for maneuver
double cost_function(t_traj &trajectory, int target_lane, double target_vel, std::map<int, t_traj> &predictions, t_traj &sensor_fusion, int car_lane);

#endif // COST_H
