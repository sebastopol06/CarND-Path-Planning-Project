#ifndef MAP_H
#define MAP_H

#include <math.h>
#include <iostream>
#include <fstream>
#include <vector>

#include "spline.h"

class Map {

public:
  /**
  * Constructor
  */
  Map(std::string map_file);

  /**
  * Destructor
  */
  virtual ~Map();

  t_coord getFrenet(double x, double y, double theta);
  t_coord getXY(double s, double d);
  t_coord getXYspline(double s, double d); // with splines
  double getSpeedToFrenet(double Vxy, double s);

  void plot(void);
  double testError(double x, double y, double yaw); // (x,y) -> (s,d) -> (x,y) conversions and dump accuracy

private:
  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;

  t_coord map_waypoints_x;
  t_coord map_waypoints_y;
  t_coord map_waypoints_s;
  t_coord map_waypoints_dx;
  t_coord map_waypoints_dy;
  t_coord map_waypoints_normx;
  t_coord map_waypoints_normy;

  t_coord map_s; // pre-computed for faster access

  // better granularity: 1 point per meter
  t_coord new_map_waypoints_x;
  t_coord new_map_waypoints_y;
  t_coord new_map_waypoints_dx;
  t_coord new_map_waypoints_dy;

  t_coord new_map_s; // pre-computed for faster access

  double max_error = 0.0;
  double sum_error = 0.0;
  double avg_error = 0.0;
  unsigned int num_error = 0;

  int ClosestWaypoint(double x, double y, const t_coord &maps_x, const t_coord &maps_y);
  int NextWaypoint(double x, double y, double theta, const t_coord &maps_x, const t_coord &maps_y);
};

#endif // MAP_H
