#include "cost.h"

using namespace std;

// check max speed, acceleration, jerk
// Returns bool
bool check_max_capabilities(t_traj &traj)
{
  double vx, ax, jx;
  double vy, ay, jy;
  double vel, acc, jerk;
  double max_vel = 0;
  double max_acc = 0;
  double total_jerk = 0;
  double x, x_1, x_2, x_3;
  double y, y_1, y_2, y_3;
  double jerk_per_second;

  assert(traj[0].size() == traj[1].size()); // as much x than y ...

  for (int t = 3; t < traj[0].size(); t++)
  {
    x   = traj[0][t];
    x_1 = traj[0][t-1];
    x_2 = traj[0][t-2];
    x_3 = traj[0][t-3];

    y   = traj[1][t];
    y_1 = traj[1][t-1];
    y_2 = traj[1][t-2];
    y_3 = traj[1][t-3];

    vx = (x - x_1) / PARAM_DT;
    vy = (y - y_1) / PARAM_DT;

    ax = (x - 2*x_1 + x_2) / (PARAM_DT * PARAM_DT);
    ay = (y - 2*y_1 + y_2) / (PARAM_DT * PARAM_DT);

    // rounding to 2 decimals (cm precision) to ensure numerical stability
    jx = (x - 3*x_1 + 3*x_2 - x_3);
    jx = roundf(jx * 100) / 100;
    jx = jx / (PARAM_DT * PARAM_DT * PARAM_DT);
    jy = (y - 3*y_1 + 3*y_2 - y_3);
    jy = roundf(jy * 100) / 100;
    jy = jy / (PARAM_DT * PARAM_DT * PARAM_DT);

    vel = sqrt(vx*vx + vy*vy);
    acc = sqrt(ax*ax + ay*ay);
    jerk = sqrt(jx*jx + jy*jy);

    total_jerk += jerk /** PARAM_DT*/; // Issue: [m.s-2] = jerk [m.s-2] * [s]

    //cout << "jx=" << jx << " jy=" << jy << endl;
    //cout << "vel=" << vel << " acc=" << acc << " jerk=" << jerk << endl;

    if (vel > max_vel)
      max_vel = vel;
    if (acc > max_acc)
      max_acc = acc;
  }

  jerk_per_second = total_jerk / (PARAM_NB_POINTS /** PARAM_DT*/);

  if (roundf(max_vel) > PARAM_MAX_SPEED || roundf(max_acc) > PARAM_MAX_ACCEL || jerk_per_second > PARAM_MAX_JERK)
  {
    cout << "max_vel=" << max_vel << " max_acc=" << max_acc << " jerk_per_second=" << jerk_per_second  << endl;
    //assert(1 == 0);
    return true;
  }
  else
  {
    return false;
  }
}

double get_predicted_dmin(t_traj &trajectory, std::map<int, t_traj> &predictions)
{
  double dmin = INF; // Minimal distance over the whole trajectory

  std::map<int, t_traj>::iterator it = predictions.begin(); // <int, t_traj> = <fusion_index, prediction>
  while(it != predictions.end())
  {
    int fusion_index = it->first;
    //cout << "fusion_index=" << fusion_index << endl;
    t_traj prediction = it->second;

    assert(prediction.size() == trajectory[0].size());
    assert(prediction.size() == trajectory[1].size());

    for (int i = 0; i < prediction.size(); i++) // up to 50 (x,y) coordinates
    {
      dmin = min(dmin, distance(trajectory[0][i]/*ego_x*/, trajectory[1][i]/*ego_y*/, prediction[i][0]/*obj_x*/, prediction[i][1]/*obj_y*/));
      //cout << "dist[" << i << "]=" << dist << endl;
      //if (dist <= PARAM_DIST_COLLISION)
      //{
      //  cout << "=====> DMIN = " << dmin << endl;
      //  cout << "predicted collision in " << i << " steps with fusion_index " << fusion_index << " (dist=" << dist << ")" << endl;
      //  //assert(1 == 0); // TODO temp just for checking purposes
      //}
    }
    it++;
  }

  cout << "=====> dmin = " << dmin << endl;
  assert(dmin >= 0);

  return dmin;
}

double cost_function(t_traj &trajectory, int target_lane, double target_vel, std::map<int, t_traj> &predictions, t_traj &sensor_fusion, int car_lane)
{
  double cost_feasibility = 0.; // vs collisions, vs vehicle capabilities
  double cost_safety      = 0.; // vs buffer distance, vs visibility
  double cost_legality    = 0.; // vs speed limits
  double cost_comfort     = 0.; // vs jerk
  double cost_efficiency  = 0.; // vs desired lane and time to goal

  double weight_feasibility = 100000.; // vs collisions, vs vehicle capabilities
  double weight_safety      =  10000.; // vs buffer distance, vs visibility or curvature
  double weight_legality    =   1000.; // vs speed limits
  double weight_comfort     =    100.; // vs jerk
  double weight_efficiency  =     10.; // vs target lane, target speed and time to goal

  // 1) FEASIBILITY cost
  // TODO: handled via safety so far
  //if (check_collision(trajectory, predictions))
  //{
  //  cost_feasibility += 10;
  //}
  //if (check_max_capabilities(trajectory))
  //{
  //  cost_feasibility += 1;
  //}
  // TBA

  // 2) SAFETY cost
  double dmin = get_predicted_dmin(trajectory, predictions);
  cost_safety = (dmin < PARAM_DIST_SAFETY)? PARAM_DIST_SAFETY - dmin: 0;

  // 3) LEGALITY cost
  // TBA

  // 4) COMFORT cost
  // TBA: sum over traj of squared lateral speed +
  // TBA: sum over traj of squared lateral jerk

  // 5) EFFICIENCY cost
  cost_efficiency = PARAM_MAX_SPEED_MPH - target_vel;
  // TBA: Squared difference of longi final speed and longi init speed
  // TBA: Squared difference of lqt final position and middle of lane

  double cost =
    weight_feasibility * cost_feasibility + // not used yet
    weight_safety      * cost_safety      +
    weight_legality    * cost_legality    + // not used yet
    weight_comfort     * cost_comfort     + // not used yet
    weight_efficiency  * cost_efficiency;

  // cout << "cost safety    =" << weight_safety      * cost_safety      << endl;
  // cout << "cost efficiency=" << weight_efficiency  * cost_efficiency  << endl;

  t_coord ego{trajectory[0][0], trajectory[1][0]};
  // copy(ego.begin(), ego.end(), ostream_iterator<double>(cout, "\n")); // cout whole vector via copy trick

  bool free_lane = true;

  std::map<int, t_traj>::iterator it = predictions.begin();
  // Sensor fusion: vector of doubles [id, x, y, vx, vy, s, d], stored in vector (forall cars)
  vector<double> pSensorFusion = sensor_fusion[0];
  while(it != predictions.end()) // Loop over all the objects
  {
    pSensorFusion = sensor_fusion[it->first/*fusion_index*/];

    if (get_lane(pSensorFusion[D]/*obj_d*/) == target_lane)
    {
      if (target_lane != car_lane)
      {
        cost += distance(ego[0]/*ego_x*/, ego[1]/*ego_y*/, pSensorFusion[1]/*obj_x*/, pSensorFusion[2]/*obj_y*/);
        cost /= PARAM_FOV; // penalize lane changes
      }
      free_lane = false;
    }
    it++;
  }
  if (free_lane)
  {
    cost--; // cost marginally decreased when target lane is not free
  }

  cout << "car_lane=" << car_lane << " target_lane=" << target_lane << " target_vel=" << target_vel << " cost=" << cost << endl;

  return cost;
}
