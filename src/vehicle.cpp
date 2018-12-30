#include "vehicle.h"


static const double dt = 0.02;
static const double max_vel = 45.0/2.23694;
static const int    max_num_waypoints = 50;
static const double kl_buffer_dist = 35.0;
static const double min_buffer_dist = 15.0;
static const double target_kl_dist = 30.0;
static const double target_lc_dist = 50.0;
static const double mph_ms = 2.23694;

Vehicle::Vehicle()
{
  current_state = RDY;
  ref_vel = 1;
  target_vel = max_vel;
  lane = 1;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  lane_change_lanes = {{1}, {0,2}, {1}};

  s_i.push_back(0);
  s_i.push_back(0);
  s_i.push_back(0);
  s_f.push_back(10);
  s_f.push_back(0);
  s_f.push_back(0);

  alpha = VectorXd(6);
  T = 5.0;
  calculateJMT(0, 50, 0, 20, 0, 1, T);

}

Vehicle::~Vehicle()
{
}

void Vehicle::calculateJMT(double s_i, double s_f, double s_i1,
    double s_f1, double s_i2, double s_f2, double T)
{
  //JMT
  MatrixXd A = MatrixXd(3,3);
  MatrixXd B = MatrixXd(3,1);
  A << T*T*T, T*T*T*T, T*T*T*T*T,
    3*T*T, 4*T*T*T,5*T*T*T*T,
    6*T, 12*T*T, 20*T*T*T;
  MatrixXd Ai = A.inverse();

  B << s_f - (s_i + s_i1*T + 0.5*s_i2*T*T),
       s_f1 - (s_i1 + s_i2*T), 
       s_f2 - s_i2;

  MatrixXd C = Ai*B;

  alpha << s_i, s_i1, s_i2/2, C(0), C(1), C(2);

}

void Vehicle::update_data(double car_x, double car_y, double car_s, 
  double car_d, double car_yaw, double car_speed, int prev_size)
{
  this->car_x = car_x;
  this->car_y = car_y;
  this->car_s = car_s;
  this->car_d = car_d;
  this->car_yaw = car_yaw;
  this->car_speed = car_speed;
  this->car_speed_ms = car_speed/mph_ms;
  this->prev_size = prev_size;
}

void Vehicle::process_current_state()
{

  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = deg2rad(car_yaw);

  // ----------------STATE MATCHINE----------------
  if(current_state == RDY)
  {
    if(prev_size < 2)
    {
      int num_pts = (int)(T/dt);
      //use two points that make path tangent to referencee
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);
      ptsx.push_back(prev_car_x);
      ptsy.push_back(prev_car_y);

      vector<double> next_wp0 = getXY(car_s + target_kl_dist, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
      vector<double> next_wp1 = getXY(car_s + 60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
      ptsx.push_back(next_wp0[0]);
      ptsx.push_back(next_wp1[0]);
      ptsy.push_back(next_wp0[1]);
      ptsy.push_back(next_wp1[1]);
      for(int i = 0; i < ptsx.size(); i++)
      {
        //shift car reference angle to 0
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y*sin(-ref_yaw));
        ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y*cos(-ref_yaw));

      }

      tk::spline s;
      s.set_points(ptsx, ptsy);

      double t = 0.0;
      for(int i = 0; i < num_pts; i++)
      {
        double x_point = alpha(1)*t + alpha(2)*t*t + alpha(3)*t*t*t +
                         alpha(4)*t*t*t*t + alpha(5)*t*t*t*t*t;
        double y_point = s(x_point);
        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
        y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
        t += dt;

      }
    }

    if(prev_size > 0 && prev_size < max_num_waypoints/2)
    {
      current_state = KL;
      ref_vel = car_speed_ms;
      cout << "REF_VEL: " << ref_vel << endl;
    }

  }
  else if(current_state == KL)
  {
    double lane_dist = sensor_lead[lane][0];
    double lane_velocity = sensor_lead[lane][1];
    if(lane_velocity == -1.0 || lane_dist > kl_buffer_dist)
      target_vel = max_vel;
    else
      target_vel = lane_velocity;
      
    if(ref_vel < target_vel && (lane_dist > min_buffer_dist || lane_dist == -1.0) )
    {
      ref_vel += (0.225/mph_ms);
    }
    else
    {
      ref_vel -= (0.225/mph_ms);
    }

    //Define first two points of spline
    //use last two end points to make path tanget to last two previous points 
    ptsx.clear();
    ptsy.clear();
    ref_x = next_x_vals[prev_size-1];
    ref_y = next_y_vals[prev_size-1];
    double ref_x_prev = next_x_vals[prev_size-2];
    double ref_y_prev = next_y_vals[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

    //Define next waypoints for spline
    for(int i = 0; i < 3; i++)
    {
      vector<double> next_wp = getXY(car_s + target_kl_dist*(i+1), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
      ptsx.push_back(next_wp[0]);
      ptsy.push_back(next_wp[1]);
    }

    for(int i = 0; i < ptsx.size(); i++)
    {
      //shift car reference angle to 0
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;
      ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y*sin(-ref_yaw));
      ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y*cos(-ref_yaw));

    }

    //Spline
    tk::spline s;
    s.set_points(ptsx, ptsy);
    double target_x = target_lc_dist;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    double x_addon = 0;
    for(int i = 0; i < (max_num_waypoints - prev_size); i++)
    {
      double N = target_dist/(0.02*ref_vel);
      double x_point = x_addon + target_x/N;
      double y_point = s(x_point);
      x_addon = x_point;
      double x_ref = x_point;
      double y_ref = y_point;
      x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
      y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
      x_point += ref_x;
      y_point += ref_y;
      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);
    }

    //State Transition
    if(target_vel < max_vel)
    {
      vector<double> lane_changes = lane_change_lanes[lane];
      double cost_threshold = 0.5;
      cout << "Calculate LC Cost" << endl;
      for(int i = 0; i < lane_changes.size(); i++)
      {
        int lc_lane = lane_changes[i];
        double lc_cost = trailing_car_cost(lc_lane) +
                         lead_car_cost(lc_lane);
        cout << "LC[" << lc_lane << "]: " << lc_cost << ", CT: "<< cost_threshold <<  endl;
        if(lc_cost < cost_threshold)
        {
          cost_threshold = lc_cost;
          lane = lc_lane;
          current_state = LC;
          lc_s = max_s + car_s + target_lc_dist + 10; 
        }
      }
    }
  }
  else if(current_state == LC)
  {
    double lane_dist = sensor_lead[lane][0];
    double lane_velocity = sensor_lead[lane][1];
    if(lane_velocity == -1.0 || lane_dist > kl_buffer_dist)
      target_vel = max_vel;
    else
      target_vel = lane_velocity;
      
    if(ref_vel < target_vel )
    {
      ref_vel += (0.225/mph_ms);
    }
    else
    {
      ref_vel -= (0.225/mph_ms);
    }
    //Define first two points of spline
    //use last two end points to make path tanget to last two previous points 
    ptsx.clear();
    ptsy.clear();
    ref_x = next_x_vals[prev_size-1];
    ref_y = next_y_vals[prev_size-1];
    double ref_x_prev = next_x_vals[prev_size-2];
    double ref_y_prev = next_y_vals[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

    //Define next waypoints for spline
    for(int i = 0; i < 3; i++)
    {
      vector<double> next_wp = getXY(car_s + target_lc_dist*(i+1), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
      ptsx.push_back(next_wp[0]);
      ptsy.push_back(next_wp[1]);
    }

    for(int i = 0; i < ptsx.size(); i++)
    {
      //shift car reference angle to 0
      double shift_x = ptsx[i] - ref_x;
      double shift_y = ptsy[i] - ref_y;
      ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y*sin(-ref_yaw));
      ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y*cos(-ref_yaw));

    }

    //Spline
    tk::spline s;
    s.set_points(ptsx, ptsy);

    double target_x = target_lc_dist;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    double x_addon = 0;
    for(int i = 0; i < (max_num_waypoints - prev_size); i++)
    {
      double N = target_dist/(0.02*ref_vel);
      double x_point = x_addon + target_x/N;
      double y_point = s(x_point);

      x_addon = x_point;

      double x_ref = x_point;
      double y_ref = y_point;

      x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
      y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

      x_point += ref_x;
      y_point += ref_y;

      next_x_vals.push_back(x_point);
      next_y_vals.push_back(y_point);

    }

    if((max_s + car_s) >= lc_s)
    {
      current_state = KL;
    }
  }

}

//weights - one for current lane, one for adjacent lane
double Vehicle::calculate_cost(vector<double> weights, int lane)
{
  double cost = 0.0;
  return 0.0;
}

double Vehicle::trailing_car_cost(int lane)
{
  double trail_car_speed = sensor_trail[lane][1];
  double trail_car_start = sensor_trail[lane][0];
  //If no car is detected 
  if(trail_car_start == -1.0)
    return 0.0;

  //Buffers
  double buffer_start = 10.0;
  double buffer_len = 5.0;

  double t_est = target_lc_dist/car_speed_ms;
  double trail_car_travel = t_est*(car_speed_ms - trail_car_speed);
  double trail_car_end = trail_car_start + trail_car_travel;

  double cost;
  if(trail_car_end < buffer_start)
  {
    cost = 1.0;
  }
  else
  {
    cost = exp(-(trail_car_end - buffer_start)/buffer_len);
  }
  return cost;
}

double Vehicle::lead_car_cost(int lane)
{
  double lead_car_speed = sensor_lead[lane][1];
  double lead_car_start = sensor_lead[lane][0];

  if(lead_car_start == -1.0)
    return 0.0;

  //Buffers
  double buffer_start = 15.0;
  double buffer_len = 7.5;

  double t_est = target_lc_dist/car_speed_ms;
  double lead_car_travel = t_est*(lead_car_speed - car_speed_ms);
  double lead_car_end = lead_car_start + lead_car_travel;

  double cost;
  if(lead_car_end < buffer_start)
  {
    cost = 1.0;
  }
  else
  {
    cost = exp(-(lead_car_end - buffer_start)/buffer_len);
  }
  return cost;
}

double Vehicle::ego_car_cost()
{
  double cost;

  cost = 0.0;


  return cost;
}







