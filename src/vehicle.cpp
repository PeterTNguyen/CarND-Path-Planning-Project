#include "vehicle.h"


static const double dt = 0.02;
static const double max_vel = 45.0;
static const int    max_num_waypoints = 50;
static const double kl_buffer_dist = 30.0;
static const double min_buffer_dist = 15.0;

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
      //user two points that make path tangent to referencee
      double prev_car_x = car_x - cos(car_yaw);
      double prev_car_y = car_y - sin(car_yaw);
      ptsx.push_back(prev_car_x);
      ptsy.push_back(prev_car_y);

      vector<double> next_wp0 = getXY(car_s + 30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
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
      ref_vel = car_speed;
    }

  }
  else if(current_state == KL)
  {
    double min_s = 10000;
    target_vel = max_vel;
    for(int i = 0 ; i < sensor_fusion.size(); i++)
    {
      double sensor_d = sensor_fusion[i][6];
      double sensor_s = sensor_fusion[i][5];
      if(sensor_d > lane*4.0 && sensor_d < (lane+1)*4.0)
      {
        if(sensor_s > car_s)
        {
          double s_diff = sensor_s - car_s;
          if(s_diff < min_s && s_diff < kl_buffer_dist)
          {
            min_s = s_diff;
            double sensor_vx = sensor_fusion[i][3];
            double sensor_vy = sensor_fusion[i][4];

            target_vel =sqrt(sensor_vx*sensor_vx + sensor_vy*sensor_vy) * 2.24;
          }
        }
      }
    }
    if(ref_vel < target_vel && min_s > min_buffer_dist)
    {
      ref_vel += 0.225;
    }
    else
    {
      ref_vel -= 0.225;
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
      vector<double> next_wp = getXY(car_s + 30*(i+1), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
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

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    double x_addon = 0;
    for(int i = 0; i < (max_num_waypoints - prev_size); i++)
    {
      double N = target_dist/(0.02*ref_vel/2.24);
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
    if(target_vel < 40)
    {
      current_state = LCL;
      lane = 0;
    }
  }
  else if(current_state == PLCL)
  {
  }
  else if(current_state == LCL)
  {
    lane = 0;
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
      vector<double> next_wp = getXY(car_s + 30*(i+1), (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
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

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);
    double x_addon = 0;
    for(int i = 0; i < (50 - prev_size); i++)
    {
      double N = target_dist/(0.02*ref_vel/2.24);
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
  }
  else if(current_state == PLCR)
  {
  }
  else if(current_state == LCR)
  {
  }
  else
  {
  }

}

void Vehicle::process_state_transition()
{
  vector<state> next_states = this->successor_states();
  switch(this->current_state)
  {
    case RDY:
      break;
    case KL:
     break; 
    case PLCL:
     break;
    case LCL:
     break;
    case PLCR:
     break;
    case LCR:
     break;
    default:
     break;
  }
}


vector<state> Vehicle::successor_states()
{
  vector<state> next_states;
  switch(this->current_state)
  {
    case RDY:
      next_states.push_back(RDY);
      next_states.push_back(KL);
      break;
    case KL:
      next_states.push_back(KL);
      next_states.push_back(PLCL);
      next_states.push_back(PLCR);
     break; 
    case PLCL:
      next_states.push_back(PLCL);
      next_states.push_back(KL);
      next_states.push_back(LCL);
     break;
    case LCL:
      next_states.push_back(LCL);
      next_states.push_back(KL);
     break;
    case PLCR:
      next_states.push_back(PLCR);
      next_states.push_back(KL);
      next_states.push_back(LCR);
     break;
    case LCR:
      next_states.push_back(LCR);
      next_states.push_back(KL);
     break;
    default:
     break;
  }
  return next_states;
}





