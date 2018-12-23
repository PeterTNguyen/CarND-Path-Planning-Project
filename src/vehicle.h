#ifndef VEHICLE_H
#define VEHICLE_H
#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <thread>
#include "spline.h"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using namespace tk;
using Eigen::MatrixXd;
using Eigen::VectorXd;

enum state {
  RDY=0,
  KL,
  PLCL,
  LCL,
  PLCR,
  LCR
};

class Vehicle
{
  public:
    Vehicle();
    ~Vehicle();
    //Next waypoints
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    vector<double> s_i;
    vector<double> s_f;
    //points for spline
    vector<double> ptsx;
    vector<double> ptsy;
    //Sensor fusion data
    vector<vector<double> > sensor_fusion;
    vector<vector<int> > sensor_id_by_lane;
    vector<vector<double> > sensor_trail;
    vector<vector<double> > sensor_lead;
    //Mapwaypoints
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    void update_data(double car_x, double car_y, double car_s, 
      double car_d, double car_yaw, double car_speed, int prev_size);

    void process_current_state();
    void process_state_transition();

  private:
    //State info
    vector<state> successor_states();
    state current_state;
    spline spline_s;

    //pathing variables
    double ref_vel;
    double target_vel;
    int lane;
    int prev_size;
    VectorXd alpha;
    double x_temp;
    double T;

    //Car parameters
    double car_s;
    double car_d;
    double car_x;
    double car_y;
    double car_yaw;
    double car_speed;
    static constexpr double max_s = 6945.554;

    //Helper functions from main.cpp
    static constexpr double pi() { return M_PI; }
    static double deg2rad(double x) { return x * pi() / 180; }
    static double rad2deg(double x) { return x * 180 / pi(); }
    void calculateJMT(double s_i, double s_f, double s_i1,
                      double s_f1, double s_i2, double s_f2, double T);
    static double distance(double x1, double y1, double x2, double y2)
    {
      return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }
    static int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
    {
    
      double closestLen = 100000; //large number
      int closestWaypoint = 0;
    
      for(int i = 0; i < maps_x.size(); i++)
      {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
          closestLen = dist;
          closestWaypoint = i;
        }
      }
      return closestWaypoint;
    }


    static int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
    {
    
      int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
    
      double map_x = maps_x[closestWaypoint];
      double map_y = maps_y[closestWaypoint];
    
      double heading = atan2((map_y-y),(map_x-x));
    
      double angle = fabs(theta-heading);
      angle = min(2*pi() - angle, angle);
    
      if(angle > pi()/4)
      {
        closestWaypoint++;
      if (closestWaypoint == maps_x.size())
      {
        closestWaypoint = 0;
      }
      }
    
      return closestWaypoint;
    }
    
    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    static vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
    {
      int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
    
      int prev_wp;
      prev_wp = next_wp-1;
      if(next_wp == 0)
      {
        prev_wp  = maps_x.size()-1;
      }
    
      double n_x = maps_x[next_wp]-maps_x[prev_wp];
      double n_y = maps_y[next_wp]-maps_y[prev_wp];
      double x_x = x - maps_x[prev_wp];
      double x_y = y - maps_y[prev_wp];
    
      // find the projection of x onto n
      double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
      double proj_x = proj_norm*n_x;
      double proj_y = proj_norm*n_y;
    
      double frenet_d = distance(x_x,x_y,proj_x,proj_y);
    
      //see if d value is positive or negative by comparing it to a center point
    
      double center_x = 1000-maps_x[prev_wp];
      double center_y = 2000-maps_y[prev_wp];
      double centerToPos = distance(center_x,center_y,x_x,x_y);
      double centerToRef = distance(center_x,center_y,proj_x,proj_y);
    
      if(centerToPos <= centerToRef)
      {
        frenet_d *= -1;
      }
    
      // calculate s value
      double frenet_s = 0;
      for(int i = 0; i < prev_wp; i++)
      {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
      }
    
      frenet_s += distance(0,0,proj_x,proj_y);
    
      return {frenet_s,frenet_d};
    
    }
    
    // Transform from Frenet s,d coordinates to Cartesian x,y
    static vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
    {
      int prev_wp = -1;
    
      while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
      {
        prev_wp++;
      }
    
      int wp2 = (prev_wp+1)%maps_x.size();
    
      double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
      // the x,y,s along the segment
      double seg_s = (s-maps_s[prev_wp]);
    
      double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
      double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
    
      double perp_heading = heading-pi()/2;
    
      double x = seg_x + d*cos(perp_heading);
      double y = seg_y + d*sin(perp_heading);
    
      return {x,y};
    
    }
};

#endif
