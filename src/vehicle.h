#ifndef VEHICLE_H
#define VEHICLE_H
#include <vector>
using namespace std;
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
    Vehicle(double ref_velocity);
    ~Vehicle();
    vector<double> next_x_vals;
    vector<double> next_y_vals;
    vector<vector<double> > sensor_fusion;

    void update_data(double car_x, double car_y, double car_s, 
      double car_d, double car_yaw, double car_speed);

    void process_current_state();
    void process_state_transition();

  private:
    vector<state> successor_states();
    state current_state;
    double ref_vel;

    double car_s;
    double car_d;
    double car_x;
    double car_y;
    double car_yaw;
    double car_speed;
};

#endif
