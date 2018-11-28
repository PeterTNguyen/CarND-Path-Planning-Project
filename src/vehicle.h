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
  private:
    vector<state> successor_states();
    state current_state;
    double ref_vel;

};

#endif
