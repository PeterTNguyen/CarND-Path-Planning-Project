#include "vehicle.h"



Vehicle::Vehicle()
{
  current_state = RDY;
  Vehicle::ref_vel = 40.0;
}

Vehicle::Vehicle(double ref_velocity)
{
  current_state = RDY;
  Vehicle::ref_vel = ref_velocity;
}

Vehicle::~Vehicle()
{
}

void Vehicle::update_data(double car_x, double car_y, double car_s, 
  double car_d, double car_yaw, double car_speed, auto previous_path_x)
{
}

vector<state> Vehicle::successor_states()
{
  vector<state> next_states;
  switch(current_state)
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
  }
  return next_states;
}





