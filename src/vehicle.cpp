#include "vehicle.h"



Vehicle::Vehicle()
{
  current_state = RDY;
  Vehicle::ref_vel = 40.0;
}

Vehicle::Vehicle(double ref_velocity)
{
  current_state = RDY;
  this->ref_vel = ref_velocity;
}

Vehicle::~Vehicle()
{
}

void Vehicle::update_data(double car_x, double car_y, double car_s, 
  double car_d, double car_yaw, double car_speed)
{
  this->car_x = car_x;
  this->car_y = car_y;
  this->car_s = car_s;
  this->car_d = car_d;
  this->car_yaw = car_yaw;
  this->car_speed = car_speed;
}

void Vehicle::process_current_state()
{
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





