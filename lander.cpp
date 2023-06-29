// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"
#include <vector>
#include <fstream>
double P_out, error, Kh, Kp,delta;
vector<double> t_list, h_list, v_list;


void autopilot (void)
  // Autopilot to adjust the engine throttle, parachute and attitude control
{
    Kp = 0.7;
    Kh = 0.019; //0.019
    delta = 0.6; //these constants works well for case 1, 3, 5 (all can be done even without parachute!),  case 4 can be done with parachute 
    P_out = Kp * (-(0.5 + Kh * (sqrt((position * position))-MARS_RADIUS) + velocity * position.norm()));


    //From the python plotting, it is clear that the real velocity fits extremely well with the targeting velocity when Kh=0.019 
    //For the v-h diagram:
    // If the constant Kh is to high, eg 0.05, the real velocity cannot fit well with (catch up) the target velocity, thus leading to crashing
    // If the constant Kh is to low, eg 0.01, the lander starts to decrease speed too early. Thus the lander will ends up no fuel before landing
    

    if (P_out<=-delta) 
    {
        throttle = 0;
    }
    else if (P_out>=1-delta) 
    {
        throttle = 1;
    }
    else 
    {
        throttle = delta + P_out;
    }
}

int i;
vector3d previous_position_2;
vector3d previous_position_1;

void numerical_dynamics (void)
  // This is the function that performs the numerical integration to update the
  // lander's pose. The time step is delta_t (global variable).

{
   vector3d acceleration;
   vector3d drag;
   vector3d gravity;
   
   
   
   drag = (-DRAG_COEF_LANDER * 3.14 * LANDER_SIZE * LANDER_SIZE * (velocity * velocity) * atmospheric_density(position)) * velocity.norm() / 2;
   gravity = (-GRAVITY * MARS_MASS * (UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY) * position.norm()) / (position * position);
   

   if ((parachute_status == DEPLOYED)) 
   {
       drag =drag+ ( - DRAG_COEF_CHUTE * 5 * 2 * LANDER_SIZE * LANDER_SIZE * (velocity * velocity) * atmospheric_density(position))* velocity.norm() / 2;
  }
   acceleration = (thrust_wrt_world() + drag + gravity) / (UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY);
   
   //Euler
   //position = position + delta_t * velocity;
   //velocity = velocity + delta_t * acceleration;
   //cout << velocity << endl;

    //Test cases:
    // 1.exactly 176.1m/s, 83.5s (1)
    // 2. Not Correct, performs like circular motion and does that land/crash on time. This is possibly due to the error accumulation and thus it might not be suitable to use here.
    // 3. 330.05m/s, 361.6s (5)

   //Verlet:
   if (i == 0) {
       previous_position_2 = position;
       position = previous_position_2 + delta_t * velocity;
       previous_position_1 = position;
       velocity = velocity + delta_t * acceleration;
       i += 1; 
  }
   else {
      position = 2 * previous_position_1 - previous_position_2 + (delta_t * delta_t) * acceleration;
       velocity = (previous_position_1 - previous_position_2) / delta_t;
       previous_position_2 = previous_position_1;
       previous_position_1 = position;
   

       cout << simulation_time << ' ' << sqrt(position * position) - MARS_RADIUS << ' ' << velocity * position.norm() << endl;
       

   //Test cases:
    // 1.exactly 176.156m/s, 83.5s (1)
    // 2.197m/s, 42650s (4) Very obvious discrepancy from the given data, however still in the acceptable range(4)
    // 3. 328.279m/s,361.9s (5)
   }
   

  // Here we can apply an autopilot to adjust the thrust, parachute and attitude
  if (autopilot_enabled) autopilot();

  // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
  if (stabilized_attitude) attitude_stabilization();
}

void initialize_simulation (void)
  // Lander pose initialization - selects one of 10 possible scenarios
{
  // The parameters to set are:
  // position - in Cartesian planetary coordinate system (m)
  // velocity - in Cartesian planetary coordinate system (m/s)
  // orientation - in lander coordinate system (xyz Euler angles, degrees)
  // delta_t - the simulation time step
  // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
  // scenario_description - a descriptive string for the help screen

  scenario_description[0] = "circular orbit";
  scenario_description[1] = "descent from 10km";
  scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
  scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
  scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
  scenario_description[5] = "descent from 200km";
  scenario_description[6] = "";
  scenario_description[7] = "";
  scenario_description[8] = "";
  scenario_description[9] = "";

  switch (scenario) {

  case 0:
    // a circular equatorial orbit
    position = vector3d(1.2*MARS_RADIUS, 0.0, 0.0);
    velocity = vector3d(0.0, -3247.087385863725, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    i = 0;
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 1:
    // a descent from rest at 10km altitude
    position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    i = 0;
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 2:
    // an elliptical polar orbit
    position = vector3d(0.0, 0.0, 1.2*MARS_RADIUS);
    velocity = vector3d(3500.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    i = 0;
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 3:
    // polar surface launch at escape velocity (but drag prevents escape)
    position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE/2.0);
    velocity = vector3d(0.0, 0.0, 5027.0);
    orientation = vector3d(0.0, 0.0, 0.0);
    i = 0;
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 4:
    // an elliptical orbit that clips the atmosphere each time round, losing energy
    position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
    velocity = vector3d(4000.0, 0.0, 0.0);
    orientation = vector3d(0.0, 90.0, 0.0);
    i = 0;
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = false;
    autopilot_enabled = false;
    break;

  case 5:
    // a descent from rest at the edge of the exosphere
    position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
    velocity = vector3d(0.0, 0.0, 0.0);
    orientation = vector3d(0.0, 0.0, 90.0);
    i = 0;
    delta_t = 0.1;
    parachute_status = NOT_DEPLOYED;
    stabilized_attitude = true;
    autopilot_enabled = false;
    break;

  case 6:
    break;

  case 7:
    break;

  case 8:
    break;

  case 9:
    break;

  }
}

