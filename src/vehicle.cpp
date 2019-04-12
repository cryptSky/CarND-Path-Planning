#include <iostream>
#include <math.h>
#include "vehicle.h"
#include "helpers.h"

using namespace std;

double Vehicle::ref_vel = 0.;

Vehicle::Vehicle(int id, double x, double y, double s, double d, double speed)
{
    this->id = id;

    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->speed = speed;

    this->lane = calculateLane(d);

}

Vehicle::Vehicle(double x, double y, double s, double d, double yaw, const vector<double>& previous_path_x,  const vector<double>& previous_path_y)
{
    this->id = -1;

    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    this->yaw = yaw;
 
    this->previous_path_x = previous_path_x;
    this->previous_path_y = previous_path_y;

    this->lane = calculateLane(d);
}


void Vehicle::setVehicleParams(const vector<Vehicle>& other_vehicles)
{
  int prev_size = this->previous_path_x.size();

  bool too_close = false;
  bool car_left = false;
  bool car_right = false;

  for (auto& other_vehicle: other_vehicles)
  {
    double check_car_s = other_vehicle.s;
    check_car_s += ((double)prev_size*0.02*other_vehicle.speed);

    int gap = 30; // m

    if (other_vehicle.lane == this->lane) {
        // ahead
	too_close |= (check_car_s > this->s) && ((check_car_s - this->s) < gap);
    } else if (other_vehicle.lane - this->lane == 1) {
        // to the right
	car_right |= ((this->s - gap) < check_car_s) && ((this->s + gap) > check_car_s);
    } else if (this->lane - other_vehicle.lane == 1) {
        // to the left
        car_left |= ((this->s - gap) < check_car_s) && ((this->s + gap) > check_car_s);
    }
   
  }

  
    double acc = 0.224;
    double max_speed = 49.5;

    if (too_close) {
    	if (!car_right && this->lane < 2) {
			this->lane++;
		} else if (!car_left && this->lane > 0) {
			this->lane--;
		} else {
			// slow down
			ref_vel -= acc;
		}
	} else {
		if (this->lane != 1) {
			// Not in the center lane. Check if it is safe to move back
			if ((this->lane == 2 && !car_left) || (this->lane == 0 && !car_right)) {
				// Move back to the center lane
				this->lane = 1;
			}
		}
		
		if (this->ref_vel < max_speed) {
			ref_vel += acc;
		}
	}
  
}

vector<vector<double>> Vehicle::processTrajectory(const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y)
{
	  vector<vector<double>> result;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = this->x;
          double ref_y = this->y;
          double ref_yaw = deg2rad(this->yaw);

          int prev_size = this->previous_path_x.size();

          if (prev_size < 2)
          {
            double prev_car_x = this->x - cos(this->yaw);
            double prev_car_y = this->y - sin(this->yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(this->x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(this->y);
          }
          else
          {
            ref_x = this->previous_path_x[prev_size-1];
            ref_y = this->previous_path_y[prev_size-1];

            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);

          }

          vector<double> next_wp0 = getXY(this->s+30,(2+4*this->lane), map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(this->s+60,(2+4*this->lane), map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(this->s+90,(2+4*this->lane), map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i = 0; i < ptsx.size(); ++i) {

            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

          }

          tk::spline s;
          s.set_points(ptsx,ptsy);

          for (int i = 0; i < this->previous_path_x.size(); i++)
          {
            next_x_vals.push_back(this->previous_path_x[i]);
            next_y_vals.push_back(this->previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

          double x_add_on = 0;

          for (int i = 1; i <= 50-this->previous_path_x.size(); i++) {
            double N = (target_dist/(.02*ref_vel/2.24));
            double x_point = x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);			

          }		
	
		  result.push_back(next_x_vals);
		  result.push_back(next_y_vals);

	return result;
}

Vehicle::~Vehicle() {}
