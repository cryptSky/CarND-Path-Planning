#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <vector>

using namespace std;

class Vehicle
{
    public:
        int id;
        
        double x;
        double y;
        
        double s;
        double d;
 
        double speed;

        vector<double> previous_path_x;
        vector<double> previous_path_y;

        // We can compute those
        int lane;
        static double ref_vel;

        double yaw;

        Vehicle(); 
        Vehicle(int id, double x, double y, double s, double d, double speed);

	Vehicle(double x, double y, double s, double d, double yaw, const vector<double>& previous_path_x,  const vector<double>& previous_path_y);
        virtual ~Vehicle();

        void setVehicleParams(const vector<Vehicle>& vehicles);

        vector<vector<double>> processTrajectory(const vector<double>& map_waypoints_s, const vector<double>& map_waypoints_x, const vector<double>& map_waypoints_y);
        
        
};
#endif
