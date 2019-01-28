#ifndef __PATH_PLANNER__
#define __PATH_PLANNER__
#include <vector>
#include <iostream>
#include <math.h>
#include <chrono>
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

using namespace std;
constexpr double pi() { return M_PI; }

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

double distance(double x1, double y1, double x2, double y2);

class PathPlanner {
	public:
        PathPlanner (
                double target_speed,
                unsigned int lane_id,
                double spacing,
                unsigned int total_points,
                vector<double> map_waypoints_x,
                vector<double> map_waypoints_y,
                vector<double> map_waypoints_s,
                vector<double> map_waypoints_dx,
                vector<double> map_waypoints_dy); 
        double get_target_speed () const { return target_speed_; } ;
		double setTargetSpeed(const double new_speed);
		void generateNewTrajectoryWithMinJerk(vector<double> prev_path_x,
												vector<double> prev_path_y, 
											 double car_x, 
											 double car_y, 
											 double car_s, 
											 double car_d,
											 double car_yaw,
											 vector<double> &next_x_vals,
											 vector<double> &next_y_vals,
											 double target_x,
											 vector<vector<double> > sensor_fusion);
		
	private:
		double target_speed_;
        int lane_id_;
		bool changing_lane_complete_ = true;
        double spacing_;
        double safety_distance_ = 25.0; // m
		unsigned int total_points_;
		vector<double> map_waypoints_x_;
  		vector<double> map_waypoints_y_;
 		vector<double> map_waypoints_s_;
  		vector<double> map_waypoints_dx_;
 		vector<double> map_waypoints_dy_;
		/* cost function */
        bool isLaneEmptyForChange(int lane_id,
                          int car_s,
                          vector<vector<double> > sensor_fusion, 
                          vector<double> &maps_x, 
                          vector<double> &maps_y,
                          int prev_size);

		int changeLaneIfPossible(double car_x, 
                                       double car_y, 
                                       double car_s,
                                       vector<double> &maps_x, 
                                       vector<double> &maps_y, 
                                       vector<vector<double> > sensor_fusion,
                                       int prev_size);
 
        double calculateTrajectoryCost(void);
		bool needToSlowDown(vector<vector<double> > sensor_fusion, double car_s, double car_d, int prev_size);
		bool canSpeedUp(vector<vector<double> > sensor_fusion, double car_s, double car_d, int prev_size);
		/* initialize reference points to start planning trajectory */
		void initializeReferencePoints(const vector<double> &previous_path_x, 
											const vector<double> &previous_path_y, 
											const double car_yaw,
											const double car_x,
											const double car_y,
											vector<double> &pts_x, 
											vector<double> &pts_y,
											double &ref_yaw,
											double &ref_x,
											double &ref_y );
		void addFixAnchorPoints(vector<double> &x , vector<double> &y, double car_s);
};









#endif // __PATH_PLANNER__
