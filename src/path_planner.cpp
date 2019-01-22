#include "path_planner.h"



using namespace std;
using namespace tk;
static double deg2rad(double x) { return x * pi() / 180; }
static double rad2deg(double x) { return x * 180 / pi(); }

static void print_double(vector<double> v) {
	for (double n: v) {
		std::cout << n <<" ";
	}
	std::cout << std::endl;
}

double PathPlanner::calculateTrajectoryCost(void) {

	return 0.0;
}

PathPlanner::PathPlanner (
		double target_speed,
		unsigned int lane_id,
		double spacing,
		unsigned int total_points,
		vector<double> map_waypoints_x,
  		vector<double> map_waypoints_y,
 		vector<double> map_waypoints_s,
  		vector<double> map_waypoints_dx,
 		vector<double> map_waypoints_dy) {

	this->target_speed_ = target_speed;
	this->lane_id_ = lane_id;
	this->total_points_ = total_points;
	this->map_waypoints_x_ = map_waypoints_x;
	this->map_waypoints_y_ = map_waypoints_y;
	this->map_waypoints_s_ = map_waypoints_s;
	this->map_waypoints_dx_ = map_waypoints_dx;
	this->map_waypoints_dy_ = map_waypoints_dy;
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

void PathPlanner::initializeReferencePoints(const vector<double> &previous_path_x, 
											const vector<double> &previous_path_y, 
											const double car_yaw,
											const double car_x,
											const double car_y,
											vector<double> &pts_x, 
											vector<double> &pts_y,
											double &ref_yaw,
											double &ref_x,
											double &ref_y ) {

	int prev_size = previous_path_x.size();
	if (prev_size < 2) {
		std::cout << "Initialized with zero points !" << std::endl;
		double prev_x = car_x - cos(car_yaw);
		double prev_y = car_y - sin(car_yaw);

		pts_x.push_back(prev_x);
		pts_x.push_back(car_x);

		pts_y.push_back(prev_y);
		pts_y.push_back(car_y);
		
		ref_x = car_x;
		ref_y = car_y;
		ref_yaw = deg2rad(car_yaw);
	} else {
		std::cout << "Initialized with points !" << std::endl;
		std::cout << "previous_path_x" << std::endl;
		//print_double(previous_path_x);

		double prev_x = previous_path_x[prev_size - 1];
		double prev_y = previous_path_y[prev_size - 1];
		
		double prev_prev_x = previous_path_x[prev_size - 2];
		double prev_prev_y= previous_path_y[prev_size - 2];
		
		std::cout << prev_x << "," << prev_y << std::endl;
		std::cout << prev_prev_x << "," << prev_prev_y << std::endl;

		pts_x.push_back(prev_prev_x);
		pts_x.push_back(prev_x);
		
		pts_y.push_back(prev_prev_y);
		pts_y.push_back(prev_y);
		
		ref_x = prev_x;
		ref_y = prev_y;
		ref_yaw = atan2(prev_y - prev_prev_y, prev_x - prev_prev_x);
	}
}

void PathPlanner::addFixAnchorPoints(vector<double> &x , vector<double> &y, double car_s) {
	vector<double> p1 = getXY(car_s + 30, (2 + 4 * lane_id_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
	vector<double> p2 = getXY(car_s + 60, (2 + 4 * lane_id_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
	vector<double> p3 = getXY(car_s + 90, (2 + 4 * lane_id_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
	
	x.push_back(p1[0]);
	x.push_back(p2[0]);
	x.push_back(p3[0]);

	y.push_back(p1[1]);
	y.push_back(p2[1]);
	y.push_back(p3[1]);
}

tk::spline initializeSpline(const vector<double> x_pts, const vector<double> y_pts) {

	tk::spline spline_fit;
	spline_fit.set_points(x_pts, y_pts);
	return spline_fit;
}

void transformToCarRef(vector<double> &pts_x, vector<double> &pts_y, const double ref_yaw, const double ref_x, const double ref_y) {
	for (int i = 0; i < pts_x.size(); i++){
		double shift_x = pts_x[i] - ref_x;
		double shift_y = pts_y[i] - ref_y;
		pts_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 -ref_yaw) ;
		pts_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
	}
}

void transformToGlobal(vector<double> &pts_x, vector<double> &pts_y, const double ref_yaw, const double ref_x, const double ref_y) {
	for (int i = 0; i < pts_x.size(); i++){
		double x_pt = pts_x[i];
		double y_pt = pts_y[i];
		pts_x[i] = (x_pt * cos(ref_yaw) - y_pt * sin(ref_yaw)) + ref_x ;
		pts_y[i] = (x_pt * sin(ref_yaw) + y_pt * cos(ref_yaw)) + ref_y ;
	}
}


vector<double> transformSinglePointToGlobal(double x_pt, double y_pt, const double ref_yaw, const double ref_x, const double ref_y) {
		double g_x = (x_pt * cos(ref_yaw) - y_pt * sin(ref_yaw)) + ref_x ;
		double g_y = (x_pt * sin(ref_yaw) + y_pt * cos(ref_yaw)) + ref_y ;
		vector<double> result;
		result.push_back(g_x);
		result.push_back(g_y);
		return result;
}

void generateNewPointsGivenTarget(double target_x, 
								  tk::spline s_fit, 
								  vector<double> &next_x_vals,
								  vector<double> &next_y_vals, 
								  int prev_path_size,
								  double ref_x,
								  double ref_y,
								  double ref_yaw,
								  int total_points,
								  double target_speed) {

	double target_y = s_fit(target_x);
	double target_dist = sqrt(target_x * target_x + target_y * target_y);
	double x_add_on = 0;
	for (int i = 1; i < total_points - prev_path_size; i++) {
			double N = target_dist / (0.02 * target_speed/2.24); // 200ms * speed => convert to m/s
			double x_point = i * (target_x / N);
			double y_point = s_fit(x_point);
			vector<double> point_glob = transformSinglePointToGlobal(x_point, y_point, ref_yaw, ref_x, ref_y);
			next_x_vals.push_back(point_glob[0]);
			next_y_vals.push_back(point_glob[1]);
			//x_add_on = x_point; 
	}
}



 
void PathPlanner::generateNewTrajectoryWithMinJerk(vector<double> prev_path_x,
												vector<double> prev_path_y, 
											 double car_x, 
											 double car_y, 
											 double car_s, 
											 double car_yaw,
											 vector<double> &next_x_vals,
											 vector<double> &next_y_vals,
											 double target_x) {

	// 1.Calculate current heading angle 
	
	vector<double> pts_x;
	vector<double> pts_y;
	double ref_x;
	double ref_y;
	double ref_yaw;
	initializeReferencePoints(prev_path_x, prev_path_y, car_yaw, car_x, car_y, pts_x, pts_y, ref_yaw, ref_x, ref_y);

	addFixAnchorPoints(pts_x, pts_y, car_s);
	// need to convert all points to car ref 
	transformToCarRef(pts_x, pts_y, ref_yaw, ref_x, ref_y);
	

	// initialize spline to fit the next points
	tk::spline spline_fit = initializeSpline(pts_x, pts_y);

	std::cout << "Points " << endl;
	print_double(pts_x);
	print_double(pts_y);


	// push old prev points to new path to smooth the trajectory
	for (int i = 0; i < prev_path_x.size(); i++) {
		next_x_vals.push_back(prev_path_x[i]);
		next_y_vals.push_back(prev_path_y[i]);
	}

	

	generateNewPointsGivenTarget(target_x, 
								spline_fit, 
								next_x_vals, 
								next_y_vals, 
								prev_path_x.size(),
								ref_x,
								ref_y,
								ref_yaw,
								total_points_,
								target_speed_
								);

	std::cout << "Output Points " << endl;
	print_double(next_x_vals);
	print_double(next_y_vals);
}