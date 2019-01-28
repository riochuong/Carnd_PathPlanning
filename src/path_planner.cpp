#include "path_planner.h"



using namespace std;
using namespace tk;
bool isInSameLane(int lande_id, double other_d);
static double deg2rad(double x) { return x * pi() / 180; }
static double rad2deg(double x) { return x * 180 / pi(); }

static void print_double(vector<double> v) {
	for (double n: v) {
		std::cout << n <<" ";
	}
	std::cout << std::endl;
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
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

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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


bool PathPlanner::isLaneEmptyForChange(int lane_id,
                          int car_s,
                          vector<vector<double> > sensor_fusion, 
                          vector<double> &maps_x, 
                          vector<double> &maps_y,
                          int prev_size) {

    for (vector<double> car_data: sensor_fusion) {
		double other_car_x = car_data[1];
		double other_car_y = car_data[2];
		double other_car_vx = car_data[3];
		double other_car_vy = car_data[4];
		double other_car_s = car_data[5];
		double other_car_d = car_data[6];
		double other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
        if (isInSameLane(lane_id, other_car_d)) {
            // car is in the lane we are looking for 
            std::cout << "Our car is at:  " << car_s << std::endl;
            int other_car_wp =  ClosestWaypoint(other_car_x, other_car_y, maps_x, maps_y);
		    double dx =  map_waypoints_x_[other_car_wp];
		    double dy =  map_waypoints_y_[other_car_wp];
            double vd = other_car_vx * dx + other_car_vy * dy;
            double vs = other_car_speed * other_car_speed / vd;
            double other_car_pred_s = other_car_s + vs * prev_size * 0.02; 
            // check if it is possible to cross  
            std::cout << "Car on lane: " << lane_id << "  is at:  " << other_car_pred_s << std::endl;
            std::cout << "Diff s: " << car_s - other_car_pred_s << std::endl;
            if (other_car_pred_s <= car_s) {
                // other car is behind
                if ((car_s - other_car_pred_s) < 20.0) { 
                    
                    std::cout << "other car is too fast behind" << std::endl;
                    return false; 
                } // toof fast 
                else if (((car_s - other_car_pred_s) < 35.0) && (other_car_speed >= target_speed_))  {  
                    std::cout << "other car is too close behind" << std::endl;
                    return false;
                } // too close  
            }
            else {
                // other car is ahead 
                if ((other_car_pred_s - car_s) < 20.0) { 
                    std::cout << "other car is too close ahead" << std::endl;
                    return false;
                } // too close 
               else if ((other_car_speed <  (target_speed_)) && (other_car_s - car_s) < 35.0) { 
                    std::cout << "other car is slow close ahead" << std::endl;
                    return false; 
                } // too slow
            }
        }

    }
    return true;
}

int PathPlanner::changeLaneIfPossible(double car_x, 
                                       double car_y, 
                                       double car_s,
                                       vector<double> &maps_x, 
                                       vector<double> &maps_y, 
                                       vector<vector<double> > sensor_fusion,
                                       int prev_size) {
    bool left_change = false;
    bool right_change = false;
    int car_closest_waypoint = ClosestWaypoint(car_x, car_y, maps_x, maps_y);
    if (this->lane_id_ == 1) {
       left_change =  isLaneEmptyForChange(0, car_s, sensor_fusion, maps_x, maps_y, prev_size);
       std::cout << " =========================== " << std::endl;
       right_change =  isLaneEmptyForChange(2, car_s, sensor_fusion, maps_x, maps_y, prev_size);
       std::cout << " =========================== " << std::endl;
       std::cout << "Left Lane Can Be changed:  " <<  left_change << std::endl;
       std::cout << "Right Lane Can Be changed:  " << right_change << std::endl;
       if (left_change) { return 0;}
       if (right_change) { return 2;}
    } 
    else if (this->lane_id_ == 0) {
       if  (isLaneEmptyForChange(1, car_s, sensor_fusion, maps_x, maps_y, prev_size)){
           std::cout << "Right Lane Can Be changed:  " << right_change << std::endl;
           return 1;
       } 
    }
    else if (this->lane_id_ == 2) {
       if (isLaneEmptyForChange(1, car_s, sensor_fusion, maps_x, maps_y, prev_size)) {
           std::cout << "Left Lane Can Be changed:  " << right_change << std::endl;
           return 1;
        }
    }
    return -1;
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
		print_double(previous_path_x);

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
	vector<double> p1 = getXY(car_s + 50, (2 + 4 * lane_id_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
	vector<double> p2 = getXY(car_s + 100, (2 + 4 * lane_id_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
	vector<double> p3 = getXY(car_s + 150, (2 + 4 * lane_id_), map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
	
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
	double N = target_dist / (0.02 * target_speed/2.24);  // 200ms * speed => convert to m/s
	double spacing = target_x / N;
	// generate all points 
	for (int i = 1; i < total_points - prev_path_size; i++) {		
			double x_point = i * spacing;
			double y_point = s_fit(x_point);
			vector<double> point_glob = transformSinglePointToGlobal(x_point, y_point, ref_yaw, ref_x, ref_y);
			next_x_vals.push_back(point_glob[0]);
			next_y_vals.push_back(point_glob[1]);
	}
}

bool isChangeLaneComplete(int lane_id, double other_d) {
    
    double middle_of_lane = 2 + 4*lane_id;
    double diff = abs(middle_of_lane - other_d);
    
    std::cout << "diff lane: " << diff << std::endl;
	if (diff <= 0.2){ 
		return true;
	}
	return false;
}

bool isInSameLane(int lane_id, double other_d){
	if ( (other_d < (2 + 4*lane_id + 2)) && (other_d > (2 + 4*lane_id - 2))) {
		return true;
	}
	return false;
}

bool PathPlanner::needToSlowDown(vector<vector<double> > sensor_fusion, double car_s, double car_d, int prev_size) {
	// check if any car is in the same lane and is in front of our car within 20m
	bool need_slow_down = false;
	for (vector<double> car_data: sensor_fusion) {	
		double other_car_vx = car_data[3];
		double other_car_vy = car_data[4];
		double other_car_s = car_data[5];
		double other_car_d = car_data[6];
		double other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
		if (isInSameLane(this->lane_id_, other_car_d)) {
			double predict_other_car_s = other_car_s + 0.02 * other_car_speed *prev_size;
			if (predict_other_car_s > car_s && ((predict_other_car_s - car_s) < safety_distance_)) {
				return true;
			}
		}
	} 
}

bool PathPlanner::canSpeedUp(vector<vector<double> > sensor_fusion, double car_s, double car_d, int prev_size) {
	// check if any car is in the same lane and is in front of our car within 20m
	bool need_slow_down = false;
	for (vector<double> car_data: sensor_fusion) {	
		double other_car_vx = car_data[3];
		double other_car_vy = car_data[4];
		double other_car_s = car_data[5];
		double other_car_d = car_data[6];
		double other_car_speed = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
		if (isInSameLane(this->lane_id_, other_car_d)) {
			double predict_other_car_s = other_car_s + 0.02 * other_car_speed *prev_size;
			if (predict_other_car_s > car_s && ((predict_other_car_s - car_s) < (safety_distance_ + 15))) {
				return false;
			}
		}
	}
	return true; 
}



 
void PathPlanner::generateNewTrajectoryWithMinJerk(vector<double> prev_path_x,
												vector<double> prev_path_y, 
											 double car_x, 
											 double car_y, 
											 double car_s, 
											 double car_d,
											 double car_yaw,
											 vector<double> &next_x_vals,
											 vector<double> &next_y_vals,
											 double target_x,
											 vector<vector<double> > sensor_fusion) {

	// 1.Calculate current heading angle 
	
	vector<double> pts_x;
	vector<double> pts_y;
	double ref_x;
	double ref_y;
	double ref_yaw;
	initializeReferencePoints(prev_path_x, prev_path_y, car_yaw, car_x, car_y, pts_x, pts_y, ref_yaw, ref_x, ref_y);
	std::cout << "Points " << endl;
	print_double(pts_x);
	print_double(pts_y);
	// addFixAnchorPoints(pts_x, pts_y, car_s);
	
	// need to convert all points to car ref 
	// transformToCarRef(pts_x, pts_y, ref_yaw, ref_x, ref_y);

    if (!changing_lane_complete_ && isChangeLaneComplete(lane_id_, car_d)) {
        changing_lane_complete_ = true;
        std::cout << "CHANGE LANE COMPLETE !!! " << std::endl;
    }

    if (changing_lane_complete_) {
        if (needToSlowDown(sensor_fusion, car_s, car_d, prev_path_x.size())) {
            std::cout << "Need to slow down !!! " << std::endl;
            if (changing_lane_complete_) {
                int new_lane_id = changeLaneIfPossible(car_x, 
                        car_y, 
                        car_s,
                        map_waypoints_x_, 
                        map_waypoints_y_,
                        sensor_fusion,
                        prev_path_x.size());

                if (new_lane_id >= 0 && target_speed_ > 45.0) {
                    std::cout << "APPLY CHANGING LANE" << std::endl;
                    this->lane_id_ = new_lane_id;
                    this->changing_lane_complete_ = false;
                }
                else {
                    target_speed_ *= 0.98;
                }
            }   
            else {
                target_speed_ *= 0.98;
            }
        } 
        else if (canSpeedUp(sensor_fusion, car_s, car_d, prev_path_x.size())) {
            std::cout << "Can Speed up now !!! " << std::endl;
            if (target_speed_ < 49.5) {
                // slowly increase speed 
                target_speed_ = (target_speed_ + 0.5) > 49.5 ? 49.5 : target_speed_+ 0.5;
            }
        }
    }
    addFixAnchorPoints(pts_x, pts_y, car_s);
	transformToCarRef(pts_x, pts_y, ref_yaw, ref_x, ref_y);
  	tk::spline spline_fit = initializeSpline(pts_x, pts_y);

	// push old prev points to new path to smooth the trajectory
	for (int i = 0; i < prev_path_x.size(); i++) {
		next_x_vals.push_back(prev_path_x[i]);
		next_y_vals.push_back(prev_path_y[i]);
	}

	// slow down to avoid collision 
std::cout << "Target Speed:  " << target_speed_ << " m/s" <<  std::endl;
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
	//print_double(next_x_vals);
	//print_double(next_y_vals);
}
