#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <time.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

// Helpers
#define KMpH2MpS 0.277777
#define Mps2KMpH 3.6
#define SIGN(x) ((x >= 0) ? 1 : -1)
#define DISTANCE(x1, y1, x2, y2) sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
#define _RAD2DEG 180 / M_PI
#define _DEG2RAD M_PI / 180

// Tracker Module // 1: Pure-pursuit, 2: Stanley, 3: Kanayama
#define LOCAL_TRACKER 3

// Vehicle Parameters
#define TURN_RADIUS 6.0
#define CURVATURE_MAX 1.0 / TURN_RADIUS
#define WHEEL_BASE 2.8325
#define MIN_VEL_INPUT 1.0
#define MAX_VEL_INPUT 6.0
#define LimitSteering 540
#define SteerRatio 15.43
#define LimitDegPerSec 360.0
#define LimitVelPerSec 5.0

ros::Subscriber Sub_localization, Sub_refPath;
ros::Publisher Pub_MarkerCar, Pub_poseVehicle, Pub_ControlCmd, Pub_vehicleTraj, Pub_finish, Pub_Point;
visualization_msgs::Marker m_CarPos, m_CarPosLine;
visualization_msgs::Marker m_point_marker;
bool m_pathFlag{false};
bool m_finishFlag{false};

struct CARPOSE
{
    double x, y, th, vel;
};
CARPOSE m_car;
double m_dir_mode = 1.0;
double m_Steer_cmd = 0.0;
double m_Velocity_cmd = 0.0;
ros::Time ros_time;
ros::Time ros_time2;
ros::Time start_ros_time;

bool is_init_path = true;
vector<Vector3d> m_ref_path;
int m_carIdx = 0;
int m_remainIdx = 0;
double m_curvature_max = 1.0 / TURN_RADIUS;

Vector3d m_init_pose = Vector3d(0.0, 0.0, 0.0);
Vector3d m_goal_pose = Vector3d(0.0, 0.0, 0.0);
bool almost_done_flag = false;
bool m_finish_flag = false;

// Kwan add
int Idx_old_ = 0;

double K_y_temp = 0.0;
double K_th_temp = 0.0;
double FF_gain_temp = 0.0;

void Local2Global(double Lx, double Ly, double &Gx, double &Gy)
{
    double tmpX = Lx;
    double tmpY = Ly;
    Gx = m_car.x + (tmpX * cos(m_car.th) - tmpY * sin(m_car.th));
    Gy = m_car.y + (tmpX * sin(m_car.th) + tmpY * cos(m_car.th));
}

//% target_{x,y}: position with respect to the global frame
//% coord_{x,y,th}: pose with respect to the global frame
//% rel_{x,y}: relative position of target with respect to coord_{x,y,th}
void GetRelativePosition(double target_x, double target_y, double target_th,
                         double coord_x, double coord_y, double coord_th,
                         double &rel_x, double &rel_y, double &rel_th)
{
    double rel_position_x = target_x - coord_x;
    double rel_position_y = target_y - coord_y;
    double D = sqrt(pow(rel_position_x, 2) + pow(rel_position_y, 2));
    double alpha = atan2(rel_position_y, rel_position_x);
    rel_x = D * cos(alpha - coord_th);
    rel_y = D * sin(alpha - coord_th);
    rel_th = atan2(sin(target_th - coord_th), cos(target_th - coord_th));
}

double three_pt_curvature(double x1, double y1, double x2, double y2, double x3, double y3)
{
    double fAreaOfTriangle = fabs((x1 * (y2 - y3) +
                                   x2 * (y3 - y1) +
                                   x3 * (y1 - y2)) /
                                  2);
    double fDist12 = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double fDist23 = sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3));
    double fDist13 = sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    double fKappa = 4 * fAreaOfTriangle / (fDist12 * fDist23 * fDist13);

    // cross product
    double first_vec_x = x2 - x1;
    double first_vec_y = y2 - y1;
    double second_vec_x = x3 - x2;
    double second_vec_y = y3 - y2;
    double cross_product = first_vec_x * second_vec_y - second_vec_x * first_vec_y;
    int sign_ = (cross_product >= 0) ? 1 : -1;

    if (isnan(fKappa) != 0)
        fKappa = 0.0;

    return sign_ * fKappa;
}

double CalculateCurvature(int idx)
{
    int path_size = m_ref_path.size();
    if (idx != 0 && idx <= path_size - 2)
        return three_pt_curvature(m_ref_path[idx - 1][0], m_ref_path[idx - 1][1],
                                  m_ref_path[idx][0], m_ref_path[idx][1],
                                  m_ref_path[idx + 1][0], m_ref_path[idx + 1][1]);
    else
        return m_curvature_max;
}

double m_dist_tol = 2.0;
double parkingVeloController(double velo_input)
{
    if (velo_input == 0.0)
    {
        return 0.0;
    }
    else
    {
        double ClosestPtX = m_ref_path[m_carIdx][0];
        double ClosestPtY = m_ref_path[m_carIdx][1];
        double dist = 0.0;
        dist = DISTANCE(ClosestPtX, ClosestPtY, m_ref_path.back()[0], m_ref_path.back()[1]);
        if (dist < m_dist_tol)                       //[m]
            return (dist / m_dist_tol) * velo_input; //
        // return MIN_VEL_INPUT;//
        else
            return velo_input;
    }
}

void PublishCarPose(double car_x, double car_y, double car_th)
{
    // Car Red (Car)
    m_CarPos.header.stamp = ros::Time::now();
    m_CarPos.pose.position.x = car_x;
    m_CarPos.pose.position.y = car_y;

    // Create the vehicle's AXIS
    geometry_msgs::PoseStamped poseStamped;
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "map";
    poseStamped.pose.position = m_CarPos.pose.position;
    poseStamped.pose.position.z = 5.5;
    poseStamped.pose.orientation = tf::createQuaternionMsgFromYaw(car_th);
    poseStamped.header = header;
    if (Pub_poseVehicle.getNumSubscribers() > 0)
        Pub_poseVehicle.publish(poseStamped);
}

void PublishPoint(double x, double y)
{
    m_point_marker.pose.position.x = x;
    m_point_marker.pose.position.y = y;
    m_point_marker.pose.position.z = 2.0;
    Pub_Point.publish(m_point_marker);
}

/* Pure-pursuit algorithm
** Input: Look-ahead distance, Velocity
** Output: steering angle and velocity for tracking the path
** Variables: (All information is with respect to the global frame (Global Axes of RVIZ))
LookAheadPtX, LookAheadPtY: LookAheadPoint with respect to the global frame. These are local variables.
m_ref_path: A global path which the vehicle should track. Its structure is [(x0,y0,index(=0)), (x1,y1,1), (x2,y2,2), ..., (x_{goal},y_{goal},N)] (global variables).
m_carIdx: The index of the closest waypoint on the global path (m_ref_path) from the vehicle (global variables).
m_car.x, m_car.y, m_car.th: The pose (position [m] + orientation [rad]) of the vehicle's center of the rear axle (global variables).
*/
pair<double, double> PurePursuit(double look_ahead_dist, double constant_velo_)
{
    double LookAheadPtX, LookAheadPtY;
    double steerAngle{0.0};  // [deg]
    double velo_result{0.0}; // [km/h]

    /* ========================================
    TODO: Code the pure-pursuit steering controller
    =========================================== */

    // (1)
    // Index of Look-ahead Point
    int Idx_ = 0;
    double theta = 0;
    double minDist = 99999, dist = 0;

    for (int i = Idx_old_; i < m_ref_path.size() - 1; i++)
    {
        dist = DISTANCE(m_ref_path[i][0], m_ref_path[i][1],
                        m_ref_path[m_carIdx][0], m_ref_path[m_carIdx][1]);
        if (dist > look_ahead_dist)
        {
            if (dist < minDist) // It may have two values.
            {
                minDist = dist;
                Idx_ = i;
            }
        }
    }

    Idx_old_ = Idx_;

    LookAheadPtX = m_ref_path[Idx_][0];
    LookAheadPtY = m_ref_path[Idx_][1];

    // (2)
    // Calculate alpha

    double rel_x = 0.0;
    double rel_y = 0.0;
    double rel_th = 0.0;
    GetRelativePosition(LookAheadPtX, LookAheadPtY, atan2(LookAheadPtY - m_car.y, LookAheadPtX - m_car.x),
                        m_car.x, m_car.y, m_car.th,
                        rel_x, rel_y, rel_th);
    double alpha = atan2(sin(rel_th), cos(rel_th));

    // (3)
    // Calculate R
    double L = WHEEL_BASE;
    double R = abs(look_ahead_dist / (2.0 * sin(alpha)));

    // (4)
    // Calculate Steering angle
    steerAngle = -SteerRatio * atan2(2.0 * L * sin(alpha) / look_ahead_dist, 1.0) * _RAD2DEG;
    velo_result = constant_velo_;

    if (abs(steerAngle) > LimitSteering)
        steerAngle = SIGN(steerAngle) * LimitSteering;
    velo_result = constant_velo_;
    velo_result = max(velo_result, MIN_VEL_INPUT);
    velo_result = min(velo_result, MAX_VEL_INPUT);
    PublishCarPose(m_car.x, m_car.y, m_car.th);
    PublishPoint(LookAheadPtX, LookAheadPtY);
    return std::make_pair(steerAngle, velo_result);
}

/* Stanley algorithm
** Input: Gain, Velocity, Cross-track err.
** Output: steering angle and velocity for tracking the path
** Variables: (All information is with respect to the global frame (Global Axes of RVIZ))
ClosestFrontPtX, ClosestFrontPtY: The closest point from the vehicle's center of the front wheel w.r.t the global frame. These are local variables.
m_ref_path: A global path which the vehicle should track. Its structure is [(x0,y0,index(=0)), (x1,y1,1), (x2,y2,2), ..., (x_{goal},y_{goal},N)] (global variables).
m_carIdx: The index of the closest waypoint on the global path (m_ref_path) from the vehicle (global variables).
m_car.x, m_car.y, m_car.th: The pose (position [m] + orientation [rad]) of the vehicle's center of the rear axle (global variables).
PublishPoint function: Visualize the closest point on Rviz.
*/
pair<double, double> Stanley(double gainK, double constant_velo_, double cross_track_err)
{
    double ClosestFrontPtX, ClosestFrontPtY;
    double steerAngle{0.0};  // [deg]
    double velo_result{0.0}; // [km/h]

    /* ========================================
    TODO: Code the stanley steering controller
    =========================================== */

    // (1)
    // Get the closest point from the vehicle's center of the front wheel
    ClosestFrontPtX = m_ref_path[m_carIdx][0];
    ClosestFrontPtY = m_ref_path[m_carIdx][1];

    double ClosestFrontPtX_prev = m_ref_path[m_carIdx - 1][0];
    double ClosestFrontPtY_prev = m_ref_path[m_carIdx - 1][1];

    // (2)
    // Calculate theta
    double front_wheel_x = m_car.x + WHEEL_BASE * cos(m_car.th);
    double front_wheel_y = m_car.y + WHEEL_BASE * sin(m_car.th);

    double rel_x = 0.0;
    double rel_y = 0.0;
    double rel_th = 0.0;
    GetRelativePosition(ClosestFrontPtX, ClosestFrontPtY, atan2(ClosestFrontPtY - ClosestFrontPtY_prev, ClosestFrontPtX - ClosestFrontPtX_prev),
                        front_wheel_x, front_wheel_y, m_car.th,
                        rel_x, rel_y, rel_th);
    double theta_e = atan2(sin(rel_th), cos(rel_th));

    cross_track_err = SIGN(rel_y) * cross_track_err;
    double theta_d = atan2(gainK * cross_track_err / (constant_velo_*KMpH2MpS), 1.0);
    theta_d = atan2(sin(theta_d), cos(theta_d));

    steerAngle = theta_e + theta_d;
    steerAngle = -steerAngle * SteerRatio * _RAD2DEG;

    if (abs(steerAngle) > LimitSteering)
        steerAngle = SIGN(steerAngle) * LimitSteering;
    velo_result = constant_velo_;
    velo_result = max(velo_result, MIN_VEL_INPUT);
    velo_result = min(velo_result, MAX_VEL_INPUT);
    PublishCarPose(m_car.x + WHEEL_BASE * cos(m_car.th), m_car.y + WHEEL_BASE * sin(m_car.th), m_car.th);
    PublishPoint(ClosestFrontPtX, ClosestFrontPtY);

    return std::make_pair(steerAngle, velo_result);
}

/* Kanayama algorithm
** Input: Lateral gain (K_y), Orientation gain(K_th), RefVelocity
** Output: steering angle and velocity for tracking the path
** Variables: (All information is with respect to the global frame (Global Axes of RVIZ))
ClosestPtX, ClosestPtY: The closest point from the vehicle's center of the rear wheel w.r.t the global frame. These are local variables.
RefPtX, RefPtY: The reference point in the Kanayama controller paper w.r.t the global frame. These are local variables.
                In this example, the reference point is determined as a fixed point 1 m away along the path from the closest point.
FF_term: Kanayama feed-forward term (angular velocity).
FF_gain: Gain for feed-forward term. As the reference path's curvature has some problems, so you need to adjust the FF_gain first before adjusting the feed-forward gain.
         In this step (the problem #1 for homework 3), set the feed-back gain (K_y, K_th) to zeros before adjusting the FF_gain.
FB_term: Kanayama feed-back term (angular velocity).
m_ref_path: A global path which the vehicle should track. Its structure is [(x0,y0,index(=0)), (x1,y1,1), (x2,y2,2), ..., (x_{goal},y_{goal},N)] (global variables).
m_carIdx: The index of the closest waypoint on the global path (m_ref_path) from the vehicle (global variables).
m_car.x, m_car.y, m_car.th: The pose (position [m] + orientation [rad]) of the vehicle's center of the rear axle (global variables).
PublishPoint function: Visualize the closest point on Rviz.
*/
pair<double, double> Kanayama(double K_y, double K_th, double ref_velo)
{
    double ClosestPtX, ClosestPtY;
    double RefPtX, RefPtY;
    double steerAngle{0.0};  // [deg]
    double velo_result{0.0}; // [km/h]

    ClosestPtX = m_ref_path[m_carIdx][0];
    ClosestPtY = m_ref_path[m_carIdx][1];

    // (1)
    // Index of Reference Point
    int Idx_ = 0;
    double minDist = 99999, dist = 0;

    for (int i = Idx_old_; i < m_ref_path.size() - 1; i++)
    {
        dist = DISTANCE(m_ref_path[i][0], m_ref_path[i][1],
                        m_ref_path[m_carIdx][0], m_ref_path[m_carIdx][1]);
        if (dist > 1.0)
        {
            if (dist < minDist)
            {
                minDist = dist;
                Idx_ = i;
            }
        }
    }

    RefPtX = m_ref_path[Idx_][0];
    RefPtY = m_ref_path[Idx_][1];

    double RefPtX_prev = m_ref_path[Idx_ - 1][0];
    double RefPtY_prev = m_ref_path[Idx_ - 1][1];

    double rel_x = 0.0;
    double rel_y = 0.0;
    double rel_th = 0.0;
    GetRelativePosition(RefPtX,  RefPtY,  atan2(RefPtY - RefPtY_prev, RefPtX - RefPtX_prev),
                        m_car.x, m_car.y, m_car.th,
                        rel_x,   rel_y,   rel_th);
    double theta_e = atan2(sin(rel_th), cos(rel_th));
    double y_e = rel_y;

    Idx_old_ = Idx_;

    /* ========================================
    TODO: Code the Kanayama steering controller */
    double FF_term = ref_velo * KMpH2MpS * CalculateCurvature(Idx_);
    double FB_term = ref_velo * KMpH2MpS * (K_y * y_e + K_th * sin(theta_e));
    /* YOU JUST NEED TO COMPUTE THE FF_term and FB_term above.
    =========================================== */

    // Lateral Control
    double FF_gain = FF_gain_temp;
    double kanayama_angular_velo = FF_gain * FF_term + FB_term;
    steerAngle = atan2(WHEEL_BASE * kanayama_angular_velo, ref_velo * KMpH2MpS);
    steerAngle = -(_RAD2DEG * SteerRatio) * atan2(sin(steerAngle), cos(steerAngle));
    if (abs(steerAngle) > LimitSteering)
        steerAngle = SIGN(steerAngle) * LimitSteering;

    velo_result = ref_velo;

    // Longitudinal Control
    // velo_result = (ref_velo * cos(heading_err)); // basic KANAYAMA // [km/h]
    // velo_result = (ref_velo * cos(heading_err) + K_x * x_e); // basic KANAYAMA
    // As the fixed ref_pose is used, neglect the 'K_x*x_e' term.

    /* ========== The problem #4 for homework 3. Longitudinal controller considering vehicle curvature errors ==========
    Kim, Minsoo, et al. "A Comparative Analysis of Path Planning and Tracking Performance According to the Consideration
    of Vehicle's Constraints in Automated Parking Situations." The Journal of Korea Robotics Society 16.3 (2021): 250-259. */
    // double K_v = 0.3;
    // double vehicle_curv = velo_result*KMpH2MpS* tan(m_Steer_cmd / (_RAD2DEG*SteerRatio)) / WHEEL_BASE;
    // velo_result = (velo_result != 0) ? velo_result * (1.0 - K_v * abs(CalculateCurvature(ref_idx) - vehicle_curv) / (2.0 * m_curvature_max)) : 0.0; // [m/s]
    // velo_result = parkingVeloController(velo_result);

    velo_result = max(velo_result, MIN_VEL_INPUT);
    velo_result = min(velo_result, MAX_VEL_INPUT);
    PublishCarPose(m_car.x, m_car.y, m_car.th);
    PublishPoint(RefPtX, RefPtY);

    return std::make_pair(steerAngle, velo_result);
}

// Assumes, forward motion only
void CallbackRefPath(const nav_msgs::Path::ConstPtr &msg)
{
    cout << "reference path callback!" << endl;
    if (is_init_path)
    {
        is_init_path = false; // MK add
        start_ros_time = ros::Time::now();
    }

    // MemberValInit();
    if (msg->poses.size() != 0)
    {
        // check forward/backward switching
        double first_th_, second_th_;
        int pathIdx = 0;

        m_init_pose = Vector3d(msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 0);
        double goal_th = atan2(msg->poses.back().pose.position.y - msg->poses[msg->poses.size() - 2].pose.position.y,
                               msg->poses.back().pose.position.x - msg->poses[msg->poses.size() - 2].pose.position.x);
        cout << "GOAL: " << msg->poses.back().pose.position.x << ", " << msg->poses.back().pose.position.y << endl;
        m_goal_pose = Vector3d(msg->poses.back().pose.position.x, msg->poses.back().pose.position.y, goal_th);

        // Substitute the first value
        m_ref_path.push_back(Vector3d(msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 0));
        cout << "Callback ReferencePath # Size (# of way-points) : " << msg->poses.size() << endl;
        // Each pathIdx is represented as the index of path segment, which is distinguished by Gear Switching.
        for (int i = 0; i < msg->poses.size(); i++)
            m_ref_path.push_back(Vector3d(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, i));
        m_pathFlag = true;
    }
}

void vehicleTrajectory(int time)
{ // Vehicle's future trajectory
    double dt = 0.1;
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.poses.resize((int)(time / dt));

    double yaw = 0.0, xL = 0.0, yL = 0.0;
    for (int i = 0; i < (int)(time / dt); i++)
    { // a unit of the time is second.
        xL += dt * m_dir_mode * m_car.vel * cos(yaw);
        yL += dt * m_dir_mode * m_car.vel * sin(yaw);
        yaw += dt * dt * m_dir_mode * m_car.vel * atan2(-m_Steer_cmd * _DEG2RAD, 1.0); /// 2.85;
        Local2Global(xL, yL, msg.poses[i].pose.position.x, msg.poses[i].pose.position.y);
        msg.poses[i].pose.position.z = 0.0;
    }
    Pub_vehicleTraj.publish(msg);
    msg.poses.clear();
}

// Publish control commands as a topic for controlling the vehicle
void VehicleControl(double steer, double velocity)
{
    // Physical limit
    double dt = (ros::Time::now() - ros_time2).toSec();
    if (LimitDegPerSec * dt < abs(steer - m_Steer_cmd))
        m_Steer_cmd += (double)(SIGN(steer - m_Steer_cmd) * LimitDegPerSec * dt);
    else
        m_Steer_cmd = steer;

    if (LimitVelPerSec * dt < abs(velocity - m_Velocity_cmd))
        m_Velocity_cmd += (double)(SIGN(velocity - m_Velocity_cmd) * LimitVelPerSec * dt);
    else
        m_Velocity_cmd = velocity;

    // For CARLA
    std_msgs::Float32MultiArray msg_;
    msg_.data.push_back(m_Steer_cmd);
    msg_.data.push_back(m_Velocity_cmd);
    Pub_ControlCmd.publish(msg_);
}

void PublishTopic()
{
    static int pub_cnt;
    pub_cnt++;
    if (pub_cnt == 7)
    {
        m_CarPosLine.points.push_back(m_CarPos.pose.position);
        pub_cnt = 0;
    }
    if (Pub_MarkerCar.getNumSubscribers() > 0)
        Pub_MarkerCar.publish(m_CarPosLine);
}

// Find the closest way-point from the vehicle
double CalculateClosestPt(int &pWIdx, double xx, double yy)
{
    double minDist = 99999, dist = 0;
    for (int i = 0; i < m_ref_path.size() - 1; i++)
    {
        dist = DISTANCE(m_ref_path[i][0], m_ref_path[i][1], xx, yy);
        if (dist < minDist)
        {
            minDist = dist;
            pWIdx = i;
        }
    }
    m_carIdx = pWIdx;

    return minDist;
}

// Check that the vehilce reaches the goal pose. If the distance between the vehicle's position and goal is lower than 'check_thres', then the vehicle reaches the goal.
void GoalCheck(double check_thres, int which_tracker)
{
    bool goal_checking_front = (which_tracker == 2) ? true : false;
    if (!goal_checking_front && almost_done_flag && (DISTANCE(m_car.x, m_car.y, m_goal_pose[0], m_goal_pose[1]) < check_thres))
    {
        std::cout << "Goal!" << std::endl;
        double tracking_time = (ros::Time::now() - start_ros_time).toSec();

        double rel_x, rel_y, rel_th;
        GetRelativePosition(m_car.x, m_car.y, m_car.th, m_goal_pose[0], m_goal_pose[1], m_goal_pose[2], rel_x, rel_y, rel_th);
        std::cout << "Tracking time [s]: " << tracking_time << std::endl;
        std::cout << "Lateral error [m]: " << abs(rel_y) << std::endl;
        std::cout << "Orientation error [deg]: " << _RAD2DEG * atan2(sin(rel_th), cos(rel_th)) << std::endl;

        // For CARLA
        std_msgs::Float32MultiArray msg_;
        msg_.data.push_back(0.0);
        msg_.data.push_back(0.0);
        Pub_ControlCmd.publish(msg_);
        m_finish_flag = true;
    }
    double car_front_x = m_car.x + WHEEL_BASE * cos(m_car.th);
    double car_front_y = m_car.y + WHEEL_BASE * sin(m_car.th);
    double goal_front_x = m_goal_pose[0] + WHEEL_BASE * cos(m_goal_pose[2]);
    double goal_front_y = m_goal_pose[1] + WHEEL_BASE * sin(m_goal_pose[2]);

    if (goal_checking_front && almost_done_flag && (DISTANCE(car_front_x, car_front_y, goal_front_x, goal_front_y) < check_thres))
    {
        std::cout << "Goal!" << std::endl;
        double tracking_time = (ros::Time::now() - start_ros_time).toSec();

        double rel_x, rel_y, rel_th;
        GetRelativePosition(car_front_x, car_front_y, m_car.th, goal_front_x, goal_front_y, m_goal_pose[2], rel_x, rel_y, rel_th);
        std::cout << "Tracking time [s]: " << tracking_time << std::endl;
        std::cout << "Lateral error [m]: " << abs(rel_y) << std::endl;
        std::cout << "Orientation error [deg]: " << _RAD2DEG * atan2(sin(rel_th), cos(rel_th)) << std::endl;

        // For CARLA
        std_msgs::Float32MultiArray msg_;
        msg_.data.push_back(0.0);
        msg_.data.push_back(0.0);
        Pub_ControlCmd.publish(msg_);
        m_finish_flag = true;
    }
    return;
}

// Check that how much distance to the goal is remained
void remain_path_check(double close_to_goal_thres)
{
    if (almost_done_flag)
        return;

    m_remainIdx = m_ref_path.size() - m_carIdx;
    if (close_to_goal_thres > (double)m_remainIdx * 0.01)
    {
        int path_size = m_ref_path.size();
        double first_th = atan2(m_ref_path[path_size - 2][1] - m_ref_path[path_size - 3][1], m_ref_path[path_size - 2][0] - m_ref_path[path_size - 3][0]);
        double second_th = atan2(m_ref_path[path_size - 1][1] - m_ref_path[path_size - 2][1], m_ref_path[path_size - 1][0] - m_ref_path[path_size - 2][0]);
        double th_diff = second_th - first_th;
        if (th_diff > M_PI)
            th_diff = th_diff - 2 * M_PI;
        else if (th_diff < -M_PI)
            th_diff = 2 * M_PI - abs(th_diff);
        double extend_distance = DISTANCE(m_ref_path[path_size - 2][0], m_ref_path[path_size - 2][1],
                                          m_ref_path[path_size - 1][0], m_ref_path[path_size - 1][1]);
        for (int j = 1; j <= 2000; j++) // 20 m extension
            m_ref_path.push_back(Vector3d(m_ref_path[path_size - 2 + j][0] + extend_distance * cos(second_th + j * th_diff),
                                          m_ref_path[path_size - 2 + j][1] + extend_distance * sin(second_th + j * th_diff),
                                          path_size));
        almost_done_flag = true;
        return;
    }
}

// Compute the control commands for tracking
void Compute()
{
    int pWIdx = 0;
    if (!m_finish_flag && m_ref_path.size() > 1)
    {
        pair<double, double> _control;
        switch (LOCAL_TRACKER)
        {
        case 1:
        {                                                // PurePursuit
            CalculateClosestPt(pWIdx, m_car.x, m_car.y); // Computing the closest point from the rear axle
            _control = PurePursuit(2.0, 3.0);
            break;
        }
        case 2:
        {                                                                                                                            // Stanley
            double min_dist = CalculateClosestPt(pWIdx, m_car.x + WHEEL_BASE * cos(m_car.th), m_car.y + WHEEL_BASE * sin(m_car.th)); // Computing the closest point from the front axle
            _control = Stanley(1.0, 3.0, min_dist);
            break;
        }
        case 3:                                          // Kanayama
            CalculateClosestPt(pWIdx, m_car.x, m_car.y); // Computing the closest point from the rear axle
            _control = Kanayama(K_y_temp, K_th_temp, 3.0);
            break;
        default:
        {
            _control = PurePursuit(0.0, 0);
            break;
        }
        }
        remain_path_check(5.0);
        VehicleControl(_control.first, m_dir_mode * _control.second); //[km/h]
        ros_time2 = ros::Time::now();
        GoalCheck(0.2, LOCAL_TRACKER); // When you use Stanley, set it to 'true', otherwise 'false'.
    }
    PublishTopic();
}

void CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    m_car.x = msg->data.at(0);              // x
    m_car.y = msg->data.at(1);              // y
    m_car.th = msg->data.at(2);             // theta
    m_car.vel = msg->data.at(3) * KMpH2MpS; // to [m/s]

    // Set a center to rear axis from CoM
    double c2r = -1.2865;
    m_car.x += c2r * cos(m_car.th);
    m_car.y += c2r * sin(m_car.th);

    ros_time = ros::Time::now();
    if (m_pathFlag && !m_finishFlag)
    { // PATH IS GENERATED, WHICH CONSISTS OF PATH SEGMENTS
        Compute();
    }
    else
    {
        VehicleControl(0.0, 0.0);
    }
    vehicleTrajectory(1.5); // based on the vehicle's kinematic model
}

void MarkerInit()
{
    m_CarPosLine.points.clear();
    m_CarPos.header.frame_id = m_CarPosLine.header.frame_id = "map";
    m_CarPos.ns = m_CarPosLine.ns = "RegionOfInterest";
    m_CarPos.id = 0;
    m_CarPosLine.id = 1;

    m_CarPos.type = visualization_msgs::Marker::ARROW;
    m_CarPosLine.type = visualization_msgs::Marker::LINE_STRIP;
    m_CarPos.action = m_CarPosLine.action = visualization_msgs::Marker::ADD;

    m_CarPos.scale.x = 4.5 / 2;
    m_CarPos.scale.y = 1.5;
    m_CarPos.scale.z = 0.1;
    m_CarPos.color.a = 0.7;
    m_CarPos.color.r = 1.0f;
    m_CarPos.color.g = 0.0f;
    m_CarPos.color.b = 0.0f;
    m_CarPos.pose.position.z = 0.5;

    m_CarPosLine.scale.x = 0.15;
    m_CarPosLine.pose.position.z = 1.2;
    m_CarPosLine.color.r = 1.0;
    m_CarPosLine.color.a = 0.7;

    m_point_marker.header.frame_id = "map";
    m_point_marker.header.stamp = ros::Time();
    m_point_marker.ns = "point";
    m_point_marker.id = 0;
    m_point_marker.type = visualization_msgs::Marker::SPHERE;
    m_point_marker.action = visualization_msgs::Marker::ADD;
    m_point_marker.pose.position.x = 1;
    m_point_marker.pose.position.y = 1;
    m_point_marker.pose.position.z = 1;
    m_point_marker.pose.orientation.x = 0.0;
    m_point_marker.pose.orientation.y = 0.0;
    m_point_marker.pose.orientation.z = 0.0;
    m_point_marker.pose.orientation.w = 1.0;
    m_point_marker.scale.x = 0.5;
    m_point_marker.scale.y = 0.5;
    m_point_marker.scale.z = 0.0;
    m_point_marker.color.a = 1.0;
    m_point_marker.color.r = 1.0;
    m_point_marker.color.g = 0.0;
    m_point_marker.color.b = 0.0;
}

void LOCALPLANNERTHREAD()
{
    int argc = 0;
    char **argv;
    ros::init(argc, argv, "Tracker");
    ros::NodeHandle nh_, anytime_node;
    ros::NodeHandle priv_nh("~");
    MarkerInit();

    ros::CallbackQueue anytime_queue;
    anytime_node.setCallbackQueue(&anytime_queue);

    Pub_MarkerCar = nh_.advertise<visualization_msgs::Marker>("markerVehicleTraj", 1);
    Pub_poseVehicle = nh_.advertise<geometry_msgs::PoseStamped>("poseVehicle", 1);
    Pub_vehicleTraj = nh_.advertise<nav_msgs::Path>("vehicle_traj", 1);
    Pub_ControlCmd = nh_.advertise<std_msgs::Float32MultiArray>("Control_Command", 1);
    Pub_finish = nh_.advertise<std_msgs::Float32MultiArray>("FinishFlag", 1);
    Pub_Point = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 0);

    Sub_localization = nh_.subscribe("/LocalizationData", 10, &CallbackLocalizationData);
    Sub_refPath = nh_.subscribe("ref_path", 1, &CallbackRefPath); // From doRRT node

    nh_.getParam("K_y"    , K_y_temp);
    nh_.getParam("K_th"   , K_th_temp);
    nh_.getParam("FF_gain", FF_gain_temp);

    cout << "K_y: " << K_y_temp <<  endl;
    cout << "K_th: " << K_th_temp <<  endl;
    cout << "FF_gain: " << FF_gain_temp << endl;

    // MemberValInit();
    cout << "START CONTROLLER !!!" << endl;


    ros::AsyncSpinner spinner(0, &anytime_queue);
    spinner.start();
    ros::spin();
}

int main(int argc, char *argv[])
{
    LOCALPLANNERTHREAD();
    return 0;
}