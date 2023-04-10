#ifndef _HEIGHT_MAP_H_
#define _HEIGHT_MAP_H_


#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <string>
#include <nav_msgs/Path.h>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

namespace velodyne_height_map {

// shorter names for point cloud types in this namespace
typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

typedef pcl::PointXYZRGB VIMage;
typedef pcl::PointCloud<VIMage> VPointImage;

class HeightMap{
public:

    /** Constructor
   *
   *  @param node NodeHandle of this instance
   *  @param private_nh private NodeHandle of this instance
   */
    HeightMap(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~HeightMap();

    /** callback to process data input
   *
   *  @param scan vector of input 3D data points
   *  @param stamp time stamp of data
   *  @param frame_id data frame of reference
   */
    void processData(const VPointCloud::ConstPtr &scan);

private:
    struct CARPOSE
    {
        double x;
        double y;
        double th;
        double vel;
    };
    CARPOSE m_car, m_carObs;

    void constructGridClouds(const VPointCloud::ConstPtr &scan, size_t &data_count_vel);
    void constructGridClouds_forSemantic(const VPointCloud::ConstPtr &scan, size_t &data_count_vel);
    void constructGridCam(const sensor_msgs::ImageConstPtr& scan, size_t &data_count_cam);
    void constructGridDist(const sensor_msgs::ImageConstPtr& scan, size_t &data_count_cam);
    

    void CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void callBackParkingGoal(const geometry_msgs::PoseStamped::ConstPtr& end);
    void callBackImage(const sensor_msgs::ImageConstPtr& scan);
    void callBackImageRaw(const sensor_msgs::ImageConstPtr& scan);
    void callBackImageDist(const sensor_msgs::ImageConstPtr& scan);
	void CallbackPath(const nav_msgs::Path::ConstPtr& msg);

    void real2arr(double recvX, double recvY, int& outX, int& outY);
    void arr2real(int recvX, int recvY, double& outX, double& outY);
    void Local2Global(double Lx, double Ly, double &Gx, double &Gy);
    void Global2Local(double Gx, double Gy, double &Lx, double &Ly);
    void Rotate(double inX, double inY, double th, double X, double Y, double &Gx, double &Gy) ;

    double m_gridResol;
    int m_gridDim;
    double m_x;
    double m_y;
    double m_theta;
    double m_vel;

    double cam_perspective_angle;
    double cam2groundLen;
    double x_cali;
    double y_cali;

    // double m_gX[4], m_gY[4];
    double m_goalX, m_goalY, m_goalTh;
    bool m_parkingFlag;

    std::string m_NaviMsg;

    VPointCloud m_obstaclePtCloudLocal;
    VPointImage m_cam_data;
    nav_msgs::OccupancyGrid occupancyGridFront;
    nav_msgs::OccupancyGrid occupancyGrid;
    nav_msgs::OccupancyGrid distanceGrid;

    // ROS topics
    ros::Subscriber Sub_velodyne;
    ros::Subscriber Sub_localization;
    ros::Subscriber Sub_parking;
    ros::Subscriber Sub_cam, Sub_cam_raw, Sub_dist;
    ros::Subscriber Sub_Path_backward;
    
    ros::Publisher Pub_obstacleLocal;
    ros::Publisher Pub_occupancygrid, Pub_distancegrid;
    ros::Publisher Pub_front_array;
    ros::Publisher Pub_potential_array;
    ros::Publisher Pub_cam, Pub_dist;
    
};

} // namespace velodyne_height_map

#endif
