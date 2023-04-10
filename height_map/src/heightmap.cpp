#include <velodyne_height_map/heightmap.h>
// #include <MathParam.h>
// #include <GeometricUtils.h>
// #include <AngleUtils.h>

using namespace std;
using namespace cv;

namespace velodyne_height_map {

#define DISTANCE(x1, y1, x2, y2) sqrt((x1 - x2)*(x1 - x2) +(y1 - y2)*(y1 - y2))
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

#define heightDiffThreshold 0.2
#define GroundZ -1.5
#define Occluded true

#define DIMENSION 32//Anytime: 32 //19//20//33    //[m] 30

#define LINE 0.3        //[m]
//#define PARKING_H (5.1 + 4*LINE)    //[m]
#define PARKING_H 5.1    //[m]
#define PARKING_W (2.3 + 2*LINE)    //[m]

void HeightMap::CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg){
    m_car.x = msg->data.at(0); // the center of the vehicle
    m_car.y = msg->data.at(1);
    m_car.th = msg->data.at(2);
    m_car.vel = msg->data.at(3);
}

// void HeightMap::callBackParkingFlag(const geometry_msgs::PoseStamped::ConstPtr& parking) {
    // m_parkingFlag = true;

    // m_gridResol = 0.3;
    // m_gridDim = (int)(DIMENSION*(int)(1/m_gridResol));

    // m_goalX = parking->pose.position.x;
    // m_goalY = parking->pose.position.y;
    // double goalTh = tf::getYaw(parking->pose.orientation);;//end->pose.pose.orientation;

    // Rotate(goalX, goalY, goalTh, PARKING_H/2, PARKING_W/2, m_gX[0], m_gY[0]);
    // Rotate(goalX, goalY, goalTh, PARKING_H/2, -PARKING_W/2, m_gX[1], m_gY[1]);
    // Rotate(goalX, goalY, goalTh, -PARKING_H/2, -PARKING_W/2, m_gX[2], m_gY[2]);
    // Rotate(goalX, goalY, goalTh, -PARKING_H/2, PARKING_W/2, m_gX[3], m_gY[3]);
// }

HeightMap::HeightMap(ros::NodeHandle node, ros::NodeHandle priv_nh){

    // Pub_obstacleLocal = node.advertise<VPointCloud>("velodyne_obs",1);
    Pub_occupancygrid = node.advertise<nav_msgs::OccupancyGrid>("occ_map", 10);
    Pub_distancegrid = node.advertise<nav_msgs::OccupancyGrid>("dist_map", 10);
    Pub_front_array = node.advertise<nav_msgs::OccupancyGrid>("vel_front", 1);
    Pub_potential_array = node.advertise<std_msgs::Float32MultiArray>("velodyne_potential_array", 1);
    Pub_cam = node.advertise<VPointImage>("grid_data_cam", 1);
    Pub_dist = node.advertise<VPointImage>("/traj_distribution_data", 1);

    Sub_velodyne = node.subscribe("velodyne_points", 100,
                                    &HeightMap::processData, this,
                                    ros::TransportHints().tcpNoDelay(true));
    Sub_cam = node.subscribe("avm_usb_cam/image_box", 5,
                             &HeightMap::callBackImage, this,
                             ros::TransportHints().tcpNoDelay(true));

    Sub_dist = node.subscribe("/raw_dist_img", 5,
                             &HeightMap::callBackImageDist, this,
                             ros::TransportHints().tcpNoDelay(true));

    Sub_cam_raw = node.subscribe("avm_usb_cam/image_raw", 5,
                             &HeightMap::callBackImageRaw, this,
                             ros::TransportHints().tcpNoDelay(true));

    Sub_localization = node.subscribe("LocalizationData", 1, &HeightMap::CallbackLocalizationData, this);
    Sub_Path_backward = node.subscribe("parkingPathBackward", 1, &HeightMap::CallbackPath, this);
    // Sub_parking = node.subscribe("PoseGoal", 1, &HeightMap::callBackParkingGoal, this);

    m_gridResol = 0.2;//0.2;; 0.19;//1.0/5.15; (meter per cell)
    m_gridDim = (int)(DIMENSION*(int)(1.0 / m_gridResol));  ///(int)(50*(int)(1/m_per_cell_)) // 나누어 떨어진다는 가정.
    
    std::cout << "Occmap Info." << std::endl;
    std::cout << "grid dim: " << m_gridDim << std::endl;
    std::cout << "grid resol: " << m_gridResol << std::endl;

    cam_perspective_angle = 60.0*M_PI/180.0;   //[deg->rad]
    x_cali = 80.0;
    y_cali = 80.0;//79.0;
    double scale = 45.0;
    cam2groundLen = scale / tan(cam_perspective_angle);
 
    m_parkingFlag = false;

    occupancyGrid.header.frame_id = "map";
    occupancyGrid.info.resolution = m_gridResol;
    occupancyGrid.info.width = m_gridDim;   // 96
    occupancyGrid.info.height = m_gridDim;  // 96
    // occupancyGrid.info.origin.position.x = - DIMENSION / 2.0;
    // occupancyGrid.info.origin.position.y = - DIMENSION / 2.0;
    // occupancyGrid.info.origin.position.z = -0.1;//GroundZ;
    occupancyGrid.data.resize(occupancyGrid.info.width*occupancyGrid.info.height);

    // Distance Grid
    distanceGrid.header.frame_id = "map";
    distanceGrid.info.resolution = m_gridResol;
    distanceGrid.info.width =  m_gridDim;   // 96
    distanceGrid.info.height = m_gridDim;  // 96
    distanceGrid.info.origin.position.x = -DIMENSION / 2;//+DIMENSION / 2 + m_gridResol * 2;
    distanceGrid.info.origin.position.y = -DIMENSION / 2;
    distanceGrid.info.origin.position.z = -0.1;//GroundZ0;
    distanceGrid.data.resize(distanceGrid.info.width * distanceGrid.info.height);

    m_car.x = m_car.y = m_car.th = 0.0;
}

HeightMap::~HeightMap() {}

void HeightMap::constructGridClouds(const VPointCloud::ConstPtr &scan, size_t &data_count_vel)
{   
    // Initialize
    float min[m_gridDim][m_gridDim], max[m_gridDim][m_gridDim], num_obs[m_gridDim][m_gridDim]; // MK: for constructing the occupancy grid
    bool init[m_gridDim][m_gridDim], obs_z[m_gridDim][m_gridDim], unknown[m_gridDim][m_gridDim];
    for (int x = 0; x < m_gridDim; x++)
        for (int y = 0; y < m_gridDim; y++) {
            min[x][y] = 0.0;
            max[x][y] = 0.0;
            num_obs[x][y] = 0.0;
            init[x][y] = false;
            obs_z[x][y] = false;
            unknown[x][y] = false;
        }
    
    double rX, rY, Gx, Gy;
    int x, y;
    const double donutRange = 1.5, donutMinHeight = -0.3; // [m] // donutRange is for the velodyne grandeur
    for (int i = 0; i < scan->points.size(); i++) // for each points
    {
        double dist = DISTANCE(0, 0, scan->points[i].x, scan->points[i].y);
        if (dist > donutRange && scan->points[i].z < donutMinHeight) {    //to erase the donut (for real-vehicle, grandeur)
            real2arr(scan->points[i].x, scan->points[i].y, x, y); // it converts the point with respect to the grid-frame
            if (x >= 0 && x < m_gridDim && y >= 0 && y < m_gridDim) {
                if (!init[x][y]) {
                    min[x][y] = scan->points[i].z;
                    max[x][y] = scan->points[i].z;
                    init[x][y] = true;
                } else {
                    min[x][y] = MIN(min[x][y], scan->points[i].z);
                    max[x][y] = MAX(max[x][y], scan->points[i].z);
                }
                if (scan->points[i].z > GroundZ) // filter the points, which is above the ground
                    obs_z[x][y] = true;
            }
        }
    }

    int x_ = 0, y_ = 0;
    for (int x = 0; x < m_gridDim; x++) {
        for (int y = 0; y < m_gridDim; y++) {
            arr2real(x, y, rX, rY); // get real X and real Y (X, Y)
            if ((obs_z[x][y]) || (max[x][y] - min[x][y] > heightDiffThreshold)) {   // height filter
                num_obs[x][y]++;    // occupied
                double th = atan2((rY), (rX));
                while (Occluded) {  // consider the obstacle's back to occupied
                    double L = 0.19;//1*m_gridResol / cos(th);
                    rX += L*cos(th);
                    rY += L*sin(th);
                    real2arr(rX, rY, x_, y_);
                    if (x_ > 0 && y_ > 0 && x_ < m_gridDim - 1 && y_ < m_gridDim - 1)
                        unknown[x_][y_] = true;
                    else
                        break;
                }
            }
        }
    }
    
    
    int cnt = 0; // occupancy Grid는 맨 밑에 row부터 오른쪽으로 데이터 index가 진행됨.
    for (int x = 0; x < m_gridDim; x++) 
        for (int y = 0; y < m_gridDim; y++) {
            if (num_obs[y][x] > 0) {
                occupancyGrid.data[cnt] = 100;    //obs
            }
            else if (unknown[y][x] == true) {
                occupancyGrid.data[cnt] = -1;    //obs
            }
            else
                occupancyGrid.data[cnt] = 0;    //free
            cnt++;
        }

    if( Pub_occupancygrid.getNumSubscribers() > 0 )
        Pub_occupancygrid.publish(occupancyGrid);
}

// semantic label table
// 0: unknown object(?)
// 0.14013 
// 0.840779: line
// 0.980909: road
// 1.26117: tree
// 1.4013: vehicle
// 3.22299: curb
// 3.36312: bumper 
void HeightMap::constructGridClouds_forSemantic(const VPointCloud::ConstPtr &scan, size_t &data_count_vel)
{   
    // Initialize
    float min[m_gridDim][m_gridDim], max[m_gridDim][m_gridDim]; // MK: for constructing the occupancy grid
    bool init[m_gridDim][m_gridDim], obs_z[m_gridDim][m_gridDim], unknown[m_gridDim][m_gridDim], occupied[m_gridDim][m_gridDim], free[m_gridDim][m_gridDim];
    bool line[m_gridDim][m_gridDim], drivable[m_gridDim][m_gridDim], vehicle[m_gridDim][m_gridDim], object[m_gridDim][m_gridDim], curb[m_gridDim][m_gridDim], bumper[m_gridDim][m_gridDim];
    for (int x = 0; x < m_gridDim; x++)
        for (int y = 0; y < m_gridDim; y++) {
            min[x][y] = 0.0;
            max[x][y] = 0.0;
            occupied[x][y] = false;
            free[x][y] = false;
            init[x][y] = false;
            obs_z[x][y] = false;
            unknown[x][y] = false;
            line[x][y] = false;
            drivable[x][y] = false;
            vehicle[x][y] = false;
            object[x][y] = false;
            curb[x][y] = false;
            bumper[x][y] = false;
        }
    
    // set<double> sss;

    double rX, rY, Gx, Gy;
    int x, y;
    const double donutRange = 1.5, donutMinHeight = -0.3; // [m] // donutRange is for the velodyne grandeur
    for (int i = 0; i < scan->points.size(); i++) // for each points
    {
        double dist = DISTANCE(0, 0, scan->points[i].x, scan->points[i].y);
        if (dist > donutRange && scan->points[i].z < donutMinHeight) {    //to erase the donut (for real-vehicle, grandeur)
            real2arr(scan->points[i].x, scan->points[i].y, x, y); // it converts the point with respect to the grid-frame
            if (x >= 0 && x < m_gridDim && y >= 0 && y < m_gridDim) {
                if (!init[x][y]) {
                    min[x][y] = scan->points[i].z;
                    max[x][y] = scan->points[i].z;
                    init[x][y] = true;
                } else {
                    min[x][y] = MIN(min[x][y], scan->points[i].z);
                    max[x][y] = MAX(max[x][y], scan->points[i].z);
                }
                if (scan->points[i].z > GroundZ) // filter the points, which is above the ground
                    obs_z[x][y] = true;

                // sss.insert(scan->points[i].intensity * 1e+44);

                if (fabs(scan->points[i].intensity * 1e+44 - 1.4013) <= 1e-4)
                    vehicle[x][y] = true;
                else if (fabs(scan->points[i].intensity * 1e+44 - 0.980909) <= 1e-4)
                    drivable[x][y] = true;
                else if (fabs(scan->points[i].intensity * 1e+44 - 0.840779) <= 1e-4)
                    line[x][y] = true;
                else if (fabs(scan->points[i].intensity * 1e+44 - 0.0) <= 1e-4)
                    object[x][y] = true;
                else if (fabs(scan->points[i].intensity * 1e+44 - 3.22299) <= 1e-4)
                    curb[x][y] = true;
                else if (fabs(scan->points[i].intensity * 1e+44 - 3.36312) <= 1e-4)
                    bumper[x][y] = true;
            }
        }
    }

    bool definitely_obs, definitely_free;
    int x_ = 0, y_ = 0;
    for (int x = 0; x < m_gridDim; x++) {
        for (int y = 0; y < m_gridDim; y++) {
            arr2real(x, y, rX, rY); // get real X and real Y (X, Y)
            if ((vehicle[x][y] || object[x][y] || curb[x][y]) || obs_z[x][y] || (max[x][y] - min[x][y] > heightDiffThreshold)) {   // height filter
                occupied[x][y] = true;    // definitely occupied
                double th = atan2((rY), (rX));
                while (Occluded) {  // consider the obstacle's back to occupied
                    double L = 0.20;//1*m_gridResol / cos(th);
                    rX += L*cos(th);
                    rY += L*sin(th);
                    real2arr(rX, rY, x_, y_);
                    if (x_ > 0 && y_ > 0 && x_ < m_gridDim - 1 && y_ < m_gridDim - 1)
                        unknown[x_][y_] = true; //occluded
                    else
                        break;
                }
            }
            else if (drivable[x][y] || line[x][y] || bumper[x][y]) // definitely free
                free[x][y] = true;
        }
    }


    int cnt = 0; // occupancy Grid는 맨 밑에 row부터 오른쪽으로 데이터 index가 진행됨.
    cv::Mat src(m_gridDim, m_gridDim, CV_8UC1, Scalar(255)); // add for a distance map
    for (int x = 0; x < m_gridDim; x++) 
        for (int y = 0; y < m_gridDim; y++) {
            if (occupied[y][x]) {//definetly occupied
                occupancyGrid.data[cnt] = 100;    //obs
                src.at<uchar>(y, x) = 0;
            }
            else if (!free[y][x] && unknown[y][x])  // not definetly occupied or free, and it is categorized as occluded
                occupancyGrid.data[cnt] = -1;    //unknown
            else
                occupancyGrid.data[cnt] = 0;    //obs

            cnt++;
        }

    occupancyGrid.info.origin.position.x = m_car.x + (DIMENSION / 2.0) * (cos(m_car.th - M_PI / 2.0) + cos(m_car.th + M_PI));
    occupancyGrid.info.origin.position.y = m_car.y + (DIMENSION / 2.0) * (sin(m_car.th - M_PI / 2.0) + sin(m_car.th + M_PI));
    occupancyGrid.info.origin.position.z = -0.1;//GroundZ;
    occupancyGrid.info.origin.orientation = tf::createQuaternionMsgFromYaw(m_car.th);

    cv::Mat dist  = cv::Mat::zeros(m_gridDim, m_gridDim, CV_32FC1); // add for a distance map
    cv::distanceTransform(src, dist, cv::DIST_L2, 5); // DIST_L2:Euclidean Distance
    cv::normalize(dist, dist, 0.0, 100.0, NORM_MINMAX); // Normalize in a range of [0.0, 100.0]
    cnt = 0;
    for (int x = 0; x < m_gridDim; x++) 
        for (int y = 0; y < m_gridDim; y++) {
            distanceGrid.data[cnt++] =  100.0 - dist.at<float>(y, x);    //obs: '0 ~ 100'// 0.0 is entirely free, 100.0 is obs
        } // occupancy_grid_map's index starts from the bottom-left of the map


    if( Pub_occupancygrid.getNumSubscribers() > 0 )
        Pub_occupancygrid.publish(occupancyGrid);
    if( Pub_distancegrid.getNumSubscribers() > 0 )
        Pub_distancegrid.publish(distanceGrid);
}


void HeightMap::constructGridDist(const sensor_msgs::ImageConstPtr& scan, size_t &data_count_cam)
{
    // for debugging mk --------------------------------
    cv::Mat cvImg = cv_bridge::toCvShare(scan, "rgb8")->image;
    cv::rotate(cvImg, cvImg, cv::ROTATE_90_CLOCKWISE);
    cv::rotate(cvImg, cvImg, cv::ROTATE_90_CLOCKWISE);
    cv_bridge::CvImage out_msg;
    // out_msg.header   = header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    out_msg.image    = cvImg; // Your cv::Mat
    sensor_msgs::ImageConstPtr scan2 = out_msg.toImageMsg();
    for (size_t i = 0; i < scan2->data.size(); i += 3) // [#1(b,g,r), #2(b,g,r), ...]
    {
        double pixX = i/3 / scan->width;
        double pixY = i/3 - pixX * scan->width;
        
        double x_real, y_real;
        x_real = (scan->height/2.0 - pixY)*m_gridResol - m_gridResol / 2.0;
        y_real = (pixX - scan->width/2.0)*m_gridResol + m_gridResol / 2.0;
        
        double gX, gY;
        Local2Global(x_real, y_real, gX, gY);
        m_cam_data.points[data_count_cam].x = gX;
        m_cam_data.points[data_count_cam].y = gY;
        m_cam_data.points[data_count_cam].z = 1.0;

        if (scan2->data[i]<=10 && scan2->data[i + 1]<=10 && scan2->data[i + 2]<=10) continue;
        m_cam_data.points[data_count_cam].b = scan2->data[i];
        m_cam_data.points[data_count_cam].g = scan2->data[i + 1];
        m_cam_data.points[data_count_cam++].r = scan2->data[i + 2];
    }
}

void HeightMap::constructGridCam(const sensor_msgs::ImageConstPtr& scan, size_t &data_count_cam)
{
    // for debugging mk --------------------------------
    cv::Mat cvImg = cv_bridge::toCvShare(scan, "bgr8")->image;
    cv_bridge::CvImage out_msg;
    // out_msg.header   = header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    out_msg.image    = cvImg; // Your cv::Mat
    sensor_msgs::ImageConstPtr scan2 = out_msg.toImageMsg();
    //  --------------------------------

    // int goalPixX = (int)(-m_goalX*cam2groundLen + x_cali + scan->height/2.0);
    // int goalPixY = (int)(m_goalY*cam2groundLen/1.5 - y_cali + scan->width/2.0);
    // cv::Mat cvImg, cvSubImg, cvCannyImg;
    // if (m_parkingFlag)  cvImg = cv_bridge::toCvShare(scan, "rgb8")->image;
    // else    cvImg = cv_bridge::toCvShare(scan, "bgr8")->image;
    
    // double radius = 51;
    // int rectX = goalPixX - radius*cos(-45*3.14/180.0), rectY = goalPixY + radius*sin(-45*3.14/180.0);
    // cv::Rect ROI_image(rectX, rectY, radius/1.4*2, radius/1.4*2);
    // bool cvFlag = true;
    // if (rectX < 0 || 480 < rectX || rectY < 0 || 320 < rectY ||
    //     rectX+radius/1.4*2 < 0 || 480 < rectX+radius/1.4*2 || rectY+radius/1.4*2 < 0 || 320 < rectY+radius/1.4*2)
    //     cvFlag = false; 

    // if (cvFlag) {
    //     cvSubImg = cvImg(ROI_image);
        
    //     int whiteTh = 129;
    //     cv::Mat whiteMask, cvWhiteImg;
    //     cv::inRange(cvSubImg, cv::Scalar(whiteTh, whiteTh, whiteTh), cv::Scalar(255, 255, 255), whiteMask);
    //     cv::bitwise_and(cvSubImg, cvSubImg, cvWhiteImg, whiteMask);
    //     // cv::imshow("df", cvWhiteImg);
    //     // cv::waitKey(10);

    //     cv::Canny(cvWhiteImg, cvCannyImg, 175, 255);

    //     vector<Vec2f> lines;
    //     HoughLines(cvCannyImg, lines, 1, CV_PI/360, radius/1.4*2*0.55);

    //     for( size_t i = 0; i < lines.size(); i++ ) {
    //         float rho = lines[i][0], theta = lines[i][1];
    //         cout << i << ": " << -1.0*AngleUtils::toRange_PItoPI(m_car.th) << " " << theta << endl;
    //         cv::Point pt1, pt2;
    //         double a = cos(theta), b = sin(theta);
    //         double x0 = a*rho, y0 = b*rho;
    //         pt1.x = cvRound(x0 + 111*(-b));
    //         pt1.y = cvRound(y0 + 111*(a));
    //         pt2.x = cvRound(x0 - 111*(-b));
    //         pt2.y = cvRound(y0 - 111*(a));
    //         cv::line(cvSubImg, pt1, pt2, Scalar(0,0,255), 1, CV_AA);
    //     }
    //     cv::rectangle(cvImg, cv::Point(rectX, rectY), cv::Point(rectX+radius/1.4*2, rectY+radius/1.4*2), cv::Scalar(0,255,0), 1);
    // }

    for (size_t i = 1; i < scan->data.size(); i += 3)
    {
        double pixX = i/3 / scan->width;
        double pixY = i/3 - pixX*scan->width;
        
        double x_real, y_real;
        if (m_parkingFlag) {
            if (pixY < 0 || 170 < pixY || pixX < 5 || 310 < pixX)
                continue;
            x_real = 1.57*(scan->width/2.0 - pixX - 80) / cam2groundLen;     // + m_XY_resol;
            y_real = 2.1*(scan->height/2.0 - pixY - 80) / cam2groundLen;    // - m_XY_resol;
            // x_real = 1.5*(scan->width/2.0 - pixX - 76)*0.035;     // + m_XY_resol;
            // y_real = 2.1*(scan->height/2.0 - pixY - 83)*0.05;    // - m_XY_resol;
        }
        else {                
            x_real = (scan->height/2.0 - pixY + 83)*0.039;// - m_XY_resol;
            y_real = (pixX-scan->width/2.0 + 76)*0.0475;// + m_XY_resol;
        }
        
        double gX, gY;
        Local2Global(x_real, y_real, gX, gY);
        m_cam_data.points[data_count_cam].x = gX-0.1;
        m_cam_data.points[data_count_cam].y = gY+0.1;
        // m_cam_data.points[data_count_cam].x = x_real;
        // m_cam_data.points[data_count_cam].y = y_real;
        m_cam_data.points[data_count_cam].z = -0.01;

        if (m_parkingFlag) {
            m_cam_data.points[data_count_cam].g = scan2->data[i];
            m_cam_data.points[data_count_cam].b = scan2->data[i + 1];
            m_cam_data.points[data_count_cam++].r = scan2->data[i + 2];
        }
        else {
            m_cam_data.points[data_count_cam].g = scan2->data[i];
            m_cam_data.points[data_count_cam].r = scan2->data[i + 1];
            m_cam_data.points[data_count_cam++].b = scan2->data[i + 2];
        }
    }
}


void HeightMap::real2arr(double recvX, double recvY, int& outX, int& outY) { //returns to the grid "index"
    outX = (m_gridDim / 2.0) + ceil(recvX / m_gridResol) - 1;
    outY = (m_gridDim / 2.0) + ceil(recvY / m_gridResol) - 1;
    // outX = (m_gridDim / 2.0) + recvX / m_gridResol;
    // outY = (m_gridDim / 2.0) + recvY / m_gridResol;
}

void HeightMap::arr2real(int recvX, int recvY, double& outX, double& outY) { //returns the grid index to the center of the grid
    outX = (recvX + 1) * m_gridResol - (m_gridResol*m_gridDim - m_gridResol) / 2.0;
    outY = (recvY + 1) * m_gridResol - (m_gridResol*m_gridDim - m_gridResol) / 2.0;
    // outX = recvX * m_gridResol - (m_gridResol*m_gridDim - m_gridResol) / 2.0;
    // outY = recvY * m_gridResol - (m_gridResol*m_gridDim - m_gridResol) / 2.0;
}

void HeightMap::Local2Global(double Lx, double Ly, double &m_gX, double &m_gY) {
    m_gX = m_car.x + (Lx * cos(m_car.th) - Ly * sin(m_car.th));
    m_gY = m_car.y + (Lx * sin(m_car.th) + Ly * cos(m_car.th));
}

void HeightMap::Rotate(double inX, double inY, double th, double X, double Y, double &m_gX, double &m_gY) {
    m_gX = inX + (X * cos(th) - Y * sin(th));
    m_gY = inY + (X * sin(th) + Y * cos(th));
}

void HeightMap::Global2Local(double m_gX, double m_gY, double &Lx, double &Ly) {
    double tmpX = m_gX - m_car.x;
    double tmpY = m_gY - m_car.y;
    Lx = tmpX * cos(m_car.th) + tmpY * sin(m_car.th);
    Ly = tmpX * -sin(m_car.th) + tmpY * cos(m_car.th);
}

void HeightMap::processData(const VPointCloud::ConstPtr &scan) {
    m_obstaclePtCloudLocal.clear();
    m_obstaclePtCloudLocal.header.frame_id = "map"; //"camera";   //"camera_init";

    // m_obstaclePtCloudLocal.points.resize(scan->points.size());
    size_t data_count_velo = 0;
    // constructGridClouds(scan, data_count_velo);
    constructGridClouds_forSemantic(scan, data_count_velo);
    // m_obstaclePtCloudLocal.points.resize(data_count_velo);
    // if (Pub_obstacleLocal.getNumSubscribers() > 0)
    //     Pub_obstacleLocal.publish(m_obstaclePtCloudLocal);
}
 
void HeightMap::callBackImage(const sensor_msgs::ImageConstPtr& scan) {
    if (!m_parkingFlag) {
        m_cam_data.clear();
        m_cam_data.header.frame_id = "map";   //"camera_init";

        m_cam_data.points.resize(scan->data.size());

        size_t data_count_cam = 0;
        constructGridCam(scan, data_count_cam);
        m_cam_data.points.resize(data_count_cam);

        if (Pub_cam.getNumSubscribers() > 0)
            Pub_cam.publish(m_cam_data);
    }
}

void HeightMap::callBackImageDist(const sensor_msgs::ImageConstPtr& scan) {
    m_cam_data.clear();
    m_cam_data.header.frame_id = "map";   //"camera_init";

    m_cam_data.points.resize(scan->data.size());

    size_t data_count_cam = 0;
    constructGridDist(scan, data_count_cam);
    m_cam_data.points.resize(data_count_cam);

    if (Pub_dist.getNumSubscribers() > 0)
        Pub_dist.publish(m_cam_data);
}



void HeightMap::callBackImageRaw(const sensor_msgs::ImageConstPtr& scan) {
    // if (m_parkingFlag) {
        m_cam_data.clear();
        m_cam_data.header.frame_id = "map";   //"camera_init";

        m_cam_data.points.resize(scan->data.size());

        size_t data_count_cam = 0;
        constructGridCam(scan, data_count_cam);
        m_cam_data.points.resize(data_count_cam);

        if (Pub_cam.getNumSubscribers() > 0)
            Pub_cam.publish(m_cam_data);
    // }
}

void HeightMap::CallbackPath(const nav_msgs::Path::ConstPtr& msg) {
    m_parkingFlag = true;
}


}// namespace velodyne_height_map
