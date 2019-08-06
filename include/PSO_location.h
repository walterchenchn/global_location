#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include <dynamic_reconfigure/server.h>
#include <global_location/test1Config.h>
#include "geometry_msgs/Pose2D.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/PoseArray.h"
#include <iostream>
#include <algorithm>
#include <pthread.h>

using namespace std;

class particle
{
public:
    geometry_msgs::Pose2D point;
    double weight;
};
bool operator<(particle particle1,particle particle2);

class pso_location
{
public:
    pso_location();
    ~pso_location();
    void startGlobalLocation();

private:
    ros::NodeHandle n_;
    nav_msgs::OccupancyGrid global_map_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_map_;
    ros::Publisher  pub_ori_;

    int particle_numble_;
    int feature_point_number_;
    double variance_of_lidar_point_;
    int iterations_;
    int search_scope_;
    int read_lidar_flag_;
    vector<geometry_msgs::Pose2D> watch_point_;
    vector<particle> pose_swarm_;
    particle max_weight_pose_;

    void dynamicCB(global_location::test1Config& config,uint32_t level);
    void lidarCB(sensor_msgs::LaserScan scan);
    void mapCB(nav_msgs::OccupancyGrid map);
    void productParticleSwarm();
    void resampleParticleSwarm();
    void calculateParticleWeight();
    double distanceToObstacle(double x,double y);
    void initialPose();

    void PthreadStart();
    static void* ThreadStart(void * arg);
    void threadRun();
    pthread_t m_tid_;

    void analogInputData();
    void singleParticleTest(double x,double y,double theta);

    ros::Publisher pub_marker_;
    void displayEstimatepose();
};
