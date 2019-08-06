#include "PSO_location.h"

pso_location::pso_location()
:particle_numble_(500),feature_point_number_(50),variance_of_lidar_point_(1),search_scope_(10),iterations_(50)
,read_lidar_flag_(0)
{
    max_weight_pose_.weight = 0;
    dynamic_reconfigure::Server<global_location::test1Config> server;
    dynamic_reconfigure::Server<global_location::test1Config>::CallbackType f;
    f = boost::bind(&pso_location::dynamicCB,this,_1,_2);
    server.setCallback(f);
    sub_lidar_ = n_.subscribe("/scan",2,&pso_location::lidarCB,this);
    sub_map_ = n_.subscribe("/map",2,&pso_location::mapCB,this);
    pub_ori_ = n_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
    pub_marker_ = n_.advertise<geometry_msgs::PoseArray>("/watch_point",2);
    PthreadStart();
    ros::spin();
}

pso_location::~pso_location()
{}

void pso_location::dynamicCB(global_location::test1Config& config,uint32_t level)
{
    ROS_INFO("reconfigure request:%d,%d,%f",config.particle_number,config.feature_point_number,config.variance_of_lidar_point);
    particle_numble_ = config.particle_number;
    feature_point_number_ = config.feature_point_number;
    search_scope_ = config.search_scope;
    variance_of_lidar_point_ = config.variance_of_lidar_point;
    iterations_ = config.iterations;
}

void pso_location::lidarCB(sensor_msgs::LaserScan scan)
{
    if(read_lidar_flag_ == 0)
    {
        read_lidar_flag_ = 1;
    }
    else
    {
        return;
    }
    watch_point_.clear();
    geometry_msgs::Pose2D pose2D;
    for(int i = 0;i < scan.ranges.size();i++)
    {
        pose2D.x = scan.ranges[i] * cos(scan.angle_min + i * scan.angle_increment) + 0.23;
        pose2D.y = scan.ranges[i] * sin(scan.angle_min + i * scan.angle_increment);
        pose2D.theta = 0;
        if((scan.ranges[i] > 0) && (scan.ranges[i] < 100))
        {
            watch_point_.push_back(pose2D);
            i = i + scan.ranges.size() / feature_point_number_ - 2;
        }
        if(watch_point_.size() >= feature_point_number_)
        {
            return;
        }
    }
    cout << "load the lidar data successed" << endl;
}

void pso_location::mapCB(nav_msgs::OccupancyGrid map)
{
    global_map_.header.stamp = map.header.stamp;
    global_map_.header.frame_id = map.header.frame_id;
    global_map_.info.map_load_time = map.info.map_load_time;
    global_map_.info.resolution = map.info.resolution;
    global_map_.info.width = map.info.width;
    global_map_.info.height = map.info.height;
    global_map_.info.origin.position.x = map.info.origin.position.x;
    global_map_.info.origin.position.y = map.info.origin.position.y;
    global_map_.info.origin.position.z = map.info.origin.position.z;
    global_map_.info.origin.orientation.x = map.info.origin.orientation.x;
    global_map_.info.origin.orientation.y = map.info.origin.orientation.y;
    global_map_.info.origin.orientation.z = map.info.origin.orientation.z;
    global_map_.info.origin.orientation.w = map.info.origin.orientation.w;
    global_map_.data.resize(map.data.size());
    for(int i = 0;i < map.data.size();i++)
    {
        global_map_.data[i] = map.data[i];
    }
    cout << "load the map successed" << endl;
}

void pso_location::startGlobalLocation()
{
    cout << endl << endl;
    cout << "the particle number is:" << particle_numble_ << endl;
    cout << "the feature point number is:" << feature_point_number_ << endl;
    cout << "variance of lidar point is:" << variance_of_lidar_point_ << endl;
    cout << "the search scope is:" << search_scope_ << endl;
    cout << "the iteration number is:" << iterations_ << endl;
    cout << "please enter to begin location." << endl;
    getchar();
    read_lidar_flag_ = 0;
    while(read_lidar_flag_ !=1)
    {
        cout << "hav't receive the lidar point!" << endl;
        ros::Duration(1).sleep();
    }

    if(global_map_.data.size() == 0)
    {
        cout << "hav't receive the map!" << endl;
        cout << "Please load the map and restart the program" << endl;
        return;
    }

    cout << "start the global location." << endl;

    productParticleSwarm();
    calculateParticleWeight();
    for(int i = 0;i < iterations_;i++)
    {
        resampleParticleSwarm();
        //productParticleSwarm();
        calculateParticleWeight();
    }
    cout << "end the global location." << endl;
    sort(pose_swarm_.begin(),pose_swarm_.end());
    cout << "the pose is:(" << max_weight_pose_.point.x << "," << max_weight_pose_.point.y << "," << max_weight_pose_.point.theta << ")" << max_weight_pose_.weight << endl;
    initialPose();
}

void pso_location::productParticleSwarm()
{
    cout << "product the particle swarm." << endl;
    pose_swarm_.clear();
    max_weight_pose_.weight = 0;
    for(int i = 0;i < particle_numble_;i++)
    {
        particle pose;
        pose.point.x = rand() % (int)(global_map_.info.width * global_map_.info.resolution) + rand() % 100 / 100.0;
        pose.point.y = rand() % (int)(global_map_.info.height * global_map_.info.resolution) + rand() % 100 / 100.0;
        pose.point.theta = (rand() % 360 + rand() % 100 / 100.0) * 3.14159265 / 180.0;
        pose.weight = 0;
        if(global_map_.data[(int)(pose.point.x / global_map_.info.resolution)+ (int)(pose.point.y / global_map_.info.resolution) * global_map_.info.width] != 0)
        {
            i--;
        }
        else
        {
            pose_swarm_.push_back(pose);
        }
    }
    displayEstimatepose();
}

void pso_location::resampleParticleSwarm()
{
    sort(pose_swarm_.begin(),pose_swarm_.end());
    if(pose_swarm_.size() < particle_numble_)
    {
        for(int i = 0;i < particle_numble_ - pose_swarm_.size();i++)
        {
            particle pose;
            pose.point.x = 0;
            pose.point.y = 0;
            pose.point.theta = 0;
            pose.weight = 0;
            pose_swarm_.insert(pose_swarm_.begin(),pose);
        }
        cout << "increase the particle." << endl;
    }
    else if(pose_swarm_.size() > particle_numble_)
    {
        pose_swarm_.erase(pose_swarm_.begin(),pose_swarm_.begin() + pose_swarm_.size() - particle_numble_);
        cout << "descrease the particle." << endl;
    }
    for(int i = 0;i < pose_swarm_.size() * 3 / 4;i++)
    {
        pose_swarm_[i].point.x = rand() % (int)(global_map_.info.width * global_map_.info.resolution) + rand() % 100 / 100.0;
        pose_swarm_[i].point.y = rand() % (int)(global_map_.info.height * global_map_.info.resolution) + rand() % 100 / 100.0;
        pose_swarm_[i].point.theta = (rand() % 360 + rand() % 100 / 100.0) * 3.14159265 / 180.0;
        pose_swarm_[i].weight = 0;
        if(global_map_.data[(int)(pose_swarm_[i].point.x / global_map_.info.resolution)+ (int)(pose_swarm_[i].point.y / global_map_.info.resolution) * global_map_.info.width] != 0)
        {
            i--;
        }
    }
    for(int i = pose_swarm_.size() * 3 / 4;i < pose_swarm_.size() * 7 / 8;i++)
    {
        pose_swarm_[i].point.x = pose_swarm_[i].point.x + (rand() % 100 - 50) / 100.0;
        pose_swarm_[i].point.y = pose_swarm_[i].point.y + (rand() % 100 - 50) / 100.0;
        pose_swarm_[i].point.theta = pose_swarm_[i].point.theta + (rand() % 100 - 50) / 100.0;
        pose_swarm_[i].weight = 0;
    }
    for(int i = pose_swarm_.size() * 7 / 8;i < pose_swarm_.size();i++)
    {
        pose_swarm_[i].point.x = max_weight_pose_.point.x + (rand() % 100 - 50) / 100.0;
        pose_swarm_[i].point.y = max_weight_pose_.point.y + (rand() % 100 - 50) / 100.0;
        pose_swarm_[i].point.theta = max_weight_pose_.point.theta + (rand() % 100 - 50) / 100.0;
        pose_swarm_[i].weight = 0;
    }

    displayEstimatepose();
}

void pso_location::calculateParticleWeight()
{
    for(int n = 0;n < particle_numble_;n++)
    {
        pose_swarm_[n].weight = 0;
        for(int i = 0;i < watch_point_.size();i++)
        {
            double map_x = cos(pose_swarm_[n].point.theta) * watch_point_[i].x - sin(pose_swarm_[n].point.theta) * watch_point_[i].y + pose_swarm_[n].point.x;
            double map_y = cos(pose_swarm_[n].point.theta) * watch_point_[i].y + sin(pose_swarm_[n].point.theta) * watch_point_[i].x + pose_swarm_[n].point.y;
            double d = distanceToObstacle(map_x,map_y);
            if(d == 100)
            {
                pose_swarm_[n].weight = pose_swarm_[n].weight - 1;
            }
            else
            {
                pose_swarm_[n].weight += exp((0 - d) / variance_of_lidar_point_);
            }
        }
        if(pose_swarm_[n].weight > max_weight_pose_.weight)
        {
            max_weight_pose_.point.x = pose_swarm_[n].point.x;
            max_weight_pose_.point.y = pose_swarm_[n].point.y;
            max_weight_pose_.point.theta = pose_swarm_[n].point.theta;
            max_weight_pose_.weight = pose_swarm_[n].weight;

            initialPose();
        }
        
    }

    cout << "the best pose is:(" << max_weight_pose_.point.x << "," << max_weight_pose_.point.y
         << "," << max_weight_pose_.point.theta << ")" << max_weight_pose_.weight << endl;
}

double pso_location::distanceToObstacle(double x,double y)
{
    int width = x / global_map_.info.resolution;
    int height = y / global_map_.info.resolution;
    int obs_x;
    int obs_y;
    for(int i = 0;i < search_scope_;i++)
    {
        for(int j = 0 - i;j < i + 1;j++)
        {
            if((width + j >= 0) && (width + j < global_map_.info.width) && (height + i >= 0) && (height + i < global_map_.info.height))
            {
                if(global_map_.data[(height + i) * global_map_.info.width + width + j] == 100)
                {
                    obs_x = width + j;
                    obs_y = height + i;
                    return sqrt((obs_x * global_map_.info.resolution - x) * (obs_x * global_map_.info.resolution - x) + (obs_y * global_map_.info.resolution - y) * (obs_y * global_map_.info.resolution - y));
                    break;
                }
            }
        }
        for(int j = 0 - i;j < i + 1;j++)
        {
            if((width + j >= 0) && (width + j < global_map_.info.width) && (height - i >= 0) && (height - i < global_map_.info.height))
            {
                if(global_map_.data[(height - i) * global_map_.info.width + width + j] == 100)
                {
                    obs_x = width + j;
                    obs_y = height - i;
                    return sqrt((obs_x * global_map_.info.resolution - x) * (obs_x * global_map_.info.resolution - x) + (obs_y * global_map_.info.resolution - y) * (obs_y * global_map_.info.resolution - y));
                    break;
                }
            }
        }
        for(int j = 0 - i;j < i + 1;j++)
        {
            if((width + i >= 0) && (width + i < global_map_.info.width) && (height + j >= 0) && (height + j < global_map_.info.height))
            {
                if(global_map_.data[(height + j) * global_map_.info.width + width + i] == 100)
                {
                    obs_x = width + i;
                    obs_y = height + j;
                    return sqrt((obs_x * global_map_.info.resolution - x) * (obs_x * global_map_.info.resolution - x) + (obs_y * global_map_.info.resolution - y) * (obs_y * global_map_.info.resolution - y));
                    break;
                }
            }
        }
        for(int j = 0 - i;j < i + 1;j++)
        {
            if((width - i >= 0) && (width - i < global_map_.info.width) && (height + j >= 0) && (height + j < global_map_.info.height))
            {
                if(global_map_.data[(height + j) * global_map_.info.width + width + i] == 100)
                {
                    obs_x = width - i;
                    obs_y = height + j;
                    return sqrt((obs_x * global_map_.info.resolution - x) * (obs_x * global_map_.info.resolution - x) + (obs_y * global_map_.info.resolution - y) * (obs_y * global_map_.info.resolution - y));
                    break;
                }
            }
        }
    }
    return 100;
}

void pso_location::PthreadStart()
{
    if(pthread_create(&m_tid_,NULL,ThreadStart,(void*)this) != 0)
    {
        ROS_INFO("Start location thread failed!");
        return; 
    }
}

void* pso_location::ThreadStart(void * arg)
{
    pso_location *ptr =(pso_location*) arg;
    ptr->threadRun();
    return NULL;
}

void pso_location::threadRun()
{
    while(ros::ok())
    {
        startGlobalLocation();
    }
}

void pso_location::initialPose()
{
    geometry_msgs::PoseWithCovarianceStamped msg_poseinit;
    double x,y,oz,ow;
    x = max_weight_pose_.point.x + global_map_.info.origin.position.x;
    y = max_weight_pose_.point.y + global_map_.info.origin.position.y;
    oz = sin(max_weight_pose_.point.theta / 2);
    ow = cos(max_weight_pose_.point.theta / 2);
    msg_poseinit.header.frame_id = "map";
    msg_poseinit.header.stamp = ros::Time::now();
    msg_poseinit.pose.pose.position.x = x;
    msg_poseinit.pose.pose.position.y = y;
    msg_poseinit.pose.pose.orientation.z = oz;
    msg_poseinit.pose.pose.orientation.w = ow;
    pub_ori_.publish(msg_poseinit);
    
    cout << "Had set the initial position(" << x << "," << y << "," << oz << "," << ow << ")" << endl;
}

bool operator<(particle particle1,particle particle2)
{
    return particle1.weight < particle2.weight;
}

void pso_location::analogInputData()
{
    //global_map_.header.stamp = map.header.stamp;
    global_map_.header.frame_id = "test_frame";
    //global_map_.info.map_load_time = map.info.map_load_time;
    global_map_.info.resolution = 0.5;
    global_map_.info.width = 10;
    global_map_.info.height = 10;
    global_map_.info.origin.position.x = 0;
    global_map_.info.origin.position.y = 0;
    global_map_.info.origin.position.z = 0;
    global_map_.info.origin.orientation.x = 0;
    global_map_.info.origin.orientation.y = 0;
    global_map_.info.origin.orientation.z = 0;
    global_map_.info.origin.orientation.w = 1;
    global_map_.data.resize(100);
    for(int i = 0;i < 100;i++)
    {
        global_map_.data[i] = 0;
    }
    for(int i = 0;i < 10;i++)
    {
        global_map_.data[90 + i] = 100;
    }
    for(int i = 0;i < 10;i++)
    {
        //global_map_.data[9 + 10 * i] = 100;
    }
    cout << "load the map successed2" << endl;

    watch_point_.clear();
    geometry_msgs::Pose2D pose2D;
    for(int i = 0;i < feature_point_number_;i++)
    {
        pose2D.x =i * 0.2;
        pose2D.y = i * 0.2;
        pose2D.theta = 0;
        watch_point_.push_back(pose2D);
    }
}

void pso_location::singleParticleTest(double x,double y,double theta)
{
    double weight = 0;
    theta = theta * 3.14159265 / 180;
    for(int i = 0;i < watch_point_.size();i++)
    {
        double map_x = cos(theta) * watch_point_[i].x - sin(theta) * watch_point_[i].y + x - global_map_.info.origin.position.x;
        double map_y = cos(theta) * watch_point_[i].y + sin(theta) * watch_point_[i].x + y - global_map_.info.origin.position.y;
        double d = distanceToObstacle(map_x,map_y);
        if(d == 100.0) weight = weight - 1;
        else weight += exp((0 - d) / variance_of_lidar_point_);
        //weight += distanceToObstacle(map_x,map_y);
        cout << distanceToObstacle(map_x,map_y) << "--";
    }
    cout << endl;
    max_weight_pose_.point.x = x - global_map_.info.origin.position.x;
    max_weight_pose_.point.y = y - global_map_.info.origin.position.x;
    max_weight_pose_.point.theta = theta;
    max_weight_pose_.weight = weight;
    initialPose();

    cout << "the single particle weight is: " << max_weight_pose_.weight << endl;
}

void pso_location::displayEstimatepose()
{
    // geometry_msgs::PoseArray points;
    // points.header.stamp = ros::Time::now();
    // points.header.frame_id = "/map";
    // points.poses.resize(watch_point_.size());
    // for(int i = 0;i < watch_point_.size();i++)
    // {
    //     double cos0 = cos(max_weight_pose_.point.theta);
    //     double sin0 = sin(max_weight_pose_.point.theta);
    //     double x = cos0 * watch_point_[i].x - sin0 * watch_point_[i].y + max_weight_pose_.point.x + global_map_.info.origin.position.x;
    //     double y = cos0 * watch_point_[i].y + sin0 * watch_point_[i].x + max_weight_pose_.point.y + global_map_.info.origin.position.y;
    //     points.poses[i].position.x = x;
    //     points.poses[i].position.y = y;
    //     points.poses[i].position.z = 0;
    //     points.poses[i].orientation.w = 1;
    //     //cout << "error(" << watch_point_[i].x << "," << watch_point_[i].y << ")" << endl;
    // }

    geometry_msgs::PoseArray points;
    points.header.stamp = ros::Time::now();
    points.header.frame_id = "/map";
    points.poses.resize(pose_swarm_.size());
    for(int i = 0;i < pose_swarm_.size();i++)
    {
        points.poses[i].position.x = pose_swarm_[i].point.x + global_map_.info.origin.position.x;
        points.poses[i].position.y = pose_swarm_[i].point.y + global_map_.info.origin.position.y;
        points.poses[i].position.z = 0;
        points.poses[i].orientation.z = sin(pose_swarm_[i].point.theta / 2);
        points.poses[i].orientation.w = cos(pose_swarm_[i].point.theta / 2);
    }
    
    pub_marker_.publish(points);
}