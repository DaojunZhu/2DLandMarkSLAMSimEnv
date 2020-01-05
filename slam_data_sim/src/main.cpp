

#include <ros/ros.h>
#include <slam_data_sim/RangeBearingObsData.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <eigen3/Eigen/Core>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <opencv2/core/core.hpp>

using namespace std;
using namespace Eigen;

double m_r; //small map circle radius
double m_R; //big map circle radius
double t_r; //robot trajectory radius
double m_dw_1; //the small map angle intervel 
double m_dw_2; //the big map angle interval
double yawrate; //the noise-free angular velocity for simulation robot motion
double odom_pub_rate;   //the rate of odometer measurement
double obs_pub_rate;    //the rate of landmark observation 
double max_obs_range;   //the maximum observation range

ros::Publisher map_pub;
ros::Publisher pose_traj_pub;
ros::Publisher odom_traj_pub;
ros::Publisher obs_data_pub;
ros::Publisher odom_data_pub;
ros::Publisher cur_true_pose_marker_pub;
ros::Publisher obs_range_marker_pub;
ros::Publisher obs_lines_marker_pub;

visualization_msgs::MarkerArray map_markers;

vector<Vector2d> landmark_map;

vector<Vector3d> traj;
vector<Vector3d> odom_traj;

Vector3d last_pose;
Vector3d last_odom;
double last_time;
double first_time;

double control_noise_v;
double control_noise_w;
double obs_noise_r;
double obs_noise_angle;

cv::RNG rng;


void generate_map()
{
    //generate the small circle map
    int num1 = 2*M_PI / m_dw_1;
    for(int i = 0; i < num1; ++i){
        double theta = i * m_dw_1;
        double xi = m_r * cos(theta);
        double yi = m_r * sin(theta) + m_r + 1;
        landmark_map.push_back(Vector2d(xi,yi));
    }
    //generate the big circle map
    int num2 = 2*M_PI / m_dw_2;
    for(int i = 0; i < num2; ++i){
        double theta = i * m_dw_2;
        double xi = m_R * cos(theta);
        double yi = m_R * sin(theta) + m_R - 1;
        landmark_map.push_back(Vector2d(xi,yi));
    }
}

//normalize angle
double normalizeAngle(double x)
{
    return atan2(sin(x),cos(x));
}

void motion_model(const Vector3d& last_pose,const Vector2d& u, 
                const double& dt, Vector3d& new_pose)
{
    const double& theta = last_pose(2);
    const double& v = u(0);
    const double& w = u(1);
    //no motion
    if(w < 1e-5){
        new_pose = last_pose;
        return;
    }
    new_pose(0) = last_pose(0) + v*cos(theta)*dt;
    new_pose(1) = last_pose(1) + v*sin(theta)*dt;
    new_pose(2) = last_pose(2) + w*dt;
    new_pose(2) = normalizeAngle(new_pose(2));
}

void generate_control_input(Vector2d& u, const double& time)
{
    // wait at first 
    if(time - first_time <= 5.0){
        u.setZero();
    }
    else{
        u(1) = yawrate;
        u(0) = yawrate * t_r;
    }
}

int generate_obs(slam_data_sim::RangeBearingObsData& obs)
{
    int id = 0;
    for(auto lm : landmark_map){
        const double& rx = last_pose(0);
        const double& ry = last_pose(1);
        const double& lx = lm(0);
        const double& ly = lm(1);
        double r = sqrt((rx-lx)*(rx-lx)+(ry-ly)*(ry-ly));
        double angle = atan2(ly-ry,lx-rx) - last_pose(2);
        angle = normalizeAngle(angle);
        if(r <= max_obs_range){
            double r_noise = r + rng.gaussian(obs_noise_r);
            double angle_noise = angle + rng.gaussian(obs_noise_angle);
            angle_noise = normalizeAngle(angle_noise);
            obs.ids.push_back(id);
            obs.ranges.push_back(r_noise);
            obs.angles.push_back(angle_noise);
        }
        ++id;
    }
    return obs.ids.size();
}

void generate_odom(const double& time, const Vector2d& u,
                const double& dt,Vector3d& new_odom)
{
    double v_noise = u(0) + rng.gaussian(control_noise_v);
    double w_noise = u(1) + rng.gaussian(control_noise_w);
    Vector2d u_noise(v_noise,w_noise);
    motion_model(last_odom,u_noise,dt,new_odom);
}




void publish_map(ros::Time time)
{
    //publish map
    visualization_msgs::Marker lm_marker;
    lm_marker.header.frame_id = "world";
    lm_marker.header.stamp = time;
    lm_marker.ns = "landmark_map";
    lm_marker.type = visualization_msgs::Marker::CUBE;
    lm_marker.action = visualization_msgs::Marker::ADD;
    lm_marker.scale.x = 0.05;
    lm_marker.scale.y = 0.05;
    lm_marker.scale.z = 0.05;
    lm_marker.color.a = 1.0;
    lm_marker.color.r = 0.0;
    lm_marker.color.g = 1.0;
    lm_marker.color.b = 0.0;

    int id = 0;
    for(auto lm : landmark_map){
        lm_marker.id = id++;
        lm_marker.pose.position.x = lm(0);
        lm_marker.pose.position.y = lm(1);
        lm_marker.pose.position.z = 0.;
        lm_marker.pose.orientation.x = 0.;
        lm_marker.pose.orientation.y = 0.;
        lm_marker.pose.orientation.z = 0.;
        lm_marker.pose.orientation.w = 1.;
        map_markers.markers.push_back(lm_marker);
    }

    map_pub.publish(map_markers);

}

void publish(const ros::Time& time)
{
    //publish current true pose marker
    visualization_msgs::Marker curr_pose_marker;
    curr_pose_marker.header.frame_id="world";
    curr_pose_marker.header.stamp = time;
    curr_pose_marker.ns = "true_state";
    curr_pose_marker.id = 0;
    curr_pose_marker.type = visualization_msgs::Marker::CUBE;
    curr_pose_marker.pose.position.x = last_pose(0);
    curr_pose_marker.pose.position.y = last_pose(1);
    curr_pose_marker.pose.position.z = 0.;
    tf2::Quaternion q;
    q.setRPY(0.,0.,last_pose(2));
    q.normalize();
    curr_pose_marker.pose.orientation = tf2::toMsg(q);
    curr_pose_marker.scale.x = 0.5;
    curr_pose_marker.scale.y = 0.3;
    curr_pose_marker.scale.z = 0.2;
    curr_pose_marker.color.a = 1.0;
    curr_pose_marker.color.r = 0.;
    curr_pose_marker.color.g = 0.;
    curr_pose_marker.color.b = 1.;
    cur_true_pose_marker_pub.publish(curr_pose_marker);

    //publish observation range marker
    visualization_msgs::Marker obs_range_marker;
    obs_range_marker.header.frame_id = "world";
    obs_range_marker.header.stamp = time;
    obs_range_marker.ns = "obs";
    obs_range_marker.id = 0;
    obs_range_marker.type = visualization_msgs::Marker::CYLINDER;
    obs_range_marker.pose = curr_pose_marker.pose;
    obs_range_marker.scale.x = max_obs_range*2;
    obs_range_marker.scale.y = max_obs_range*2;
    obs_range_marker.scale.z = 0.001;
    obs_range_marker.color.a = 0.5;
    obs_range_marker.color.r = 0.5;
    obs_range_marker.color.g = 0.5;
    obs_range_marker.color.b = 0.5;
    obs_range_marker_pub.publish(obs_range_marker);

    //publish true trajectory
    nav_msgs::Path path;
    path.header.frame_id = "world";
    path.header.stamp = time;
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world";
    pose_msg.header.stamp = time;
    for(auto pose : traj){
        pose_msg.pose.position.x = pose(0);
        pose_msg.pose.position.y = pose(1);
        pose_msg.pose.position.z = 0.;
        tf2::Quaternion q;
        q.setRPY(0.,0.,pose(2));
        q.normalize();
        pose_msg.pose.orientation = tf2::toMsg(q);
        path.poses.push_back(pose_msg);
    }
    pose_traj_pub.publish(path);

    //publish odom msg
    nav_msgs::Odometry odom;
    odom.header.frame_id = "world";
    odom.header.stamp = time;
    odom.child_frame_id = "odom";
    odom.pose.pose.position.x = last_odom(0);
    odom.pose.pose.position.y = last_odom(1);
    odom.pose.pose.position.z = 0.;
    tf2::Quaternion q2;
    q2.setRPY(0.,0.,last_odom(2));
    q2.normalize();
    odom.pose.pose.orientation = tf2::toMsg(q2);
    odom_data_pub.publish(odom);

    //publish odom trajectory
    nav_msgs::Path odom_path;
    odom_path.header.frame_id = "world";
    odom_path.header.stamp = time;
    geometry_msgs::PoseStamped odom_msg;
    odom_msg.header.frame_id = "world";
    odom_msg.header.stamp = time;
    for(auto odom : odom_traj){
        odom_msg.pose.position.x = odom(0);
        odom_msg.pose.position.y = odom(1);
        odom_msg.pose.position.z = 0.;
        tf2::Quaternion q;
        q.setRPY(0.,0.,odom(2));
        q.normalize();
        odom_msg.pose.orientation = tf2::toMsg(q);
        odom_path.poses.push_back(odom_msg);
    }
    odom_traj_pub.publish(odom_path);

    
}

void odom_pub_cb(const ros::TimerEvent& )
{
    ros::Time c_time = ros::Time::now();
    double curr_time = c_time.toSec();
    Vector3d curr_pose;
    Vector3d curr_odom;
    double dt = curr_time - last_time;
    Vector2d u;
    generate_control_input(u,curr_time);
    motion_model(last_pose,u,dt,curr_pose);
    traj.push_back(curr_pose);
    generate_odom(curr_time,u,dt,curr_odom);
    odom_traj.push_back(curr_odom);
    last_pose = curr_pose;
    last_time = curr_time;
    last_odom = curr_odom;
    publish(c_time);
}

//for debug
void pub_obs_line(const ros::Time& time ,const slam_data_sim::RangeBearingObsData& obs)
{
    visualization_msgs::Marker obs_lines;
    obs_lines.header.stamp = time;
    obs_lines.header.frame_id = "world";
    obs_lines.type = visualization_msgs::Marker::LINE_LIST;
    obs_lines.action = visualization_msgs::Marker::ADD;
    obs_lines.ns = "obs_lines";
    obs_lines.id = 0;
    obs_lines.scale.x = 0.1;
    obs_lines.color.a = 1.0;
    obs_lines.color.r = 0.;
    obs_lines.color.g = 0.;
    obs_lines.color.b = 1.;
    
    for(int i = 0; i < obs.ids.size(); ++i){
        const int& id = obs.ids[i];
        const double& r = obs.ranges[i];
        const double& theta = obs.angles[i];
        geometry_msgs::Point p0, p1;
        p0.x = last_pose(0);
        p0.y = last_pose(1);
        p0.z = 0.;
        p1.x = p0.x + r * cos(theta+last_pose(2));
        p1.y = p0.y + r * sin(theta+last_pose(2));
        p1.z = 0.;
        obs_lines.points.push_back(p0);
        obs_lines.points.push_back(p1);
    }

    obs_lines_marker_pub.publish(obs_lines);
}


void obs_pub_cb(const ros::TimerEvent& )
{
    slam_data_sim::RangeBearingObsData obs_data;
    ros::Time time = ros::Time::now();
    obs_data.header.stamp = time;
    obs_data.header.frame_id = "robot";
    int num = generate_obs(obs_data);
    cout << "Observation landmark number: " << num << endl;
    obs_data_pub.publish(obs_data);
    pub_obs_line(time,obs_data);
}



int main(int argc,char** argv)
{
    ros::init(argc,argv,"generate_data");
    ROS_INFO("node initialized.");
    
    ros::NodeHandle private_nh("~");

    private_nh.param<double>("small_map_raduis",m_r,10.0);
    private_nh.param<double>("big_map_radius",m_R,12.0);
    private_nh.param<double>("sim_traj_radius",t_r,11.0);
    private_nh.param<double>("small_map_angle_interval",m_dw_1,5.);
    m_dw_1 = m_dw_1 * M_PI / 180;  
    private_nh.param<double>("big_map_angle_interval",m_dw_2,2.);
    m_dw_2 = m_dw_2 * M_PI / 180;
    private_nh.param<double>("angular_velocity",yawrate,0.1);
    private_nh.param<double>("control_noise_v",control_noise_v,0.5);
    private_nh.param<double>("control_noise_w",control_noise_w,10.);
    control_noise_w *= M_PI / 180;
    private_nh.param<double>("obs_noise_r",obs_noise_r,0.3);
    private_nh.param<double>("obs_noise_angle",obs_noise_angle,2.);
    obs_noise_angle *= M_PI / 180;
    private_nh.param<double>("odom_pub_rate",odom_pub_rate,100);
    private_nh.param<double>("obs_pub_rate",obs_pub_rate,30);
    private_nh.param<double>("max_obs_range",max_obs_range,3.);

    map_pub = private_nh.advertise<visualization_msgs::MarkerArray>("map",1,true);
    pose_traj_pub = private_nh.advertise<nav_msgs::Path>("true_traj",10);
    odom_traj_pub = private_nh.advertise<nav_msgs::Path>("odom_traj",10);
    obs_data_pub = private_nh.advertise<slam_data_sim::RangeBearingObsData>("observations",10);
    odom_data_pub = private_nh.advertise<nav_msgs::Odometry>("odom",10);
    cur_true_pose_marker_pub = private_nh.advertise<visualization_msgs::Marker>("robot",10);
    obs_range_marker_pub = private_nh.advertise<visualization_msgs::Marker>("sensor",10);
    obs_lines_marker_pub = private_nh.advertise<visualization_msgs::Marker>("obs_lines",10);

    generate_map();
    ROS_INFO("Landmark map generation done.");
    publish_map(ros::Time::now());

    last_pose = Vector3d(0.,0.,0.);
    last_odom = Vector3d(0.,0.,0.);
    last_time = ros::Time::now().toSec();
    first_time = last_time;
    
    ros::Timer timer1 = private_nh.createTimer(
        ros::Duration(1/odom_pub_rate),odom_pub_cb);
    ros::Timer timer2 = private_nh.createTimer(
        ros::Duration(1/obs_pub_rate),obs_pub_cb);

    ros::spin();

    return 0;
}