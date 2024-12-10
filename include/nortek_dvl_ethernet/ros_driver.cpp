/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Authors: 
      Lin Zhao <linzhao@uri.edu>
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/


#include <nortek_dvl_ethernet/udp_socket_handler.h>
#include <nortek_dvl_ethernet/ros_driver.h>


NortekDvlRos::NortekDvlRos(
    const ros::NodeHandle &nh,
    const ros::NodeHandle &nh_private) :
    nh_(nh), nh_private_(nh_private)
{
    // load parameters
    LoadParam();

    // setup ros
    SetupRos();

    // init the UDP interface
    InitDataInterface();
}

NortekDvlRos::~NortekDvlRos() {

    //! TODO: close something
}

void NortekDvlRos::LoadParam()
{
    // UDP configuration
    nh_private_.param<int>("UDP/udp_rx", udp_param_.udp_rx, DEFAULT_UDP_RX);    
    nh_private_.param<int>("UDP/udp_tx", udp_param_.udp_tx, DEFAULT_UDP_TX);    
    nh_private_.param<std::string>("UDP/udp_address", udp_param_.udp_address, DEFAULT_UDP_ADDRESS);
    nh_private_.param<int>("UDP/buffer_size", udp_param_.buffer_size, DEFAULT_UDP_BUFFER);    

    // DVL configuration
    nh_private_.param<double>("DVL/beam_angle", beam_angle_, 25.0);
    nh_private_.param<double>("DVL/sound_speed", sound_speed_, 1500.0);

    // ROS configuration
    nh_private_.param<std::string>("ROS/frame_id", frame_id_, "nortek_dvl");
}

void NortekDvlRos::SetupRos()
{
    bottom_track_pub_ = nh_.advertise<nortek_dvl_ethernet::NortekDF2>("bottom_track", 10);

    water_track_pub_ = nh_.advertise<nortek_dvl_ethernet::NortekDF2>("water_track", 10);

    current_profile_pub_ = nh_.advertise<nortek_dvl_ethernet::NortekDF3>("current_profile", 10);

    bt_velocity_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("bt_velocity", 10);

    bt_pc2_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("bt_pointcloud", 10);

    bt_range_pub_ = nh_.advertise<sensor_msgs::Range>("bt_range", 10);
    
    wt_velocity_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("wt_velocity", 10);

    pressure_pub_ = nh_.advertise<sensor_msgs::FluidPressure>("pressure", 10);

    test_sub_ = nh_.subscribe("test_data_sub", 10, &NortekDvlRos::CallbackTest, this);
}

void NortekDvlRos::InitDataInterface()
{
    // Set up callback for received data from the socket
    udp_handler_ = std::make_shared<UDPSocketHandler>(udp_param_);

    udp_handler_->SetCallback(
        std::bind(&NortekDvlRos::CallbackUDP, this, std::placeholders::_1, std::placeholders::_2)
    );

    // Set up callback for parsed data (bottom track, current profile, water track)
    parser_ = std::make_shared<NortekDVLParser>();  

    parser_->SetCallbackBT(
        std::bind(&NortekDvlRos::CallbackBT, this, std::placeholders::_1)
    );

    parser_->SetCallbackWT(
        std::bind(&NortekDvlRos::CallbackWT, this, std::placeholders::_1)
    );

    parser_->SetCallbackCP(
        std::bind(&NortekDvlRos::CallbackCP, this, std::placeholders::_1)
    );    
}

void NortekDvlRos::CallbackBT(const nortek_dvl_ethernet::NortekDF2& msg)
{
    // ===================================================================== //
    // publish bottom track message 
    // ===================================================================== //

    nortek_dvl_ethernet::NortekDF2::Ptr bt_msg(
        new nortek_dvl_ethernet::NortekDF2(msg));
    bt_msg->header.frame_id = frame_id_;
    bottom_track_pub_.publish(bt_msg);

    // ===================================================================== //
    // publish twist message
    // ===================================================================== //

    geometry_msgs::TwistWithCovarianceStamped::Ptr twist_msg(
        new geometry_msgs::TwistWithCovarianceStamped);

    TrackToVelocity(bt_msg, twist_msg);
    bt_velocity_pub_.publish(twist_msg);

    // ===================================================================== //
    // publish pressure message
    // ===================================================================== //

    sensor_msgs::FluidPressure::Ptr pressure_msg(
        new sensor_msgs::FluidPressure);
    TrackToPressure(bt_msg, pressure_msg);
    pressure_pub_.publish(pressure_msg);

    // ===================================================================== //
    // publish pointcloud2 message
    // ===================================================================== //
    sensor_msgs::PointCloud2::Ptr pc2_msg(
        new sensor_msgs::PointCloud2);
    TrackToPC2(bt_msg, pc2_msg);
    bt_pc2_pub_.publish(pc2_msg);

    // ===================================================================== //
    // publish range message
    // ===================================================================== //
    sensor_msgs::Range::Ptr range_msg(
        new sensor_msgs::Range);
    TrackToRange(bt_msg, range_msg);
    bt_range_pub_.publish(range_msg);
}

void NortekDvlRos::CallbackWT(const nortek_dvl_ethernet::NortekDF2& msg)
{
    // ===================================================================== //
    // publish water track message 
    // ===================================================================== //

    nortek_dvl_ethernet::NortekDF2::Ptr wt_msg(
        new nortek_dvl_ethernet::NortekDF2(msg));
    wt_msg->header.frame_id = frame_id_;
    water_track_pub_.publish(wt_msg);

    // ===================================================================== //
    // publish twist message
    // ===================================================================== //

    geometry_msgs::TwistWithCovarianceStamped::Ptr twist_msg(
        new geometry_msgs::TwistWithCovarianceStamped);

    TrackToVelocity(wt_msg, twist_msg);
    wt_velocity_pub_.publish(twist_msg);
}

void NortekDvlRos::CallbackCP(const nortek_dvl_ethernet::NortekDF3& msg)
{
    nortek_dvl_ethernet::NortekDF3 cp_msg = msg;
    cp_msg.header.frame_id = frame_id_;
    current_profile_pub_.publish(cp_msg);
}

void NortekDvlRos::CallbackUDP(const uint8_t* data, std::size_t size) {

    auto io_time = ros::Time::now().toSec();

    auto result = parser_->Parse(data, size, io_time);

    if(result == nortek_dvl_structs::parserID::ERROR)
    {
        ROS_ERROR("parsed something wroing");
    }

}

void NortekDvlRos::CallbackTest(const std_msgs::Float32::ConstPtr &msg) {
    //! DEBUG:
    ROS_INFO("%s: Recv: %f", ros::this_node::getName().c_str(), msg->data);
}

void NortekDvlRos::TrackToVelocity(
    const nortek_dvl_ethernet::NortekDF2::Ptr& bt_msg, 
    geometry_msgs::TwistWithCovarianceStamped::Ptr& twist_msg) {

    // prepare velocity and noise (FOM - measurement white noise level)
    double velocity_x, velocity_y, velocity_z, noise_x, noise_y, noise_z;
    velocity_x = bt_msg->velX;
    velocity_y = bt_msg->velY;
    noise_x = bt_msg->fomX;
    noise_y = bt_msg->fomY;
    if (bt_msg->velZ1 == -32.768f && bt_msg->velZ2 != -32.768f) {
        velocity_z = bt_msg->velZ2;
        noise_z = bt_msg->fomZ2;
    }
    else if (bt_msg->velZ1 != -32.768f && bt_msg->velZ2 == -32.768f){
        velocity_z = bt_msg->velZ1;
        noise_z = bt_msg->fomZ1;
    }
    else {
        velocity_z = (bt_msg->velZ1 + bt_msg->velZ2)/ 2.0;
        noise_z = (bt_msg->fomZ1 + bt_msg->fomZ2) / 2.0;
    }        

    // get the scale in case sound speed is different
    double scale = sound_speed_ / bt_msg->speed_sound;

    // fill the data
    //! TODO: FOM seems single ping precision, can we use this for each axis velocity precision
    //        At least the Nortek Nucles used as this 
    twist_msg->header = bt_msg->header;
    twist_msg->twist.twist.linear.x = velocity_x * scale;
    twist_msg->twist.twist.linear.y = velocity_y * scale;
    twist_msg->twist.twist.linear.z = velocity_z * scale;
    twist_msg->twist.covariance[6 * 0 + 0] = noise_x * noise_x;
    twist_msg->twist.covariance[6 * 1 + 1] = noise_y * noise_x;
    twist_msg->twist.covariance[6 * 2 + 2] = noise_z * noise_x;
}

void NortekDvlRos::TrackToPressure(
    const nortek_dvl_ethernet::NortekDF2::Ptr& bt_msg, 
    sensor_msgs::FluidPressure::Ptr& pressure_msg) {

    pressure_msg->header = bt_msg->header;
    pressure_msg->fluid_pressure = (bt_msg->pressure + 1.01325) * 100000;
    //! TODO: accuracy is 0.1% full scale, 
    // For example, a 100 psi gauge with 0.1 % of FS accuracy would be accurate to ± 0.1 psi across its entire range
    // BUT we don't know the full scale of DVL pressure, however, the accuracy is pretty good
    pressure_msg->variance = 0.001;
}

void NortekDvlRos::TrackToPC2(
    const nortek_dvl_ethernet::NortekDF2::Ptr& bt_msg, 
    sensor_msgs::PointCloud2::Ptr& pc2_msg) {

    // 4 beams generated pointcloud with only XYZ property
    sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");    
    modifier.resize(4); 

    // get the scale in case sound speed is different
    double scale = sound_speed_ / bt_msg->speed_sound;

    // re-generate 3D location of target point
    std::vector<Eigen::Vector3d> points;
    double beam_angle = beam_angle_*M_PI/180;

    double beam_azimuth[] = {M_PI/4.0, -M_PI/4.0, -3.0*M_PI/4.0, 3.0*M_PI/4.0};
    for (int i = 0; i < 4; i++) {
        Eigen::Vector3d pt;
        pt(0) = bt_msg->distBeam[i] * scale * tan(beam_angle) * cos(beam_azimuth[i]);
        pt(1) = bt_msg->distBeam[i] * scale * tan(beam_angle) * sin(beam_azimuth[i]);
        pt(2) = bt_msg->distBeam[i] * scale;
        points.push_back(pt);
    }

    // fill the XYZ
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(*pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(*pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(*pc2_msg, "z");

    for (size_t i = 0; i < 4; i++, ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        const Eigen::Vector3d& point = points.at(i);
        *ros_pc2_x = point(0);
        *ros_pc2_y = point(1);
        *ros_pc2_z = point(2);
    }

    // fill the header
    pc2_msg->header = bt_msg->header;
}

void NortekDvlRos::TrackToRange(
    const nortek_dvl_ethernet::NortekDF2::Ptr& bt_msg, 
    sensor_msgs::Range::Ptr& range_msg) {

    range_msg->header = bt_msg->header;
    range_msg->radiation_type = sensor_msgs::Range::ULTRASOUND;
    //! TODO: from Technical specifications, the altitude seems from 0.2~75m,
    //        but from web interface, the max only can be set is 61.62m 
    range_msg->min_range = 0.2;
    range_msg->max_range = 75;
    range_msg->range = bt_msg->altitude;
}