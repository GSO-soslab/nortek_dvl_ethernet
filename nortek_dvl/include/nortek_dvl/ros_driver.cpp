/*
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

    Authors: Lin Zhao <linzhao@uri.edu>
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/


#include <nortek_dvl/udp_socket_handler.h>
#include <nortek_dvl/ros_driver.h>


NortekDvlRos::NortekDvlRos()
    : Node("nortek_dvl_ros_node")
{
    // load parameters
    LoadParam();

    // setup ros
    SetupRos();

    // init the UDP interface
    InitDataInterface();
}

void NortekDvlRos::LoadParam()
{
    // UDP configuration
    this->declare_parameter<int>("UDP.udp_rx", DEFAULT_UDP_RX);
    if (!this->get_parameter("UDP.udp_rx", udp_param_.udp_rx)) {
        RCLCPP_ERROR(this->get_logger(), "UDP.udp_rx: no param available!");
    }
    this->declare_parameter<int>("UDP.udp_tx", DEFAULT_UDP_TX);
    if (!this->get_parameter("UDP.udp_tx", udp_param_.udp_tx)) {
        RCLCPP_ERROR(this->get_logger(), "UDP.udp_tx: no param available!");
    }
    this->declare_parameter<std::string>("UDP.udp_address", DEFAULT_UDP_ADDRESS);
    if (!this->get_parameter("UDP.udp_address", udp_param_.udp_address)) {
        RCLCPP_ERROR(this->get_logger(), "UDP.udp_address: no param available!");
    }
    this->declare_parameter<int>("UDP.buffer_size", DEFAULT_UDP_BUFFER);
    if (!this->get_parameter("UDP.buffer_size", udp_param_.buffer_size)) {
        RCLCPP_ERROR(this->get_logger(), "UDP.buffer_size: no param available!");
    }

    // DVL configuration
    this->declare_parameter<double>("DVL.beam_angle", DEFAULT_DVL_BEAM_ANGLE);
    if (!this->get_parameter("DVL.beam_angle", beam_angle_)) {
        RCLCPP_ERROR(this->get_logger(), "DVL.beam_angle: no param available!");
    }
    this->declare_parameter<double>("DVL.sound_speed", DEFAULT_DVL_SOUND_SPEED);
    if (!this->get_parameter("DVL.sound_speed", sound_speed_)) {
        RCLCPP_ERROR(this->get_logger(), "DVL.sound_speed: no param available!");
    }
    this->declare_parameter<double>("DVL.fluid_density", DEFAULT_DVL_FLUID_DENSITY);
    if (!this->get_parameter("DVL.fluid_density", fluid_density_)) {
        RCLCPP_ERROR(this->get_logger(), "DVL.fluid_density: no param available!");
    }

    // ROS configuration
    this->declare_parameter<std::string>("ROS.sensor_frame_id", "nortek_dvl");
    if (!this->get_parameter("ROS.sensor_frame_id", sensor_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "ROS.sensor_frame_id: no param available!");
    }
    this->declare_parameter<std::string>("ROS.world_frame_id", "world");
    if (!this->get_parameter("ROS.world_frame_id", world_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "ROS.world_frame_id: no param available!");
    }
    this->declare_parameter<double>("ROS.depth_cov", 0.001);
    if (!this->get_parameter("ROS.depth_cov", depth_cov_)) {
        RCLCPP_ERROR(this->get_logger(), "ROS.depth_cov: no param available!");
    }

    //! DEBUG:
    std::cout<<"UDP configure: ";
    std::cout<<" udp_rx:" << udp_param_.udp_rx <<", udp_tx:" << udp_param_.udp_tx;
    std::cout<<" udp_address:" << udp_param_.udp_address <<", buffer_size:" << udp_param_.buffer_size;
    std::cout<<"\n";    

    std::cout<<"DVL configure: ";
    std::cout<<" beam_angle:" << beam_angle_ <<", sound_speed:" << sound_speed_;
    std::cout<<" fluid_density:" << fluid_density_;
    std::cout<<"\n";     

    std::cout<<"ROS configure: ";
    std::cout<<" sensor_frame_id:" << sensor_frame_id_ <<", world_frame_id:" << world_frame_id_;
    std::cout<<" depth_cov:" << depth_cov_;
    std::cout<<"\n";      
}

void NortekDvlRos::SetupRos()
{
    // ros pub
    bottom_track_pub_ = this->create_publisher<nortek_msgs::msg::NortekDF2>(
        "~/bottom_track", 10);

    water_track_pub_ = this->create_publisher<nortek_msgs::msg::NortekDF2>(
        "~/water_track", 10);

    current_profile_pub_ = this->create_publisher<nortek_msgs::msg::NortekDF3>(
        "~/current_profile", 10);

    bt_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "~/bt_velocity", 10);

    bt_pc2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "~/bt_pointcloud", 10);

    bt_altitude_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "~/bt_altitude", 10);

    wt_velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "~/wt_velocity", 10);

    // cp_cells_pub_ = this->create_publisher<nav_msgs::msg::GridCells>(
    //     "cp_cells", 5);

    pressure_pub_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(
        "~/pressure", 10);

    depth_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "~/depth_odometry", 10);
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

void NortekDvlRos::CallbackBT(
    const nortek_msgs::msg::NortekDF2& msg)
{
    // ===================================================================== //
    // publish bottom track message 
    // ===================================================================== //

    auto track_msg = std::make_shared<nortek_msgs::msg::NortekDF2>(msg);
    track_msg->header.frame_id = sensor_frame_id_;
    bottom_track_pub_->publish(*track_msg);

    // ===================================================================== //
    // publish pressure message
    // ===================================================================== //

    auto pressure_msg = std::make_shared<sensor_msgs::msg::FluidPressure>();
    TrackToPressure(track_msg, pressure_msg);
    pressure_pub_->publish(*pressure_msg);

    // ===================================================================== //
    // publish depth odometry message
    // ===================================================================== //

    auto depth_odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    TrackToDepthOdom(track_msg, depth_odom_msg);
    depth_pub_->publish(*depth_odom_msg);

    // RCLCPP_INFO(this->get_logger(), "good_beams: {%d}", track_msg->good_beams);
    if(track_msg->good_beams ==0) {
        return;
    }

    // ===================================================================== //
    // publish twist message
    // ===================================================================== //
    // RCLCPP_INFO(this->get_logger(), "vel_x: {%f}", track_msg->vel_x);
    // RCLCPP_INFO(this->get_logger(), "vel_y: {%f}", track_msg->vel_y);
    // RCLCPP_INFO(this->get_logger(), "vel_z1: {%f}", track_msg->vel_z1);
    // RCLCPP_INFO(this->get_logger(), "vel_z2: {%f}", track_msg->vel_z2);

    std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> twist_msg;
    TrackToVelocity(track_msg, twist_msg);
    if(twist_msg != nullptr) {
        bt_velocity_pub_->publish(*twist_msg);
    }

    // ===================================================================== //
    // publish pointcloud2 message
    // ===================================================================== //
    // RCLCPP_INFO(this->get_logger(), "Dist Beam - 0: {%f}", track_msg->beam_dist[0]);
    // RCLCPP_INFO(this->get_logger(), "Dist Beam - 1: {%f}", track_msg->beam_dist[1]);
    // RCLCPP_INFO(this->get_logger(), "Dist Beam - 2: {%f}", track_msg->beam_dist[2]);
    // RCLCPP_INFO(this->get_logger(), "Dist Beam - 3: {%f}", track_msg->beam_dist[3]);

    auto pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    TrackToPC2(track_msg, pc2_msg);
    bt_pc2_pub_->publish(*pc2_msg);

    // ===================================================================== //
    // publish altitude message
    // ===================================================================== //
    auto altitude_msg = std::make_shared<geometry_msgs::msg::PointStamped>();
    TrackToAltitude(track_msg, altitude_msg);
    bt_altitude_pub_->publish(*altitude_msg);
}

void NortekDvlRos::CallbackWT(
    const nortek_msgs::msg::NortekDF2& msg)
{
    // ===================================================================== //
    // publish water track message 
    // ===================================================================== //

    auto wt_msg = std::make_shared<nortek_msgs::msg::NortekDF2>(msg);
    wt_msg->header.frame_id = sensor_frame_id_;
    water_track_pub_->publish(*wt_msg);

    // ===================================================================== //
    // publish twist message
    // ===================================================================== //
    if(wt_msg->good_beams ==0) {
        return;
    }

    std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> twist_msg;
    TrackToVelocity(wt_msg, twist_msg);
    if(twist_msg != nullptr) {
        wt_velocity_pub_->publish(*twist_msg);
    }    
}

void NortekDvlRos::CallbackCP(
    const nortek_msgs::msg::NortekDF3& msg)
{
    // ===================================================================== //
    // publish current profile message 
    // ===================================================================== //

    auto cp_msg = std::make_shared<nortek_msgs::msg::NortekDF3>(msg);
    cp_msg->header.frame_id = sensor_frame_id_;
    current_profile_pub_->publish(*cp_msg);    

    // ===================================================================== //
    // publish pressure message 
    // ===================================================================== //

    auto pressure_msg = std::make_shared<sensor_msgs::msg::FluidPressure>();
    ProfileToPressure(cp_msg, pressure_msg);
    pressure_pub_->publish(*pressure_msg);

    // ===================================================================== //
    // publish depth odometry message
    // ===================================================================== //
    auto depth_odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    ProfileToDepthOdom(cp_msg, depth_odom_msg);
    depth_pub_->publish(*depth_odom_msg);

    // ===================================================================== //
    // publish cells message 
    // ===================================================================== //

    //! TODO: use nav_msgs/GridCells ?

    //! TODO: sound speed correction ?

}

void NortekDvlRos::CallbackUDP(
    const uint8_t* data, std::size_t size) 
{
    auto io_time = this->get_clock()->now().seconds();

    auto result = parser_->Parse(data, size, io_time);

    if(result == nortek_dvl_structs::parserID::ERROR)
    {
        RCLCPP_ERROR(this->get_logger(), "parsed something wrong");
    }

}

void NortekDvlRos::TrackToVelocity(
    const nortek_msgs::msg::NortekDF2::ConstSharedPtr& track_msg, 
    geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr& twist_msg) 
{
    if(track_msg->vel_x == -32.768f || 
       track_msg->vel_y == -32.768f || 
      (track_msg->vel_z1 == -32.768f && track_msg->vel_z2 != -32.768f))
    {
        return;
    }

    twist_msg = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();

    // prepare velocity and noise (FOM - measurement white noise level)
    double velocity_x, velocity_y, velocity_z, noise_x, noise_y, noise_z;
    velocity_x = track_msg->vel_x;
    velocity_y = track_msg->vel_y;
    noise_x = track_msg->fom_x;
    noise_y = track_msg->fom_y;
    if (track_msg->vel_z1 == -32.768f && track_msg->vel_z2 != -32.768f) {
        velocity_z = track_msg->vel_z2;
        noise_z = track_msg->fom_z2;
    }
    else if (track_msg->vel_z1 != -32.768f && track_msg->vel_z2 == -32.768f){
        velocity_z = track_msg->vel_z1;
        noise_z = track_msg->fom_z1;
    }
    else {
        velocity_z = (track_msg->vel_z1 + track_msg->vel_z2)/ 2.0;
        noise_z = (track_msg->fom_z1 + track_msg->fom_z2) / 2.0;
    }        

    // get the scale in case sound speed is different
    double scale = sound_speed_ / track_msg->speed_sound;

    // fill the data
    //! TODO: FOM seems single ping precision, can we use this for each axis velocity precision
    //        At least the Nortek Nucles used as this 
    twist_msg->header = track_msg->header;
    twist_msg->twist.twist.linear.x = velocity_x * scale;
    twist_msg->twist.twist.linear.y = velocity_y * scale;
    twist_msg->twist.twist.linear.z = velocity_z * scale;
    twist_msg->twist.covariance[6 * 0 + 0] = noise_x * noise_x;
    twist_msg->twist.covariance[6 * 1 + 1] = noise_y * noise_x;
    twist_msg->twist.covariance[6 * 2 + 2] = noise_z * noise_x;
}

void NortekDvlRos::TrackToPressure(
    const nortek_msgs::msg::NortekDF2::ConstSharedPtr& track_msg, 
    sensor_msgs::msg::FluidPressure::SharedPtr& pressure_msg) 
{
    pressure_msg->header = track_msg->header;
    pressure_msg->fluid_pressure = (track_msg->pressure + 1.01325) * 100000;
    //! TODO: accuracy is 0.1% full scale, 
    // For example, a 100 psi gauge with 0.1 % of FS accuracy would be accurate to ± 0.1 psi across its entire range
    // BUT we don't know the full scale of DVL pressure, however, the accuracy is pretty good
    pressure_msg->variance = 0.001;
}

void NortekDvlRos::TrackToDepthOdom(
    const nortek_msgs::msg::NortekDF2::ConstSharedPtr& track_msg, 
    nav_msgs::msg::Odometry::SharedPtr& depth_odom_msg)
{
    // header
    depth_odom_msg->header.stamp = track_msg->header.stamp;
    depth_odom_msg->header.frame_id = world_frame_id_;
    depth_odom_msg->child_frame_id = sensor_frame_id_;
    // convert the pressure (Bar) to depth
    auto depth = (track_msg->pressure * 100000) / ( fluid_density_ * 9.81);
    // construct the odometry message
    depth_odom_msg->pose.pose.position.x = 0.0;
    depth_odom_msg->pose.pose.position.y = 0.0;
    depth_odom_msg->pose.pose.position.z = -depth;
    depth_odom_msg->pose.covariance[6 * 2 + 2] = depth_cov_;
}

void NortekDvlRos::TrackToPC2(
    const nortek_msgs::msg::NortekDF2::ConstSharedPtr& track_msg, 
    sensor_msgs::msg::PointCloud2::SharedPtr& pc2_msg) 
{
    // ===================================================================== //
    // msg property: 4 points with only XYZ
    // ===================================================================== //
    pc2_msg->height = 1;
    // total points
    pc2_msg->width = track_msg->good_beams;
    // fill the field: x,y,z
    sensor_msgs::msg::PointField field;
    field.count = 1;
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.name = "x";
    field.offset = 0;
    pc2_msg->fields.push_back(field);
    field.name = "y";
    field.offset = 4;
    pc2_msg->fields.push_back(field);
    field.name = "z";
    field.offset = 8;
    pc2_msg->fields.push_back(field);
    // total size of one point: 4 bytes * 3
    pc2_msg->point_step = 12;
    pc2_msg->row_step = pc2_msg->point_step * pc2_msg->width;
    pc2_msg->data.resize(pc2_msg->row_step * pc2_msg->height);
    pc2_msg->is_bigendian = false;
    pc2_msg->is_dense = true;

    // ===================================================================== //
    // get the actual point data
    // ===================================================================== //

    // get the scale in case sound speed is not right in DVL setting
    double scale = sound_speed_ / track_msg->speed_sound;

    // re-generate 3D location of target point
    std::vector<Eigen::Vector3d> points;
    double beam_angle = beam_angle_ * M_PI/180;

    double beam_azimuth[] = {M_PI/4.0, -M_PI/4.0, -3.0*M_PI/4.0, 3.0*M_PI/4.0};
    for (unsigned int i = 0; i < 4; i++) {
        // skip the bad point
        if (track_msg->beam_dist[i] == 0 ) {
            continue;
        }

        Eigen::Vector3d pt;
        pt(0) = 
            track_msg->beam_dist[i] * scale * tan(beam_angle) * cos(beam_azimuth[i]);
        pt(1) = 
            track_msg->beam_dist[i] * scale * tan(beam_angle) * sin(beam_azimuth[i]);
        pt(2) = 
            track_msg->beam_dist[i] * scale;
        points.push_back(pt);
    }

    // ===================================================================== //
    // fill the ros msg
    // ===================================================================== //

    // fill the XYZ
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_x(*pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_y(*pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> ros_pc2_z(*pc2_msg, "z");

    for (unsigned int i = 0; i < pc2_msg->width; i++, 
         ++ros_pc2_x, ++ros_pc2_y, ++ros_pc2_z) {
        const Eigen::Vector3d& point = points.at(i);
        *ros_pc2_x = point(0);
        *ros_pc2_y = point(1);
        *ros_pc2_z = point(2);
    }

    // fill the header
    pc2_msg->header = track_msg->header;
}

void NortekDvlRos::TrackToAltitude(
    const nortek_msgs::msg::NortekDF2::ConstSharedPtr& track_msg, 
    geometry_msgs::msg::PointStamped::SharedPtr& altitude_msg) 
{
    altitude_msg->header = track_msg->header;
    altitude_msg->point.x = 0;
    altitude_msg->point.y = 0;
    altitude_msg->point.z = track_msg->altitude;
}

void NortekDvlRos::ProfileToPressure(
    const nortek_msgs::msg::NortekDF3::ConstSharedPtr& profile_msg, 
    sensor_msgs::msg::FluidPressure::SharedPtr& pressure_msg) 
{
    pressure_msg->header = profile_msg->header;
    pressure_msg->fluid_pressure = (profile_msg->pressure + 1.01325) * 100000;
    //! TODO: accuracy is 0.1% full scale, 
    // For example, a 100 psi gauge with 0.1 % of FS accuracy would be accurate to ± 0.1 psi across its entire range
    // BUT we don't know the full scale of DVL pressure, however, the accuracy is pretty good
    pressure_msg->variance = 0.001;
}

void NortekDvlRos::ProfileToDepthOdom(
    const nortek_msgs::msg::NortekDF3::ConstSharedPtr& profile_msg, 
    nav_msgs::msg::Odometry::SharedPtr& depth_odom_msg)
{
    // header
    depth_odom_msg->header.stamp = profile_msg->header.stamp;
    depth_odom_msg->header.frame_id = world_frame_id_;
    depth_odom_msg->child_frame_id = sensor_frame_id_;
    // convert the pressure (Bar) to depth
    auto depth = (profile_msg->pressure * 100000) / ( fluid_density_ * 9.81);
    // construct the odometry message
    depth_odom_msg->pose.pose.position.x = 0.0;
    depth_odom_msg->pose.pose.position.y = 0.0;
    depth_odom_msg->pose.pose.position.z = -depth;
    depth_odom_msg->pose.covariance[6 * 2 + 2] = depth_cov_;
}

// void NortekDvlRos::ProfileToCells(
//     const nortek_dvl_ethernet::NortekDF3::Ptr& profile_msg, 
//     nav_msgs::GridCells::Ptr& cells_msg)
// {
//     cells_msg->header = profile_msg->header;
// }