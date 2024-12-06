#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <functional>
#include <array>
#include <string>


#include <ros/ros.h>
#include <std_msgs/String.h>


class UDPSocketHandler {
public:
    // Update DataCallback to take a char array and its size
    using DataCallback = std::function<void(const char*, std::size_t)>;    

    UDPSocketHandler(unsigned short port) : io_service_(), socket_(io_service_) {
        // Initialize the socket
        boost::asio::ip::udp::endpoint endpoint(boost::asio::ip::udp::v4(), port);
        socket_.open(endpoint.protocol());
        socket_.bind(endpoint);

        recv_buffer_.resize(1024);

        // Start async receive
        startAsyncReceive();

        // Start io_service in a separate thread
        io_thread_ = boost::thread([this]() { io_service_.run(); });
    }

    ~UDPSocketHandler() {
        io_service_.stop();
        if (io_thread_.joinable()) {
            io_thread_.join();
        }
    }

    void setDataCallback(DataCallback callback) {
        data_callback_ = callback;
    }

private:
    void startAsyncReceive() {

        recv_buffer_.assign(recv_buffer_.size(), 0);

        socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            [this](const boost::system::error_code& error, std::size_t bytes_transferred) {
                if (!error) {

                    std::ostringstream oss;
                    for (size_t i = 0; i < bytes_transferred; ++i) {
                        oss << std::hex << std::setfill('0') << std::setw(2)
                            << static_cast<int>(static_cast<unsigned char>(recv_buffer_[i])) << " ";
                    }
                    std::cout<< "Received raw data (hex): " << oss.str() <<std::endl;

                    // Invoke callback with raw data and size
                    if (data_callback_) {
                        data_callback_(recv_buffer_.data(), bytes_transferred);
                    }


                    // Start next async receive
                    startAsyncReceive();
                }
            });
    }

    boost::asio::io_service io_service_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint remote_endpoint_;
    boost::thread io_thread_;
    // std::array<char, 1024> recv_buffer_;
    std::vector<char> recv_buffer_;
    DataCallback data_callback_;
};


class ROSNode {
public:
    ROSNode(unsigned short port) : socket_handler_(port), nh_("~") {
        // Initialize ROS publisher
        data_pub_ = nh_.advertise<std_msgs::String>("received_data", 10);
        sub_ = nh_.subscribe<std_msgs::String> ("/test/sub", 1, &ROSNode::callback, this);

        // Set up callback for received data from the socket
        socket_handler_.setDataCallback(
            [this](const char* data, std::size_t size) { this->handleReceivedData(data, size); });            
    }

private:
    void handleReceivedData(const char* data, std::size_t size) {
        // Convert char array to string for publishing
        std::string data_str(data, size);

        // Create and publish ROS message
        std_msgs::String msg;
        msg.data = data_str;
        data_pub_.publish(msg);

        ROS_INFO_STREAM("Published data: " << data_str);
    }

    void callback(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("%s: Recv: %s", ros::this_node::getName().c_str(), msg->data.c_str());
    }

    ros::NodeHandle nh_;
    ros::Publisher data_pub_;
    ros::Subscriber sub_;
    UDPSocketHandler socket_handler_; // UDP socket instance
};


int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "udp_to_ros_node");

    // Create ROSNode with desired UDP port
    unsigned short udp_port = 9004;
    ROSNode node(udp_port);

    // Use ROS spinner to handle callbacks
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Keep the main thread alive
    ros::waitForShutdown();

    return 0;
}
