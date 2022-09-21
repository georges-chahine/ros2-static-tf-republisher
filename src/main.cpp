#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std::chrono_literals;
class FramePublisher : public rclcpp::Node
{

private:

    bool init;
    geometry_msgs::msg::TransformStamped get_pose_tf(std::string source_frame, std::string target_frame){

        geometry_msgs::msg::TransformStamped transformStamped;

        rclcpp::Time now = this->get_clock()->now();
        try {
            transformStamped = tf_buffer_->lookupTransform(
                        target_frame, source_frame, tf2::TimePointZero); //0, 1ms);
        }
        catch (tf2::TransformException & ex) {
            RCLCPP_INFO(
                        this->get_logger(), "Could not transform %s to %s: %s",
                        source_frame.c_str(), target_frame.c_str(), ex.what());

        }
        transformStamped.header.stamp = now;
        return transformStamped;
    }

    bool is_identity(geometry_msgs::msg::TransformStamped t){

        Eigen::Matrix4d test=Eigen::Matrix4d::Identity();

        test(0,3)=t.transform.translation.x;
        test(1,3)=t.transform.translation.y;
        test(2,3)=t.transform.translation.z;

        Eigen::Quaterniond q(t.transform.rotation.w , t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z);

        Eigen::Matrix3d R(q);

        test.block(0,0,3,3)=R;

        if (test.isIdentity()){
            return true;
        }
        else{
            std::cout <<test<<std::endl;
            return false;
        }
    }

    void publish_tf(std::string tp0, std::string tp1, std::string tp2)
    {

        geometry_msgs::msg::TransformStamped t01, t02;

        while (!init){
            t01=get_pose_tf(tp1, tp0);
            t02=get_pose_tf(tp2, tp0);

            if (! is_identity(t01) && !is_identity(t02))
            {
                std::cout<<init<<std::endl;
                init=true;

            }
            else
            {
                 std::cout<<init<<std::endl;
                init=false;
            }

        }


         t01.child_frame_id=t01.child_frame_id+"_1";
         t02.child_frame_id=t02.child_frame_id+"_1";
        while (true){

            tf_broadcaster_->sendTransform(t01);
            tf_broadcaster_->sendTransform(t02);
            rclcpp::sleep_for(std::chrono::nanoseconds(250ms));

        }
    }
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string turtlename_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


public:
    FramePublisher()
        : Node("static_frame_republisher")
    {
        init=false;
        // Declare and acquire `turtlename` parameter
        //turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");

        // Initialize the transform broadcaster
        tf_broadcaster_ =
                std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        // callback function on each message
        //std::ostringstream stream;
        //stream << "/" << turtlename_.c_str() << "/pose";
        //std::string topic_name = stream.str();
        rclcpp::TimerBase::SharedPtr timer_{nullptr};
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publish_tf("base_link","cloud","virtual_gps");


    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}
