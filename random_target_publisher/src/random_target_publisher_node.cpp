#include <random>  
#include <chrono>  
#include <thread>  
#include <rclcpp/rclcpp.hpp>  
#include <geometry_msgs/msg/pose.hpp>  
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
// #include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
  
using namespace std::chrono_literals;  
enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};
  
class RandomTargetPublisher : public rclcpp::Node  
{  
public:  
    RandomTargetPublisher()  
    : Node("random_target_publisher") 
    {  
        RCLCPP_INFO(this->get_logger(), "pub rand goal");
        timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&RandomTargetPublisher::timer_callback, this));
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    }  
    
  
private: 
    void timer_callback()
        {
            // RCLCPP_INFO(this->get_logger(), "goal index:%d",goal_index);
            // RCLCPP_INFO(this->get_logger(), "goal status:%d",static_cast<int>(current_goal_status_));
            if(static_cast<int>(current_goal_status_) == 3){
                goal_index = (goal_index + 1) % 4;
                current_goal_status_ = ActionStatus::UNKNOWN;
            }

            // double s[4][2]={
            //     {7.0, 6.5},
            //     {7.0, 9.5},
            //     {3.0, 7.0},
            //     {3.0, 9.0}};
            
            double s[4][2]={
                {1.0, 7.5},
                {7.0, 7.5},
                {1.0, 7.5},
                {7.0, 7.5}};
            
            if(goal_index < 4){
                auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
                goal_msg.pose.header.frame_id = "map";
                goal_msg.pose.header.stamp = this->now();
                goal_msg.pose.pose.position.x = s[goal_index][0];
                goal_msg.pose.pose.position.y = s[goal_index][1];
                goal_msg.behavior_tree = "/workspaces/multi_robots/src/capella_ros_launcher/navigate_to_pose_w_replanning_and_recovery.xml";
                // action_client_->async_send_goal(goal_msg);
                auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
                send_goal_options.result_callback =
                    std::bind(&RandomTargetPublisher::resultCallback, this, std::placeholders::_1);
                send_goal_options.goal_response_callback =
                    std::bind(&RandomTargetPublisher::goalResponseCallback, this, std::placeholders::_1);
    
                // using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
                
                action_client_->async_send_goal(goal_msg, send_goal_options);
            }
        }
    void resultCallback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            current_goal_status_ = ActionStatus::SUCCEEDED;
            return;
            case rclcpp_action::ResultCode::ABORTED:
            current_goal_status_ = ActionStatus::FAILED;
            return;
            case rclcpp_action::ResultCode::CANCELED:
            current_goal_status_ = ActionStatus::FAILED;
            return;
            default:
            current_goal_status_ = ActionStatus::UNKNOWN;
            return;
        }
    }

    void goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal)
    {
        if (!goal) {
            RCLCPP_ERROR(
            get_logger(),
            "navigate_to_pose action client failed to send goal to server.");
            current_goal_status_ = ActionStatus::FAILED;
        }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    int goal_index = 0;
    double robot_x,robot_y;
    ActionStatus current_goal_status_;
    
    // 声明action客户端
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
};  
  
int main(int argc, char * argv[])  
{  
    rclcpp::init(argc, argv);  
    auto node = std::make_shared<RandomTargetPublisher>();  
    rclcpp::spin(node);  
    rclcpp::shutdown();  
    return 0;  
}