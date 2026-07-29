
#ifndef INV_KIN_NODE
#define INV_KIN_NODE

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>

/**
 * Solution for take-home assignment 5 
 * Applied Robotics
 */
class InvKinNode : public rclcpp::Node
{
public:
    /**
     * Constructor
     */
    InvKinNode();

    /**
     * Destructor
     */
    ~InvKinNode() = default;

    /**
     * Solve the inverse kinematics of the next pose in the pose list
     * @return returns true if there is another pose to solve 
     */
    bool solveNextPose();
    
private:
    /**
     * Solves the IK for a given frame
     * @param frame The ee frame to try and solve the inverse kinematics for
     */
    void solveIK(const KDL::Frame & frame);
    
    /**
     * Callback for the /robot_description topic. Initializes the IK solver
     * @param msg the message on the /robot_description topic
     */
    void robotDescriptionCallback(const std_msgs::msg::String& msg);

    // subscriber for the robot description
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    sensor_msgs::msg::JointState joint_state_msg_;

    // KDL
    KDL::Tree tree_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    KDL::JntArray q_; 

    // Frames
    std::array<KDL::Frame, 2> poses_;
    uint8_t pose_index_ = 0;
};

#endif // INV_KIN_NODE