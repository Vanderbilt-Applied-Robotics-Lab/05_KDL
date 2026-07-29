#include <inv_kin/inv_kin_node.h>

InvKinNode::InvKinNode() : Node("inv_kin_node")
{
    // setup subscriber for robot description
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
        "robot_description", 
        rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
        std::bind(&InvKinNode::robotDescriptionCallback, this, std::placeholders::_1));

    // publisher for the joint_states object
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
        
    // setup poses
    poses_[0] = KDL::Frame(
        KDL::Rotation::Quaternion(0.3535, 0.1464, 0.3536, 0.8536),
        KDL::Vector(-0.403789, 0.120946, 0.371044)
    ); 
    poses_[1] = KDL::Frame(
        KDL::Rotation::Quaternion(0.7071, 0, 0, 0.7071),
        KDL::Vector(-0.200, 0.570, 0)
    );

    joint_state_msg_.name = {"joint1", "joint2", "joint3"};
    joint_state_msg_.position.resize(joint_state_msg_.name.size());
}
bool InvKinNode::solveNextPose()
{
    solveIK(poses_[pose_index_]);

    // reset if at max value
    if (pose_index_ >= poses_.size())
        return false;
    else
        return true;
}

void InvKinNode::robotDescriptionCallback(const std_msgs::msg::String& msg)
{
    // Construct KDL tree from URDF
    const std::string urdf = msg.data;
    kdl_parser::treeFromString(urdf, tree_);

    // Get kinematic chain of the robot
    tree_.getChain("world", "ee_link", chain_);

    q_ = KDL::JntArray(chain_.getNrOfJoints());

    // Create IK solver
    ik_solver_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(chain_);
}

void InvKinNode::solveIK(const KDL::Frame & frame)
{
    if (ik_solver_ != nullptr)
    {
        RCLCPP_INFO(this->get_logger(), "Trying pose number %i", pose_index_);
        
        // initial guess of IK solution
        KDL::JntArray q_init(chain_.getNrOfJoints());
        q_init(0) = 0.0; q_init(1) = 0.0; q_init(2) = 0.0;

        // solve IK 
        int error = ik_solver_->CartToJnt(q_init, frame, q_);

        if (error == KDL::ChainIkSolverPos_LMA::E_NOERROR) // IK succeeded
        {
            RCLCPP_INFO(this->get_logger(), "IK succeeded");

            // set the header time stamp
            joint_state_msg_.header.stamp = this->get_clock()->now();
            
            // copy the joint angles 
            std::copy(q_.data.begin(), q_.data.end(), joint_state_msg_.position.begin());

            // publish
            joint_pub_->publish(joint_state_msg_);

        }
        else // IK failed
        {
            RCLCPP_ERROR(this->get_logger(), "Inverse kinematics failed!");
        }

        // increase the index (only if it worked)
        ++pose_index_;
    }
}

int main(int argc, char** argv)
{
    // initialize the node
    rclcpp::init(argc, argv);
    
    // create instance of class
    auto node = std::make_shared<InvKinNode>();

    // Set loop rate
    rclcpp::Rate rate = rclcpp::Rate(1); // Hz
    
    while (node->solveNextPose())
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}