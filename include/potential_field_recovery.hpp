#ifndef POTENTIAL_FIELD_RECOVERY__POTENTIAL_FIELD_RECOVERY_HPP_
#define POTENTIAL_FIELD_RECOVERY__POTENTIAL_FIELD_RECOVERY_HPP_



#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include "nav2_recoveries/recovery.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "potential_field_recovery/action/potentialfield.hpp"

#include <angles/angles.h>


namespace potential_field_recovery{

    using namespace nav2_recoveries;
    using Action = potential_field_recovery::action::Potentialfield;

    class PotentialFieldRecovery : public Recovery<Action> 
    {

        public:
            
            PotentialFieldRecovery();
            ~PotentialFieldRecovery();

            void onConfigure() override;
            Status onRun(const std::shared_ptr<const Action::Goal> command) override; 
            Status onCycleUpdate() override;
        
        
            // std::shared_ptr<tf2_ros::Buffer> tf_;

            //nav2_util::LifecycleNode::SharedPtr node_;

            // nav2_costmap_2d::Costmap2DROS * costmap_;

            // std::string name_;
            std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
            

            bool initialized_ = false;

            double sim_granularity_, max_rotational_vel_, max_trans_vel_, acc_lim_th_, tolerance_, frequency_;

            double min_dist_;


    };


}

#endif