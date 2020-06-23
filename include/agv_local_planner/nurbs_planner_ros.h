#ifndef AGV_LOCAL_PLANNER_LOCAL_NURBS_PLANNER_ROS_H_
#define AGV_LOCAL_PLANNER_LOCAL_NURBS_PLANNER_ROS_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <mbf_utility/types.h>
#include <mbf_costmap_core/costmap_controller.h>

#include <memory>

#include "agv_local_planner/nurbs_planner.h"

namespace nurbs_local_planner {

    class NURBSPlannerROS : public mbf_costmap_core::CostmapController
    {
        public:
            /**
            * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands
            * to send to the base.
            * @param pose the current pose of the robot.
            * @param velocity the current velocity of the robot.
            * @param cmd_vel Will be filled with the velocity command to be passed to the robot base.
            * @param message Optional more detailed outcome as a string
            * @return Result code as described on ExePath action result:
            *         SUCCESS         = 0
            *         1..9 are reserved as plugin specific non-error results
            *         FAILURE         = 100   Unspecified failure, only used for old, non-mfb_core based plugins
            *         CANCELED        = 101
            *         NO_VALID_CMD    = 102
            *         PAT_EXCEEDED    = 103
            *         COLLISION       = 104
            *         OSCILLATION     = 105
            *         ROBOT_STUCK     = 106
            *         MISSED_GOAL     = 107
            *         MISSED_PATH     = 108
            *         BLOCKED_PATH    = 109
            *         INVALID_PATH    = 110
            *         TF_ERROR        = 111
            *         NOT_INITIALIZED = 112
            *         INVALID_PLUGIN  = 113
            *         INTERNAL_ERROR  = 114
            *         121..149 are reserved as plugin specific errors
            */
            uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                                const geometry_msgs::TwistStamped& velocity,
                                                geometry_msgs::TwistStamped &cmd_vel,
                                                std::string &message);

            /**
             * @brief Check if the goal pose has been achieved by the local planner within tolerance limits
             * @remark New on MBF API
             * @param xy_tolerance Distance tolerance in meters
             * @param yaw_tolerance Heading tolerance in radians
             * @return True if achieved, false otherwise
             */
            bool isGoalReached(double xy_tolerance, double yaw_tolerance);

            /**
             * @brief  Set the plan that the local planner is following
             * @param plan The plan to pass to the local planner
             * @return True if the plan was updated successfully, false otherwise
             */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

            /**
             * @brief Requests the planner to cancel, e.g. if it takes too much time
             * @remark New on MBF API
             * @return True if a cancel has been successfully requested, false if not implemented.
             */
            bool cancel();

            /**
             * @brief Constructs the local planner
             * @param name The name to give this instance of the local planner
             * @param tf A pointer to a transform listener
             * @param costmap_ros The cost map to use for assigning costs to local plans
             */
            void initialize(std::string name, TF *tf, costmap_2d::Costmap2DROS *costmap_ros);

        private:
            PhysicalConstraints constraints_;
            // NURBSPlanner trajectory_planner;
            // Curve_common curve_common_;
            // Curve_fitting curve_fitting_;
            std::unique_ptr<NURBSPlanner> trajectory_planner;
            std::unique_ptr<Curve_common> curve_common_;
            std::unique_ptr<Curve_fitting> curve_fitting_;

            bool use_existing_path;
            bool pub_existing_path;
            bool use_nurbs;
            ros::Publisher global_plan_pub;
            nav_msgs::Path existing_plan;
            std::vector<double> input_control_point;
            std::vector<double> input_knot_vector;
            std::vector<double> weight_vector;

            std::vector<AdativeFeedrateSeg> trajectory_segment_vec;
            double segment_left_distance;
            double segment_already_move;
            int segment_index;

            int count = 1; //debug use
            bool aligned_orientation = false;
            double rotate_angle; //need to rotate angle
            double trajectory_initial_angle;
            double robot_pose_yaw;
            double yaw_tolerance_;
            double dist_tolerance_;

            double radial_error_;
            double an_max;
            double curvature_radius;
            bool use_segment_ = false;

            bool isInitialized = false;
            double total_length;
            double accumulate_u;
            double current_velocity;
            double final_velocity;
            double left_distance;
            double already_move;
            EigenTrajectoryPoint::Vector fitting_point_eigen_vec;
            Spline_Inf spline_inf_;
    };
};

#endif //AGV_LOCAL_PLANNER_LOCAL_NURBS_PLANNER_ROS_H_