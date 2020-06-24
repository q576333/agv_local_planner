#ifndef AGV_LOCAL_PLANNER_LOCAL_NURBS_PLANNER_H_
#define AGV_LOCAL_PLANNER_LOCAL_NURBS_PLANNER_H_

#include "agv_path_smoothing/Curve_common.h"
#include "agv_path_smoothing/Curve_fitting.h"
#include "agv_path_smoothing/conversion.h"
#include "agv_local_planner/physical_constraints.h"

#include <geometry_msgs/TwistStamped.h>

namespace nurbs_local_planner {
    
    //Prehead use
    struct AdativeFeedrateSeg
    {
        int index;
        double end_u;
        double length;
        double start_velocity;
        double final_velocity;
    };

    class NURBSPlanner
    {
        public:
            NURBSPlanner(PhysicalConstraints *ref_constraint, ros::NodeHandle private_nh);
            ~NURBSPlanner();

            double getSegmentAlreadyMoveLength();
            void setSegmentAlreadyMoveLength(double value);
            void setAlreadyMoveLength(double value);
            double getAlreadymoveLength();
            void publishIdealCommandsPath(std::string frame_id);
            void adativeFeedrateCurvatureConstraint(Spline_Inf spline_inf, std::vector<AdativeFeedrateSeg>& trajectory_seg, double delta_u, bool UsingNURBS);
            void setInitialPose(geometry_msgs::PoseStamped pose);
            void assignVelocity(geometry_msgs::TwistStamped &cmd_vel, double linear_velocity, double angular_velocity);
            double calculateAlignAngle(double robot_yaw, double trajectory_initial_angle);
            double calculateOriginalAngle(Spline_Inf spline_inf, double u_data, bool UsingNURBS);
            double normalizeAngel(double ang_diff);
            geometry_msgs::TwistStamped computeSegmentVelocityCommands(geometry_msgs::PoseStamped robot_pose, Spline_Inf spline_inf, double &u_data, double current_velocity, double final_velocity, double left_distance, bool UsingNURBS);
            geometry_msgs::TwistStamped computeAlignOrientationCommand(double align_angle, double angular_vel);
        
        private:           
            double computeAngularVelocity(Spline_Inf spline_inf, double &u_data, double delta_u, bool UsingNURBS);
            double computeAngularVelocityMethod2(Spline_Inf spline_inf, double &u_data, double delta_u, bool UsingNURBS); 
            bool checkInitialdirection(Spline_Inf spline_inf, bool UsingNURBS);
            bool checkdirection(Spline_Inf spline_inf,double u_data, double next_u_data, bool UsingNURBS, bool direction);
            double calculateIdealLength(Spline_Inf spline_inf, double u_data, double next_u_data, bool UsingNURBS);
            double calculateDeltaU(Spline_Inf spline_inf, double u_data, double suggested_velocity, int taylor_order, int compensate_times, bool UsingNURBS);
            inline double calculateRootMeanSquareError(double value1, double value2);
            void assignIdealCommandPose(double x, double y);
            
            double arc_length;
            bool inverse_direction = false;
            //TODO: wait code refactor
            bool initialed = false;
            bool AngularVelocity2_initialized = false;
            geometry_msgs::PoseStamped initial_pose;

            //For evaluation use
            ros::Publisher ideal_commands_path_pub_;
            nav_msgs::Path ideal_commands_path;
            int command_index;
            double sum_ideal_theta;
            double original_angle;

            //Parameter of calculate angular velocity 
            Eigen::Vector3d eigen_curve_point_old;
            Eigen::Vector3d eigen_curve_point;
            Eigen::Vector3d agv_direction_vector;
            Eigen::Vector3d agv_direction_vector_old;
            double ideal_theta;

            //Parameter of calculate linear velocity 
            double already_move_distance;
            double segment_already_move_distance;

            std::unique_ptr<Curve_common> pCurve_common;
            PhysicalConstraints *constraint;
    };
};

#endif //AGV_LOCAL_PLANNER_LOCAL_NURBS_PLANNER_H_