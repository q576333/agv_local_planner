#include <pluginlib/class_list_macros.h>

#include "agv_local_planner/nurbs_planner_ros.h"
//#include "mbf_msgs/ExePathResult.h"

//register this planner as a move_base_flex CostmapController plugin
PLUGINLIB_EXPORT_CLASS(nurbs_local_planner::NURBSPlannerROS, mbf_costmap_core::CostmapController);

namespace nurbs_local_planner{

    void NURBSPlannerROS::initialize(std::string name, TF *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        ros::NodeHandle private_nh("~/" + name);
        constraints_.setParametersFromRos(private_nh);
        private_nh.param("use_existing_path", use_existing_path, false);
        private_nh.param("pub_existing_path", pub_existing_path, false);
        private_nh.param("use_nurbs", use_nurbs, false);
        private_nh.param("angle_tolerance", yaw_tolerance_, 0.01);
        private_nh.param("dist_tolerance", dist_tolerance_, 0.000001);
        private_nh.param("radial_error", radial_error_, 0.01);
        private_nh.param("use_segment", use_segment_, true);
        if(use_existing_path)
        {
            global_plan_pub = private_nh.advertise<nav_msgs::Path>("existing_plan", 1, true);
            private_nh.getParam("control_point", input_control_point);
            private_nh.getParam("knot_vector", input_knot_vector);
            private_nh.getParam("weight_vector", weight_vector);
        }
        curve_common_ = std::make_unique<Curve_common>();
        curve_fitting_ = std::make_unique<Curve_fitting>();
        trajectory_planner = std::make_unique<NURBSPlanner>(&constraints_);
    }

    bool NURBSPlannerROS::cancel()
    {
        //TODO:Add cancel option
        return false;
    }

    bool NURBSPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > control_point;

        //Path smoothing in this scope
        if(!use_existing_path)
        {
            fitting_point_eigen_vec = EigenTrajectoryVectorFromVector(plan);
            spline_inf_ = curve_fitting_->UnLimitCurveFitting(fitting_point_eigen_vec, 3, dis_u_method::Chord, knotvector_method::Average);
        }
        else
        {
            curve_common_->ReadDiscreate2DPointFromLaunch(&control_point, input_control_point);
            curve_common_->ReadSplineInf(&spline_inf_, 3, control_point, input_knot_vector);
            curve_common_->ReadSplineInf(&spline_inf_, weight_vector);       
            if(pub_existing_path)
            {
                existing_plan = curve_common_->Generate_NURBSCurve(spline_inf_, 0.01, "map");
                global_plan_pub.publish(existing_plan);
            }
        }
        total_length = curve_common_->CalculateCurveLength(spline_inf_, 0.0, 1.0, 50000, use_nurbs);
        //std::cout << "total_length is : " << total_length << "\n";

        curvature_radius = curve_common_->CalculateCurvatureRadius(spline_inf_, 0.3, use_nurbs);

        //TODO: Adding at_max limitation
        //an_max = (8 * radial_error_ / constraints_.sampling_dt_squre) - (4 * radial_error_ / curvature_radius * constraints_.sampling_dt_squre);
        //std::cout << "an_max is : " << an_max << "\n";
        // an_max = 0.0625;
        // constraints_.a_max = std::sqrt( std::pow(constraints_.system_a_max, 2) - std::pow(an_max, 2));
        //std::cout << "constraints_.a_max is : " << constraints_.a_max << "\n";

        //Trajectory partition and calculate each segment feedrate
        trajectory_planner->adativeFeedrate(spline_inf_, trajectory_segment_vec, 0.002, use_nurbs);

        //Preparing for remaining lenth interpolation
        //TODO: writing a function to initial remaining lenth interpolation
        segment_left_distance = trajectory_segment_vec.at(0).length;
        count = 0;
        accumulate_u = 0;
        final_velocity = 0;
        already_move = 0;
        segment_already_move = 0;
        segment_index = 0;
        rotate_angle = 0;
        aligned_orientation = false;
        left_distance = total_length;
        trajectory_planner->setAlreadyMoveLength(0.0);
        trajectory_planner->setSegmentAlreadyMoveLength(0.0);
        //std::cout << "trajectory segment size is : " << trajectory_segment_vec.size() << "\n";

        if(total_length > 0)
            return true;
        else
            return false;
    }

    bool NURBSPlannerROS::isGoalReached(double xy_tolerance, double yaw_tolerance)
    {
        if(left_distance > xy_tolerance)
        {            
            return false;
        }
        else
        {
            current_velocity = 0;
            return true;
        }
            
    }

    uint32_t NURBSPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity, geometry_msgs::TwistStamped &cmd_vel, std::string &message)
    {
        //Correcting the direction of the AGV
        //TODO: Adding NURBS limit fitting in agv_path_smoothing 
        if(!aligned_orientation)
        {
            robot_pose_yaw = tf::getYaw(pose.pose.orientation);
            rotate_angle = trajectory_planner->normalizeAngel(trajectory_planner->calculateAlignAngle(robot_pose_yaw, spline_inf_, 0.01, use_nurbs));
            
            if(std::abs(rotate_angle) > yaw_tolerance_)
            {
                if(rotate_angle < 0)
                    trajectory_planner->assignVelocity(cmd_vel, 0.0, 0.15);
                else
                    trajectory_planner->assignVelocity(cmd_vel, 0.0, -1 * 0.15);
            }
            else
            {
                trajectory_planner->assignVelocity(cmd_vel, 0.0, 0.0);         
                aligned_orientation = true;
                std::cout << "Already rotated" << "\n";
            }
        }
        else
        {
            //Time index of the sampling time (debug use)
            ROS_INFO_STREAM("------------ Count :" << count << "------------"); 

            //TODO: Confirm if we need agv initial pose to compute angular velocity 
            // if(!isInitialized)
            // {
            //     trajectory_planner->setInitialPose(pose);
            //     isInitialized = true;
            // }

            //Changing segment of the trajectory
            segment_left_distance = trajectory_segment_vec.at(segment_index).length - segment_already_move;
            if(segment_left_distance < dist_tolerance_)
            {
                segment_index++;
                segment_left_distance = trajectory_segment_vec.at(segment_index).length;
                segment_already_move = 0;
                trajectory_planner->setSegmentAlreadyMoveLength(0);
                std::cout << "segment: " << segment_index << " complete" << "\n";
            }
            
            final_velocity = trajectory_segment_vec.at(segment_index).final_velocity;
            //std::cout << "segment final_velocity: " << final_velocity << "\n";
            //std::cout << "segment_left_distance: " << segment_left_distance << "\n";
            cmd_vel = trajectory_planner->computeSegmentVelocityCommands(pose, spline_inf_, accumulate_u, current_velocity, final_velocity, segment_left_distance, use_nurbs);
            already_move = trajectory_planner->getAlreadymoveLength();

            //Updating segment left distance
            segment_already_move = trajectory_planner->getSegmentAlreadyMoveLength();

            //std::cout << "segment_already_move is :" << segment_already_move << "\n";
            //std::cout << "already_move is :" << already_move << "\n";
            
            //Updating unreached distance of trajectory 
            left_distance = total_length - already_move;
            current_velocity = cmd_vel.twist.linear.x;

            if(left_distance < dist_tolerance_ || accumulate_u >= 1)
            {
                cmd_vel.twist.linear.x = 0;
                cmd_vel.twist.angular.z = 0;
                left_distance = 0; 
            }

            std::cout << "accumulate_u is :" << accumulate_u << "\n";
            std::cout << "current_velocity is :" << current_velocity << "\n";  
            std::cout << "left_distance is :" << left_distance << "\n";
            count++;
        }

        //TODO: Adding compute velocity command faild situation
        //return mbf_msgs::ExePathResult::SUCCESS;
        return 0;
    }
}