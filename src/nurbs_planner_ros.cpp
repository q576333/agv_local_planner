#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>

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
        private_nh.param("pub_fitting_path", pub_fitting_path_, false);
        private_nh.param("use_nurbs", use_nurbs, false);
        private_nh.param("angle_tolerance", yaw_tolerance_, 0.01);
        private_nh.param("dist_tolerance", dist_tolerance_, 0.000001);
        private_nh.param("chord_error", chord_error_, 0.01);
        private_nh.param("use_segment", use_segment_, true);
        private_nh.param("use_limit_fitting", use_limit_fitting_, false);
        private_nh.param("test_limit_fitting", test_limit_fitting_, false);
        if(use_limit_fitting_)
        {
            private_nh.param("start_vector_weight", start_vector_weight_, 2.0);
            private_nh.param("goal_vector_weight", goal_vector_weight_, 2.0);
        }
        if(use_existing_path)
        {
            private_nh.getParam("control_point", input_control_point);
            private_nh.getParam("knot_vector", input_knot_vector);
            private_nh.getParam("weight_vector", weight_vector);
        }

        segment_point_maker_pub = private_nh.advertise<visualization_msgs::MarkerArray>("segment_point", 1, true);
        global_plan_pub = private_nh.advertise<nav_msgs::Path>("existing_plan", 1, true);

        curve_common_ = std::make_unique<Curve_common>();
        curve_fitting_ = std::make_unique<Curve_fitting>();
        trajectory_planner = std::make_unique<NURBSPlanner>(&constraints_, private_nh, use_existing_path);
    }

    bool NURBSPlannerROS::cancel()
    {
        //TODO:Add cancel option
        return false;
    }

    bool NURBSPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > control_point;
        std::vector<geometry_msgs::Point> derivatives_vector;
        double start_point_yaw = 0;
        double goal_point_yaw = 0;

        //Path smoothing in this scope
        if(!use_existing_path)
        {
            fitting_point_eigen_vec = EigenTrajectoryVectorFromVector(plan);
            if(!use_limit_fitting_)
            {
                std::cout << "Using unlimit curve fitting" << "\n";
                spline_inf_ = curve_fitting_->UnLimitCurveFitting(fitting_point_eigen_vec, 3, dis_u_method::Chord, knotvector_method::Average);
            }
            else
            {
                std::cout << "Using limit curve fitting" << "\n";
                start_point_yaw = tf::getYaw(plan.at(0).pose.orientation);
                goal_point_yaw = tf::getYaw(plan.back().pose.orientation);
                derivatives_vector.resize(2);
                derivatives_vector.at(0).x = std::cos(start_point_yaw);
                derivatives_vector.at(0).y = std::sin(start_point_yaw);
                derivatives_vector.at(1).x = std::cos(goal_point_yaw);
                derivatives_vector.at(1).y = std::sin(goal_point_yaw);
                spline_inf_ = curve_fitting_->LimitCurveFitting(fitting_point_eigen_vec, 3, derivatives_vector, dis_u_method::Chord, knotvector_method::LimitDerivative_Average, start_vector_weight_, goal_vector_weight_);
            }
        }
        else
        {
            curve_common_->ReadDiscreate2DPointFromLaunch(&control_point, input_control_point);
            curve_common_->ReadSplineInf(&spline_inf_, 3, control_point, input_knot_vector);
            curve_common_->ReadSplineInf(&spline_inf_, weight_vector, false);       
        }

        if(pub_fitting_path_) 
        {
            existing_plan = curve_common_->Generate_NURBSCurve(spline_inf_, 0.01, "map");
            global_plan_pub.publish(existing_plan);
        }

        if(test_limit_fitting_)
            return false;

        total_length = curve_common_->CalculateCurveLength(spline_inf_, 0.0, 1.0, 50000, use_nurbs);
        //std::cout << "total_length is : " << total_length << "\n";

        curvature_radius = curve_common_->CalculateCurvatureRadius(spline_inf_, 0.3, use_nurbs);

        //TODO: Adding at_max limitation
        //an_max = (8 * radial_error_ / constraints_.sampling_dt_square) - (4 * radial_error_ / curvature_radius * constraints_.sampling_dt_square);
        //std::cout << "an_max is : " << an_max << "\n";
        // an_max = 0.0625;
        // constraints_.a_max = std::sqrt( std::pow(constraints_.system_a_max, 2) - std::pow(an_max, 2));
        //std::cout << "constraints_.a_max is : " << constraints_.a_max << "\n";

        //Trajectory partition and calculate each segment feedrate
        //trajectory_planner->adaptiveFeedrateCurvatureConstraint(spline_inf_, trajectory_segment_vec, 0.002, use_nurbs);
        trajectory_planner->adaptiveFeedrate(spline_inf_, trajectory_segment_vec, chord_error_, 0.002, use_nurbs, use_segment_);
        std::cout << "trajectory segment size is : " << trajectory_segment_vec.size() << "\n";

        //debug test
        visualization_msgs::Marker segment_point_maker;
        segment_point.clear();
        geometry_msgs::Point discreate_segment_point;
        for(int i = 0; i < trajectory_segment_vec.size(); i++)
        {
            if(trajectory_segment_vec.at(i).end_u == 1)
                trajectory_segment_vec.at(i).end_u = 0.999;
            discreate_segment_point = curve_common_->CalculateCurvePoint(&spline_inf_, trajectory_segment_vec.at(i).end_u, use_nurbs);
            segment_point.push_back(discreate_segment_point);
        }
        segment_point_eigen = EigenTrajectoryVectorFromVector(segment_point);
        segment_point_maker = curve_common_->ShowDiscreatePoint2(segment_point_eigen, "map", agv_visualization::Color::Blue(), "segment point", 0.1);
        segment_point_maker_array.markers.push_back(segment_point_maker);
        segment_point_maker_pub.publish(segment_point_maker_array);

        //Preparing for remaining lenth interpolation
        //TODO: writing a function to initial remaining lenth interpolation
        goal_angle = tf::getYaw(plan.back().pose.orientation);
        segment_left_distance = trajectory_segment_vec.at(0).length;
        count = 0;
        accumulate_u = 0;
        final_velocity = 0;
        already_move = 0;
        segment_already_move = 0;
        segment_index = 0;
        rotate_angle = 0;
        retry_times = 0;
        isGetInitialPose = false;
        aligned_orientation = false;
        isReached = false;
        aligned_goalangle = false;
        left_distance = total_length;
        trajectory_planner->setAlreadyMoveLength(0.0);
        trajectory_planner->setSegmentAlreadyMoveLength(0.0);
        trajectory_planner->clearIdealCommandsPath();
        current_velocity = trajectory_segment_vec.at(segment_index).start_velocity;
        final_velocity = trajectory_segment_vec.at(segment_index).final_velocity;
        //std::cout << "trajectory segment size is : " << trajectory_segment_vec.size() << "\n";

        trajectory_initial_angle = trajectory_planner->calculateOriginalAngle(spline_inf_, 0.01, use_nurbs);
        std::cout << "trajectory_initial_angle is : " << trajectory_initial_angle * 180 / M_PI << "\n";

        if(total_length > 0)
            return true;
        else
            return false;
    }

    bool NURBSPlannerROS::isGoalReached(double xy_tolerance, double yaw_tolerance)
    {
        if(aligned_goalangle || use_existing_path)
        {
            if(left_distance > xy_tolerance)
            {            
                return false;
            }
            else
            {
                current_velocity = 0;
                trajectory_planner->publishIdealCommandsPath("map");
                return true;
            }
        }
        else
            return false;
    }

    uint32_t NURBSPlannerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose, const geometry_msgs::TwistStamped& velocity, geometry_msgs::TwistStamped &cmd_vel, std::string &message)
    {
        //Correcting the direction of the AGV
        //TODO: Adding NURBS limit fitting in agv_path_smoothing 
        if(!isReached)
        {
            if(!aligned_orientation)
            {
                robot_pose_yaw = tf::getYaw(pose.pose.orientation);
                rotate_angle = trajectory_planner->normalizeAngel(trajectory_planner->calculateAlignAngle(robot_pose_yaw, trajectory_initial_angle));
                
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
                // if(!isGetInitialPose)
                // {
                //     trajectory_planner->setInitialPose(pose);
                //     isGetInitialPose = true;
                // }

                //Changing segment of the trajectory
                segment_left_distance = trajectory_segment_vec.at(segment_index).length - segment_already_move;
                if(segment_left_distance < dist_tolerance_)
                {
                    segment_index++;
                    segment_left_distance = trajectory_segment_vec.at(segment_index).length;
                    current_velocity = trajectory_segment_vec.at(segment_index).start_velocity;
                    final_velocity = trajectory_segment_vec.at(segment_index).final_velocity;
                    segment_already_move = 0;
                    trajectory_planner->setSegmentAlreadyMoveLength(0);
                    std::cout << "segment: " << segment_index << " complete" << "\n";
                }
                
                //std::cout << "segment final_velocity: " << final_velocity << "\n";
                //std::cout << "segment_left_distance: " << segment_left_distance << "\n";
                // std::cout << "current_velocity in function :" << current_velocity << "\n";
                // std::cout << "final_velocity in function :" << final_velocity << "\n";
                cmd_vel = trajectory_planner->computeSegmentVelocityCommands(pose, spline_inf_, accumulate_u, current_velocity, final_velocity, segment_left_distance, use_nurbs);
                already_move = trajectory_planner->getAlreadymoveLength();

                //Updating segment left distance
                segment_already_move = trajectory_planner->getSegmentAlreadyMoveLength();

                //std::cout << "segment_already_move is :" << segment_already_move << "\n";
                //std::cout << "already_move is :" << already_move << "\n";
                
                //Updating unreached distance of trajectory 
                left_distance = total_length - already_move;
                current_velocity = cmd_vel.twist.linear.z; //TODO: need return new variable

                std::cout << "accumulate_u is :" << accumulate_u << "\n";
                std::cout << "current_velocity is :" << current_velocity << "\n";  
                std::cout << "left_distance is :" << left_distance << "\n";
                if(left_distance < dist_tolerance_ || accumulate_u >= 1)
                {
                    cmd_vel.twist.linear.x = 0;
                    cmd_vel.twist.angular.z = 0;
                    left_distance = 0; 
                    isReached = true;
                    std::cout << "left_distance is reached or accumulate_u bigger than 1"<< "\n";
                }

                if(cmd_vel.twist.linear.x == 0)
                {
                    retry_times++;
                    if(retry_times > 3)
                        return 11;
                }
                
                count++;
            }
        }

        if(isReached && !aligned_goalangle && !use_existing_path)
        {
            robot_pose_yaw = tf::getYaw(pose.pose.orientation);
            rotate_angle = trajectory_planner->normalizeAngel(trajectory_planner->calculateAlignAngle(robot_pose_yaw, goal_angle));
            
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
                aligned_goalangle = true;
                std::cout << "Already rotated to goal angle" << "\n";
            }
        }

        //TODO: Adding compute velocity command faild situation
        //return mbf_msgs::ExePathResult::SUCCESS;
        return 0;
    }
}