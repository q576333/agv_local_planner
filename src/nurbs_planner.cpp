#include <agv_local_planner/nurbs_planner.h>
#include <math.h>

namespace nurbs_local_planner{
    
    NURBSPlanner::NURBSPlanner(PhysicalConstraints *ref_constraint, ros::NodeHandle private_nh)
    {
        pCurve_common = std::make_unique<Curve_common>();
        constraint = ref_constraint;
        ideal_commands_path_pub_ = private_nh.advertise<nav_msgs::Path>("ideal_commands_path", 1, true);
        command_index = 0;
        sum_ideal_theta = 0;
    }

    NURBSPlanner::~NURBSPlanner()
    {

    }

    void NURBSPlanner::assignVelocity(geometry_msgs::TwistStamped &cmd_vel, double linear_velocity, double angular_velocity)
    {
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.twist.linear.x = linear_velocity;
        cmd_vel.twist.linear.y = 0.0;
        cmd_vel.twist.linear.z = 0.0;
        cmd_vel.twist.angular.x = 0.0;
        cmd_vel.twist.angular.y = 0.0;
        cmd_vel.twist.angular.z = angular_velocity;
    }

    geometry_msgs::TwistStamped NURBSPlanner::computeSegmentVelocityCommands(geometry_msgs::PoseStamped robot_pose, Spline_Inf spline_inf, double &u_data, double current_velocity, double final_velocity, double left_distance, bool UsingNURBS)
    {
        geometry_msgs::TwistStamped cmd_vel_;
        double update_suggest_velocity = 0;
        double suggest_velocity = 0;
        double suggest_accleration = 0;
        double right_term = 0;
        double angular_velocity = 0;
        double delta_u = 0;
        double ideal_length = 0;
        double update_delta_u = 0;
        double next_u_data = 0;
        double left_accleration_times = 0;
        double left_time_stop = 0;

        //ideal command path use
        double ideal_delta_pose_x = 0;
        double ideal_delta_pose_y = 0;

        //Computing suggest_velocity
        right_term = std::sqrt(std::pow(final_velocity, 2) + 0.25 * constraint->a_max_squre * constraint->sampling_dt_squre + constraint->a_max * (2 * left_distance - current_velocity * constraint->sampling_dt));
        suggest_velocity = -0.5 * constraint->a_max * constraint->sampling_dt + right_term;

        if(std::isnan(right_term))
        {
            std::cout << "suggest_velocity is nan" << "\n";
            suggest_velocity = left_distance / constraint->sampling_dt;
            segment_already_move_distance += left_distance;
            already_move_distance += left_distance;
            assignVelocity(cmd_vel_, suggest_velocity, 0.0);
            return cmd_vel_;
        }
            
        //Check suggest_velocity corresponding to some limitation
        if(suggest_velocity < final_velocity)
        {
            std::cout << "Error, left_distance < 0, this should never happen!" << "\n";
            suggest_velocity = final_velocity;
        }

        if(suggest_velocity < 0)
        {
            std::cout << "Error, suggested_velocity < 0, this should never happen!" << "\n";
            assignVelocity(cmd_vel_, 0, 0);
            return cmd_vel_;
        }
        else
        {
            if(suggest_velocity > constraint->v_max)
            {
                suggest_velocity = constraint->v_max;
                //std::cout << "constrainted suggest_velocity:" << suggest_velocity << "\n";
            }
        }    
        //std::cout << "original suggest_velocity is : " << suggest_velocity << "\n";

        //Computing suggest_accleration
        suggest_accleration = (suggest_velocity - current_velocity) / constraint->sampling_dt;

        //Adjusting accleration maximum let agv in accleration and deceleration part more smooth
        if(suggest_accleration >= 0)
        {
            left_accleration_times = std::floor((constraint->v_max - current_velocity) / (constraint->a_max * constraint->sampling_dt) + 0.5);
            
            if(left_accleration_times < 1)
                left_accleration_times = 1;

            constraint->adaptive_a_max = (constraint->v_max - current_velocity) / (left_accleration_times * constraint->sampling_dt);
        }
        else
        {
            left_time_stop = left_distance / (0.5 * (current_velocity - final_velocity) + final_velocity);
            constraint->a_max = (current_velocity - final_velocity) / left_time_stop;
            left_accleration_times = std::floor((current_velocity - final_velocity) / (constraint->adaptive_a_max * constraint->sampling_dt));
            
            if(left_accleration_times < 1)
                left_accleration_times = 1;

            constraint->adaptive_a_max = (current_velocity - final_velocity) / (left_accleration_times * constraint->sampling_dt);
        }

        //Check suggest_accleration corresponding to some limitation
        if(suggest_accleration > 0)
        {          
            std::cout << "----------Now, agv is acclerating----------" << "\n";
            if(suggest_accleration > constraint->adaptive_a_max)
            {
                suggest_accleration = constraint->adaptive_a_max;  
                //TODO: show sampling time index
                suggest_velocity = current_velocity + suggest_accleration * constraint->sampling_dt;
            }
        }
        if(suggest_accleration < 0)
        {
            std::cout << "----------Now, agv is slow down----------" << "\n";
            if(suggest_accleration < -1 * constraint->adaptive_a_max)
            {
                suggest_accleration = -1 * constraint->adaptive_a_max;  
                //TODO: show sampling time index
                suggest_velocity = current_velocity + suggest_accleration * constraint->sampling_dt;
            }
        }

        //std::cout << "before smoothing suggest_velocity is : " << suggest_velocity << "\n";
        //std::cout << "current_velocity is : " << current_velocity << "\n";

        //--------- test use ideal_length to update left distance ---------       
        update_suggest_velocity = (suggest_velocity + current_velocity) * 0.5; //for smoothing
        // if(update_suggest_velocity > constraint->v_max)
        //     update_suggest_velocity = constraint->v_max;

        // std::cout << "accleration constrained suggest_velocity is : " << suggest_velocity << "\n";
        // std::cout << "accleration constrained update_suggest_velocity is : " << update_suggest_velocity << "\n";
        // std::cout << "suggest_accleration is : " << suggest_accleration << "\n";

        delta_u = calculateDeltaU(spline_inf, u_data, update_suggest_velocity, 1, 3, UsingNURBS);
        next_u_data = u_data + delta_u;
        if(next_u_data > 1)
        {
            delta_u = 1.0 - u_data;
            next_u_data = u_data + delta_u;
        }

        ideal_length = calculateIdealLength(spline_inf, u_data, next_u_data, UsingNURBS);
        // std::cout << "u_0 value: " << u_data << "\n";
        // std::cout << "u_1 value: " << next_u_data << "\n";
        // std::cout << "delta_u value: " << delta_u << "\n";

        if(ideal_length > left_distance)
            ideal_length = left_distance;

        //std::cout << "ideal_length is : " << ideal_length << "\n";
        segment_already_move_distance += ideal_length;
        already_move_distance += ideal_length;

        update_suggest_velocity = ideal_length / constraint->sampling_dt;
        //update_delta_u = calculateDeltaU(spline_inf, u_data, update_suggest_velocity, 1, 3, UsingNURBS);
        std::cout << "update ideal length suggest_velocity is : " << update_suggest_velocity << "\n";
        //std::cout << "already_move_distance is : " << already_move_distance << "\n";
        //---------------------------------

        //angular_velocity = computeAngularVelocity(spline_inf, u_data, delta_u, UsingNURBS);
        angular_velocity = computeAngularVelocityMethod2(spline_inf, u_data, delta_u, UsingNURBS);

        std::cout << "angular_velocity is : " << angular_velocity << "\n";
        u_data += delta_u;
        assignVelocity(cmd_vel_, update_suggest_velocity, angular_velocity);
        //assignVelocity(cmd_vel_, 0.0, 0.0);

        //Visualization for ideal commands path
        if(!std::isnan(ideal_theta))
        {
            sum_ideal_theta += angular_velocity * constraint->sampling_dt;
            ideal_delta_pose_x = update_suggest_velocity * constraint->sampling_dt * std::cos(sum_ideal_theta);
            ideal_delta_pose_y = update_suggest_velocity * constraint->sampling_dt * std::sin(sum_ideal_theta);
            std::cout << "command index: " << command_index << "'s ideal_delta_pose_x is : " << ideal_delta_pose_x << "\n";
            std::cout << "command index: " << command_index << "'s ideal_delta_pose_y is : " << ideal_delta_pose_y << "\n";
            assignIdealCommandPose(ideal_delta_pose_x, ideal_delta_pose_y);
        }

        return cmd_vel_;
    }

    //TODO: Delete this function
    double NURBSPlanner::computeAngularVelocity(Spline_Inf spline_inf, double &u_data, double delta_u, bool UsingNURBS)
    {
        double angular_velocity = 0;
        //double delta_u = 0;
        double curvature_radius = 0;
        double theta = 0;
        Eigen::Vector3d eigen_derivative_point;


        //TODO:Check which u_data is better(u_data or next_u_data or average)
        curvature_radius = pCurve_common->CalculateCurvatureRadius(spline_inf, u_data, UsingNURBS);
        //arc_length = std::abs(pCurve_common->CalculateCurveLength(spline_inf, u_data, u_data + delta_u, 20000, UsingNURBS));
        //theta = (arc_length / curvature_radius) * 180 / M_PI; //radian to degree
        theta = (arc_length / curvature_radius); //radian 

        //Check initial direction
        if(!initialed)
            inverse_direction = checkInitialdirection(spline_inf, UsingNURBS);

        if(!inverse_direction)
        {
            angular_velocity = theta / constraint->sampling_dt;
        }
        else
        {
            angular_velocity = -1 * theta / constraint->sampling_dt;
        }

        inverse_direction = checkdirection(spline_inf, u_data, u_data + delta_u, UsingNURBS, inverse_direction);
        
        u_data += delta_u;
        return angular_velocity;
    }

    double NURBSPlanner::computeAngularVelocityMethod2(Spline_Inf spline_inf, double &u_data, double delta_u, bool UsingNURBS)
    {
        double angular_velocity = 0;
        double curvature_radius = 0;
        double theta = 0;
        
        if(!AngularVelocity2_initialized)
        {
            //agv_direction_vector(0) = initial_pose.pose.position.x;
            //agv_direction_vector(1) = initial_pose.pose.position.y;
            eigen_curve_point = EigenVecter3dFromPointMsg(pCurve_common->CalculateCurvePoint(&spline_inf, u_data, UsingNURBS));
            agv_direction_vector = eigen_curve_point;
            assignIdealCommandPose(eigen_curve_point(0), eigen_curve_point(1));
            sum_ideal_theta += original_angle;
            AngularVelocity2_initialized = true;
        }

        eigen_curve_point_old = eigen_curve_point;
        agv_direction_vector_old = agv_direction_vector;

        eigen_curve_point = EigenVecter3dFromPointMsg(pCurve_common->CalculateCurvePoint(&spline_inf, u_data + delta_u, UsingNURBS));
        
        agv_direction_vector = eigen_curve_point - eigen_curve_point_old;

        ideal_theta = std::atan2(agv_direction_vector(1), agv_direction_vector(0)) - std::atan2(agv_direction_vector_old(1), agv_direction_vector_old(0));

        //debug use
        // std::cout << "now agv angle(with degree): " << std::atan2(agv_direction_vector_old(1), agv_direction_vector_old(0)) * 180 / M_PI  << "\n";
        // std::cout << "desire direction angle(with degree): " << std::atan2(agv_direction_vector(1), agv_direction_vector(0)) * 180 / M_PI  << "\n";
        // std::cout << "diff angle(with degree): " << angular_velocity * 180 / M_PI  << "\n";
        
        ideal_theta = normalizeAngel(ideal_theta);
        std::cout << "normalize diff angle(with degree): " << angular_velocity * 180 / M_PI  << "\n";
        angular_velocity = ideal_theta / constraint->sampling_dt;
        return angular_velocity;
    }

    void NURBSPlanner::setInitialPose(geometry_msgs::PoseStamped pose)
    {
        initial_pose = pose;
    }

    //TODO: Delete this function
    bool NURBSPlanner::checkdirection(Spline_Inf spline_inf,double u_data, double next_u_data, bool UsingNURBS, bool direction)
    {
        Eigen::Vector3d curvature_vector_old;
        Eigen::Vector3d curvature_vector_new;
        double curvature_angle_denom;
        double curvature_angle;

        //Check direction
        curvature_vector_old = pCurve_common->CalculateCurvatureDirectionVector(spline_inf, u_data, UsingNURBS);
        curvature_vector_new = pCurve_common->CalculateCurvatureDirectionVector(spline_inf, next_u_data, UsingNURBS);
        curvature_angle_denom = curvature_vector_old.lpNorm<2>() * curvature_vector_new.lpNorm<2>();
        if(std::isnan(curvature_angle_denom))
        {
            std::cout << "this u no curvature angle " << "\n";
            curvature_angle = 0;
        }  
        else
            curvature_angle = std::acos(curvature_vector_old.dot(curvature_vector_new) / curvature_angle_denom);

        curvature_angle = curvature_angle * 180 / M_PI;
        //std::cout << "Curvature angle is : " << curvature_angle << "\n";

        if(curvature_angle > 90 && direction == false)
        {
            std::cout << "negative angular velocity" << "\n";
            return true;
        }
            
        if(curvature_angle > 90 && direction == true)
        {
            std::cout << "positive angular velocity" << "\n";
            return false;
        }

        return direction;

    }

    //TODO: Delete this function
    bool NURBSPlanner::checkInitialdirection(Spline_Inf spline_inf, bool UsingNURBS)
    {
        Eigen::Vector3d curvature_vector_initial;
        Eigen::Vector3d curvature_vector_start;
        
        curvature_vector_initial = pCurve_common->CalculateCurvatureDirectionVector(spline_inf, 0, UsingNURBS);
        curvature_vector_start = pCurve_common->CalculateCurvatureDirectionVector(spline_inf, 0.05, UsingNURBS);

        initialed = true;

        if(curvature_vector_start(0) > 0 && curvature_vector_start(1) > 0 || curvature_vector_start(0) < 0 && curvature_vector_start(1) > 0)
        {
            if(curvature_vector_initial(0) > 0)
            {
                std::cout << "positive angular velocity" << "\n";
                return false;
            }
            else
            {
                std::cout << "negative angular velocity" << "\n";
                return true;
            }
        }
        else
        {
            if(curvature_vector_initial(0) > 0)
            {
                std::cout << "negative angular velocity" << "\n";
                return true;
            }
            else
            {
                std::cout << "positive angular velocity" << "\n";
                return false;
            }
        }       
    }

    double NURBSPlanner::calculateAlignAngle(double robot_yaw, double trajectory_initial_angle)
    {
        Eigen::Vector3d initial_agv_vector;
        double fraction = 0;
        double denom = 0;
        double align_angle = 0;

        initial_agv_vector(0) = std::cos(robot_yaw);
        initial_agv_vector(1) = std::sin(robot_yaw);
       
        align_angle = std::atan2(initial_agv_vector(1), initial_agv_vector(0)) - trajectory_initial_angle;

        //debug use
        // std::cout << "diff angle(with degree): " << align_angle * 180 / M_PI  << "\n";
        // std::cout << "original angle(with degree): " << std::atan2(initial_agv_vector(1), initial_agv_vector(0)) * 180 / M_PI  << "\n";
        // std::cout << "desire angle(with degree): " << std::atan2(desire_vector(1), desire_vector(0)) * 180 / M_PI  << "\n";

        return align_angle;
    }

    double NURBSPlanner::calculateOriginalAngle(Spline_Inf spline_inf, double u_data, bool UsingNURBS)
    {
        Eigen::Vector3d trajectory_original_point;
        Eigen::Vector3d curve_vector;
        Eigen::Vector3d desire_vector;
        trajectory_original_point = EigenVecter3dFromPointMsg(pCurve_common->CalculateCurvePoint(&spline_inf, 0.0, UsingNURBS));
        curve_vector = EigenVecter3dFromPointMsg(pCurve_common->CalculateCurvePoint(&spline_inf, u_data, UsingNURBS));
        desire_vector = curve_vector - trajectory_original_point;
        original_angle = std::atan2(1, 0);
        return std::atan2(desire_vector(1), desire_vector(0));
    }

    double NURBSPlanner::normalizeAngel(double ang_diff)
    {
        //double ang_diff = align_angle - robot_pose_yaw;
        //double norm = 0.0;
        // double min = -1 * M_PI;
        // double max = M_PI;

        // norm = std::fmod(ang_diff + M_PI, 2.0 * M_PI);
        // if(norm <= 0.0)
        //     return norm + M_PI;

        // return norm - M_PI;

        // if(ang_diff >= min)
        //     norm = min + std::fmod((ang_diff - min), (max - min));
        // else
        //     norm = max - std::fmod((min - ang_diff), (max - min));
        
        // return norm;    

        if(ang_diff > M_PI)
            ang_diff -= 2 * M_PI;

        if(ang_diff < -1 * M_PI)
            ang_diff += 2 * M_PI;

        return ang_diff;
    }

    //TODO: Delete this function
    geometry_msgs::TwistStamped NURBSPlanner::computeAlignOrientationCommand(double align_angle, double angular_vel)
    {
        geometry_msgs::TwistStamped cmd_vel_;
        double align_velocity;
        double ang_diff_ = align_angle;
        std::cout << "without normalize ang_diff_ is : " << ang_diff_ << "\n";
        ang_diff_ = normalizeAngel(ang_diff_);
        std::cout << "ang_diff_ is : " << ang_diff_ << "\n";
        // align_velocity = ang_diff_ / constraint->sampling_dt;
        // assignVelocity(cmd_vel_, 0.0, align_velocity);
        
        if(ang_diff_ > 0)
            assignVelocity(cmd_vel_, 0.0, angular_vel);
        else
            assignVelocity(cmd_vel_, 0.0, -1 * angular_vel);
        
        return cmd_vel_;
    }

    void NURBSPlanner::adativeFeedrateCurvatureConstraint(Spline_Inf spline_inf, std::vector<AdativeFeedrateSeg>& trajectory_seg, double delta_u, bool UsingNURBS)
    {
        double u_data = 0.0;
        double start_u = 0.0;
        double start_velocity = 0.0;
        Eigen::Vector3d curvature_vector_old;
        Eigen::Vector3d curvature_vector_new;
        double curvature_angle_denom;
        double curvature_angle;
        int u_segment = 1 / delta_u;
        delta_u = 1 / (double)(u_segment - 1);

        std::vector<double> curve_parameter_vec;
        std::vector<double> break_u_vec;

        curve_parameter_vec.resize(u_segment);
        for(int u = 0; u < u_segment; u++)
        {
            curve_parameter_vec[u] = u_data;
            u_data += delta_u;
        }
        
        break_u_vec.clear();
        for(int i = 0; i < (u_segment - 1); i++)
        {
            //std::cout << "u data value before function value = " << u_data << "\n";
            curvature_vector_old = pCurve_common->CalculateCurvatureDirectionVector(spline_inf, curve_parameter_vec.at(i), UsingNURBS);
            //std::cout << "next u data value before function value = " << u_data + delta_u << "\n";
            curvature_vector_new = pCurve_common->CalculateCurvatureDirectionVector(spline_inf, curve_parameter_vec.at(i+1), UsingNURBS);
            curvature_angle_denom = curvature_vector_old.lpNorm<2>() * curvature_vector_new.lpNorm<2>();
            if(std::isnan(curvature_angle_denom))
            {
                std::cout << "this u no curvature angle " << "\n";
                curvature_angle = 0;
            }  
            else
                curvature_angle = std::acos(curvature_vector_old.dot(curvature_vector_new) / curvature_angle_denom);

            if(curvature_angle > M_PI / 2)
                break_u_vec.push_back(curve_parameter_vec.at(i));

            
            //std::cout << "u segment index: " << i << ", value = " << u_data << "\n";
            //u_data += delta_u;
        }
        trajectory_seg.resize(break_u_vec.size() + 1);

        for(int i = 0; i < break_u_vec.size() + 1; i++)
        {
            trajectory_seg.at(i).index = i;
            
            trajectory_seg.at(i).start_velocity = start_velocity;
            if(i != break_u_vec.size())
            {
                trajectory_seg.at(i).final_velocity = constraint->break_velocity;
                trajectory_seg.at(i).end_u = break_u_vec.at(i);
            }
            else
            {
                trajectory_seg.at(i).final_velocity = 0.0;
                trajectory_seg.at(i).end_u = 1.0;
            }
                

            trajectory_seg.at(i).length = pCurve_common->CalculateCurveLength(spline_inf, start_u, trajectory_seg.at(i).end_u, 50000, UsingNURBS);
            
            start_u = trajectory_seg.at(i).end_u;
            start_velocity = trajectory_seg.at(i).final_velocity;
        }

        for(int i = 0; i < trajectory_seg.size(); i++)
        {
            std::cout << "Trajectory segment index is : " << trajectory_seg.at(i).index << "\n";
            std::cout << "Trajectory segment start_velocity is : " << trajectory_seg.at(i).start_velocity << "\n";
            std::cout << "Trajectory segment final_velocity is : " << trajectory_seg.at(i).final_velocity << "\n";
            std::cout << "Trajectory segment end_u is : " << trajectory_seg.at(i).end_u << "\n";
            std::cout << "Trajectory segment length is : " << trajectory_seg.at(i).length << "\n";
        }
    }

    double NURBSPlanner::calculateIdealLength(Spline_Inf spline_inf, double u_data, double next_u_data, bool UsingNURBS)
    {
        double ideal_length = 0;
        geometry_msgs::Point ideal_old_position;

        if(next_u_data >= 1)
            next_u_data = 0.999;
        ideal_old_position = pCurve_common->CalculateCurvePoint(&spline_inf, u_data, UsingNURBS);
        // ideal_old_position.x = robot_pose.pose.position.x;
        // ideal_old_position.y = robot_pose.pose.position.y;
        geometry_msgs::Point ideal_position = pCurve_common->CalculateCurvePoint(&spline_inf, next_u_data, UsingNURBS);
        ideal_length = std::sqrt( std::pow( (ideal_old_position.x - ideal_position.x), 2) + std::pow( (ideal_old_position.y - ideal_position.y), 2));
        return ideal_length;
    }

    double NURBSPlanner::calculateDeltaU(Spline_Inf spline_inf, double u_data, double suggested_velocity, int taylor_order, int compensate_times, bool UsingNURBS)
    {
        double delta_u = 0;
        double delta_u_order1 = 0;
        double delta_u_order2 = 0;
        double compensate_u = 0;
        double ideal_length = 0;
        double error_length = 0;
        double next_u_data = 0;
        double correct_u_data = 0;
        double old_correct_u_data = 0;
        double fraction = 0;
        double denominator = 0;
        Eigen::Vector3d eigen_correct_derivative_point;
        Eigen::Vector3d eigen_derivative_point;
        Eigen::Vector3d eigen_derivative_twice_point;

        if(taylor_order == 1)
            pCurve_common->CalculateDerivativeBasisFunc(&spline_inf, u_data, 1);
        else
            pCurve_common->CalculateDerivativeBasisFunc(&spline_inf, u_data, 2);

        ideal_length = suggested_velocity * constraint->sampling_dt;
        eigen_derivative_point = EigenVecter3dFromPointMsg(pCurve_common->CalculateDerivativeCurvePoint(&spline_inf, u_data, 1, UsingNURBS));
        delta_u_order1 = ideal_length / eigen_derivative_point.lpNorm<2>();
        //std::cout << "wrong delta u is = " << delta_u_order1 << "\n";
        if(compensate_times > 0 && taylor_order == 1)
        {
            old_correct_u_data = u_data;
            next_u_data = u_data + delta_u_order1;
            for(int i = 0; i < compensate_times; i++)
            {
                error_length = calculateIdealLength(spline_inf, u_data, next_u_data, UsingNURBS);
                //std::cout << "ideal_length is = " << ideal_length << "\n";
                //std::cout << "error_length is = " << error_length << "\n";
                pCurve_common->CalculateDerivativeBasisFunc(&spline_inf, next_u_data, 1);
                eigen_correct_derivative_point = EigenVecter3dFromPointMsg(pCurve_common->CalculateDerivativeCurvePoint(&spline_inf, next_u_data, 1, UsingNURBS));
                compensate_u = (ideal_length - error_length) / eigen_correct_derivative_point.lpNorm<2>();
                //std::cout << "compensate_u is = " << compensate_u << "\n";
                correct_u_data = next_u_data + compensate_u;
                if(calculateRootMeanSquareError(correct_u_data, next_u_data) >= calculateRootMeanSquareError(next_u_data, old_correct_u_data))
                {
                    //correct_u_data = 2 * next_u_data - u_data;
                    correct_u_data = next_u_data;
                    std::cout << "not converage correct_u_data" << correct_u_data << "\n";
                }
                    
                old_correct_u_data = next_u_data;
                next_u_data = correct_u_data;
            }
            delta_u = correct_u_data - u_data;
            //std::cout << "correct delta u is = " << delta_u << "\n";
        }
    
        if(compensate_times == 0 && taylor_order == 1)
        {          
            delta_u = delta_u_order1;
        }

        if(taylor_order == 2)
        {
            eigen_derivative_twice_point = EigenVecter3dFromPointMsg(pCurve_common->CalculateDerivativeCurvePoint(&spline_inf, u_data, 2, UsingNURBS));
            fraction = std::pow(suggested_velocity, 2) * constraint->sampling_dt_squre * eigen_derivative_point.dot(eigen_derivative_twice_point);
            denominator = 2 * std::pow(eigen_derivative_point.lpNorm<2>(), 3);  
            delta_u_order2 = fraction / denominator;
            delta_u = delta_u_order1 - delta_u_order2;
        }
        
        return delta_u;
    }

    void NURBSPlanner::assignIdealCommandPose(double delta_x, double delta_y)
    {
        geometry_msgs::PoseStamped ideal_commands_pose;
        ideal_commands_pose.header.frame_id = "map";
        ideal_commands_pose.header.seq = command_index;
        ideal_commands_pose.header.stamp = ros::Time::now();
        if(command_index == 0)
        {
            ideal_commands_pose.pose.position.x = delta_x;
            ideal_commands_pose.pose.position.y = delta_y;
        }
        else
        {
            ideal_commands_pose.pose.position.x = ideal_commands_path.poses.at(command_index - 1).pose.position.x + delta_x;
            ideal_commands_pose.pose.position.y = ideal_commands_path.poses.at(command_index - 1).pose.position.y + delta_y;
        }
        std::cout << "command index: " << command_index << "'s ideal_commands_pose_x is : " << ideal_commands_pose.pose.position.x << "\n";
        std::cout << "command index: " << command_index << "'s ideal_commands_pose_y is : " << ideal_commands_pose.pose.position.y << "\n";
        ideal_commands_path.poses.push_back(ideal_commands_pose);
        command_index++;
    }

    void NURBSPlanner::publishIdealCommandsPath(std::string frame_id)
    {
        //TODO: clean ideal_commands_path when restart
        ideal_commands_path.header.frame_id = frame_id;
        ideal_commands_path.header.seq = 1;
        ideal_commands_path.header.stamp = ros::Time::now();
        ideal_commands_path_pub_.publish(ideal_commands_path);
    }

    inline double NURBSPlanner::calculateRootMeanSquareError(double value1, double value2)
    {
        return std::sqrt(std::pow((value1 - value2), 2));
    }

        double NURBSPlanner::getAlreadymoveLength()
    {
        return already_move_distance;
    }

    double NURBSPlanner::getSegmentAlreadyMoveLength()
    {
        return segment_already_move_distance;
    }
    
    void NURBSPlanner::setSegmentAlreadyMoveLength(double value)
    {
        segment_already_move_distance = value;
    }

    void NURBSPlanner::setAlreadyMoveLength(double value)
    {
        already_move_distance = value;
    }
}

