#ifndef AGV_LOCAL_PLANNER_PHYSICAL_CONSTRAINTS_H_
#define AGV_LOCAL_PLANNER_PHYSICAL_CONSTRAINTS_H_

#include <ros/ros.h>
#include <math.h>

namespace nurbs_local_planner {
    
    struct PhysicalConstraints
    {
        // PhysicalConstraints()
        // : v_max(1.0),
        //     a_max(2.0),
        //     yaw_rate_max(M_PI / 4.0),
        //     robot_radius(1.0),
        //     sampling_dt(0.01) {}

        void setParametersFromRos(const ros::NodeHandle& nh) {
            nh.param("v_max", v_system_max, v_system_max);
            //nh.param("a_max", a_max, a_max);
            nh.param("a_max", system_linear_a_max, system_linear_a_max);
            nh.param("yaw_rate_max", yaw_rate_max, yaw_rate_max);
            nh.param("robot_radius", robot_radius, robot_radius);
            nh.param("break_velocity", break_velocity, v_system_max / 2);
            nh.param("sampling_dt", sampling_dt, 0.01);
            nh.param("reaction_time", reaction_time, 1.0);
            std::cout << "sampling time : " << sampling_dt << "\n";
            //TODO: Can modify sampliing time in launch file
            //sampling_dt = 0.01;
            a_max = system_linear_a_max;
            //v_max_square = std::pow(v_max, 2);
            a_max_square = std::pow(a_max, 2);
            system_linear_a_max_square = std::pow(system_linear_a_max, 2);
            reaction_time_square = std::pow(reaction_time, 2);
            sampling_dt_square = std::pow(sampling_dt, 2);
        }

        double v_max;  // Meters/second
        double a_max;  // Meters/second^2
        double adaptive_a_max;
        double system_linear_a_max;
        double yaw_rate_max;  // Rad/second
        double v_distance_max;
        double v_system_max;

        double reaction_time;
        double robot_radius;  // Meters
        double sampling_dt;  // Seconds
        //double v_max_square;
        double a_max_square;
        double system_linear_a_max_square;
        double reaction_time_square;
        double sampling_dt_square;
        double break_velocity;
    };
};

#endif //AGV_LOCAL_PLANNER_PHYSICAL_CONSTRAINTS_H_