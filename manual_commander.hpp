#ifdef __cpp_lib_math_constants
#include <numbers>
#endif

#include <ros/ros.h>

#include <one_unit_robocon_2022/Twist.h>

#include "crs_lib/state_manager.hpp"
#include "crs_lib/logicool.hpp"
#include "crs_lib/rosparam_util.hpp"

#include "one_unit_robocon_2022/state.hpp"

namespace OneUnitRobocon2022
{
    namespace
    {
        class ManualCommander final
        {
            const struct RosParamData
            {
                double control_freq{};
                double max_body_linear_vel{};
                double max_body_angular_vel{};

                RosParamData(ros::NodeHandle& nh) noexcept
                {
                    using namespace CRSLib::RosparamUtil;

                    std::optional<StewXmlRpc> manual_commander_opt = get_param(nh, "manual_commander");

                    control_freq = read_param<double>(manual_commander_opt, "control_freq");
                    assert_param(control_freq, is_positive, 1000);

                    max_body_linear_vel = read_param<double>(manual_commander_opt, "max_body_linear_vel");
#ifdef __cpp_lib_math_constants
                    assert_param(max_body_linear_vel, is_positive, 2 * std::numbers::pi * 1);
#else
                    assert_param(max_body_linear_vel, is_positive, 2 * 3.141592653589793116 * 1);
                    
#endif
                    max_body_angular_vel = read_param<double>(manual_commander_opt, "max_body_angular_vel");
#ifdef __cpp_lib_math_constants
                    assert_param(max_body_angular_vel, is_positive, 2 * std::numbers::pi * 0.5);
#else
                    assert_param(max_body_angular_vel, is_positive, 2 * 3.141592653589793116 * 0.5);
#endif
                }
            } ros_param_data;

            ros::Publisher body_twist_pub;

            CRSLib::StateManager<StateEnum, ManualCommander> state_manager;
            ros::Timer pub_timer;

            CRSLib::Logicool logicool;

        public:
            ManualCommander(ros::NodeHandle& nh) noexcept:
                ros_param_data{nh},
                body_twist_pub{nh.advertise<one_unit_robocon_2022::Twist>("body_twist", 1)},
                state_manager{nh, this},
                pub_timer{nh.createTimer(ros::Duration(1 / ros_param_data.control_freq), &ManualCommander::timerCallback, this)},
                logicool{nh, "joy"}
            {}

        private:
            void timerCallback(const ros::TimerEvent&) noexcept
            {
                switch(state_manager.get_state())
                {
                case OneUnitRobocon2022::StateEnum::disable:
                    case_disable();
                    break;

                case OneUnitRobocon2022::StateEnum::reset:
                    case_reset();
                    break;

                case OneUnitRobocon2022::StateEnum::manual:
                    case_manual();
                    break;
                
                case OneUnitRobocon2022::StateEnum::automatic:
                    case_automatic();
                    break;
                }
            }

            void case_disable() noexcept
            {
                if(logicool.is_pushed_down(CRSLib::Logicool::Buttons::start))
                {
                    /// DEBUG:
                    ROS_INFO("In ManualCommander::case_disable(), disable to reset.");
                    state_manager.set_state(OneUnitRobocon2022::StateEnum::reset);
                }
            }

            void case_reset() noexcept
            {
                if(logicool.is_pushed_down(CRSLib::Logicool::Buttons::start))
                {
                    /// DEBUG:
                    ROS_INFO("In ManualCommander::case_reset(), reset to manual.");
                    state_manager.set_state(OneUnitRobocon2022::StateEnum::manual);
                }
            }

            void case_manual() noexcept
            {
                if(logicool.is_pushed_down(CRSLib::Logicool::Buttons::start))
                {
                    /// DEBUG:
                    ROS_INFO("In ManualCommander::case_manual(), manual to disable.");
                    state_manager.set_state(OneUnitRobocon2022::StateEnum::disable);
                }

                one_unit_robocon_2022::Twist cmd_vel{};

                auto axes = logicool.get_axes();
                cmd_vel.linear_x = ros_param_data.max_body_linear_vel * axes[CRSLib::Logicool::Axes::l_stick_UD];
                cmd_vel.linear_y = ros_param_data.max_body_linear_vel * axes[CRSLib::Logicool::Axes::l_stick_LR];
                cmd_vel.angular_z = ros_param_data.max_body_angular_vel * axes[CRSLib::Logicool::Axes::r_stick_LR];

                body_twist_pub.publish(cmd_vel);

            }

            void case_automatic() noexcept
            {
                if(logicool.is_pushed_down(CRSLib::Logicool::Buttons::start))
                {
                    state_manager.set_state(OneUnitRobocon2022::StateEnum::manual);
                }
            }
        };
    }
}

