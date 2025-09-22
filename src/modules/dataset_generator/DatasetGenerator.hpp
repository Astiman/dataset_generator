#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/dataset_generator.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_motors.h>
#include <lib/matrix/matrix/Quaternion.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>


class DatasetGenerator : public ModuleBase<DatasetGenerator>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    DatasetGenerator();
    ~DatasetGenerator() override = default;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool Init();

private:
    void update_vars();
    void publish_dataset();
    void Run() override;

    uORB::Subscription _local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
    uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};

    uORB::Publication<dataset_generator_s> _dataset_generator_pub{ORB_ID(dataset_generator)};

    param_t _p_use_ds_generator;
    int _use_ds_generator{1};

    float _last_pos_err_x{0.0f};
    float _last_pos_err_y{0.0f};
    float _last_pos_err_z{0.0f};

    float _last_rot_mat_0{0.0f};
    float _last_rot_mat_1{0.0f};
    float _last_rot_mat_2{0.0f};
    float _last_rot_mat_3{0.0f};
    float _last_rot_mat_4{0.0f};
    float _last_rot_mat_5{0.0f};

    float _last_lin_vel_x{0.0f};
    float _last_lin_vel_y{0.0f};
    float _last_lin_vel_z{0.0f};

    float _last_ang_vel_x{0.0f};
    float _last_ang_vel_y{0.0f};
    float _last_ang_vel_z{0.0f};

    float _last_actuator_0{0.0f};
    float _last_actuator_1{0.0f};
    float _last_actuator_2{0.0f};
    float _last_actuator_3{0.0f};
};
