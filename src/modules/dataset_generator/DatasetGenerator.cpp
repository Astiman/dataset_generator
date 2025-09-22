#include "DatasetGenerator.hpp"


using namespace time_literals;

DatasetGenerator::DatasetGenerator() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
    _p_use_ds_generator = param_find("USE_DS_GENERATOR");

    if (_p_use_ds_generator != PARAM_INVALID) {
        param_get(_p_use_ds_generator, &_use_ds_generator);
    } else {
        PX4_ERR("Failed to find parameter USE_DS_GENERATOR");
        _use_ds_generator = 1;
    }
}

void DatasetGenerator::update_vars()
{
    trajectory_setpoint_s setpoint{};
    vehicle_local_position_s local_position{};
    vehicle_angular_velocity_s angular_velocity{};
    vehicle_attitude_s attitude{};
    actuator_motors_s actuator_motors{};

    if (_local_position_sub.update(&local_position)) {
        if (_trajectory_setpoint_sub.update(&setpoint)) {
            _last_pos_err_x = setpoint.position[0] - local_position.x;
            _last_pos_err_y = setpoint.position[1] - local_position.y;
            _last_pos_err_z = setpoint.position[2] - local_position.z;
        }

        _last_lin_vel_x = local_position.vx;
        _last_lin_vel_y = local_position.vy;
        _last_lin_vel_z = local_position.vz;
    }

    if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
        _last_ang_vel_x = angular_velocity.xyz[0];
        _last_ang_vel_y = angular_velocity.xyz[1];
        _last_ang_vel_z = angular_velocity.xyz[2];
    }

    if (_actuator_motors_sub.update(&actuator_motors)) {
        _last_actuator_0 = actuator_motors.control[0];
        _last_actuator_1 = actuator_motors.control[1];
        _last_actuator_2 = actuator_motors.control[2];
        _last_actuator_3 = actuator_motors.control[3];
    }

    if (_att_sub.update(&attitude)) {
        // Quaternion oluştur
        matrix::Quatf q(attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3]);

        // Quaternion -> Rotation Matrix (DCM)
        matrix::Dcmf R(q);

        // Mesela sadece ilk iki satır/sütunu saklayalım
        _last_rot_mat_0 = R(0,0);
        _last_rot_mat_1 = R(0,1);
        _last_rot_mat_2 = R(0,2);
        _last_rot_mat_3 = R(1,0);
        _last_rot_mat_4 = R(1,1);
        _last_rot_mat_5 = R(1,2);
    }
}

void DatasetGenerator::publish_dataset()
{
    dataset_generator_s msg{};

    msg.timestamp = hrt_absolute_time();
    msg.pos_err_x = _last_pos_err_x;
    msg.pos_err_y = _last_pos_err_y;
    msg.pos_err_z = _last_pos_err_z;

    msg.rot_mat_0 = _last_rot_mat_0;
    msg.rot_mat_1 = _last_rot_mat_1;
    msg.rot_mat_2 = _last_rot_mat_2;
    msg.rot_mat_3 = _last_rot_mat_3;
    msg.rot_mat_4 = _last_rot_mat_4;
    msg.rot_mat_5 = _last_rot_mat_5;

    msg.lin_vel_x = _last_lin_vel_x;
    msg.lin_vel_y = _last_lin_vel_y;
    msg.lin_vel_z = _last_lin_vel_z;

    msg.ang_vel_x = _last_ang_vel_x;
    msg.ang_vel_y = _last_ang_vel_y;
    msg.ang_vel_z = _last_ang_vel_z;

    msg.actuator_0 = _last_actuator_0;
    msg.actuator_1 = _last_actuator_1;
    msg.actuator_2 = _last_actuator_2;
    msg.actuator_3 = _last_actuator_3;

    _dataset_generator_pub.publish(msg);
}

void DatasetGenerator::Run()
{
    if (_use_ds_generator == 0) {
        ScheduleClear();
        return;
    }

    update_vars();
    publish_dataset();

    // tekrar çalışması için
    ScheduleDelayed(10_ms);   // 100Hz
}

int DatasetGenerator::task_spawn(int argc, char *argv[])
{
    DatasetGenerator *instance = new DatasetGenerator();

    if (!instance) {
        PX4_ERR("Allocation failed");
        return PX4_ERROR;
    }

    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->Init()) {
        return PX4_OK;
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;
    return PX4_ERROR;
}

int DatasetGenerator::custom_command(int argc, char *argv[])
{
    return print_usage("Unknown command");
}

int DatasetGenerator::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Dataset generator module.

Publishes dataset_generator topic.

### Usage
dataset_generator start
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("dataset_generator", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    return 0;
}

bool DatasetGenerator::Init()
{
    ScheduleNow();
    return true;
}

extern "C" __EXPORT int dataset_generator_main(int argc, char *argv[])
{
    return DatasetGenerator::main(argc, argv);
}
