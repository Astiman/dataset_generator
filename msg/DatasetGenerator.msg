# Dataset generator for neural network training.

uint64 timestamp # [us] Time since system start.

float64 pos_err_x 	# Position error in x
float64 pos_err_y 	# Position error in y
float64 pos_err_z 	# Position error in z

float64 rot_mat_0 	# Rotation matrix element 0
float64 rot_mat_1 	# Rotation matrix element 1
float64 rot_mat_2 	# Rotation matrix element 2
float64 rot_mat_3 	# Rotation matrix element 3
float64 rot_mat_4 	# Rotation matrix element 4
float64 rot_mat_5 	# Rotation matrix element 5

float64 lin_vel_x 	# Linear velocity in x
float64 lin_vel_y 	# Linear velocity in y
float64 lin_vel_z 	# Linear velocity in z

float64 ang_vel_x 	# Angular velocity in x
float64 ang_vel_y 	# Angular velocity in y
float64 ang_vel_z 	# Angular velocity in z

float32 actuator_0 	# Actuator command 0
float32 actuator_1 	# Actuator command 1
float32 actuator_2 	# Actuator command 2
float32 actuator_3 	# Actuator command 3

# TOPICS dataset_generator
