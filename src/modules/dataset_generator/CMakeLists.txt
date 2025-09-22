px4_add_module(
	MODULE modules__dataset_generator
	MAIN dataset_generator
	STACK_MAIN 2000
	SRCS
		DatasetGenerator.cpp
        DatasetGenerator.hpp
	EXTERNAL
	)