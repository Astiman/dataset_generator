If you want to add the module to the PX4-SITL build;

add this line:
-> add_topic("dataset_generator", 100);
into add_default_topics() function in logged_topics.cpp file.

run command:
make px4_sitl_default EXTERNAL_MODULES_LOCATION=<this-folder's-path>

and then start the SITL and run "dataset_generator start" command in the pxh console.
The required dataset will be automatically created in dataset_generator topic which can be found in the logs.
