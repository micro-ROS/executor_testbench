# The rcl_executor repository

This repository contains executors developed in the micro-ROS project for real-time execution.

In detail, this repository contains two ROS packages:

*   [rcl_executor](rcl_executor/) provides an executor that works directly based on the ROS 2 Client Support Library (rcl)
*   [rcl_executor_examples](rcl_executor_examples/) provides small examples for the use of rcl_executor.

Technical information on the interfaces and use of these packages is given in the README.md files in the corresponding subfolders.

## Purpose of the project

The software is not ready for production use. It has neither been developed nor tested for a specific use case. However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs. Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust it according to any applicable safety standards (e.g. ISO 26262).


## Requirements, how to build, test, install, use, etc.

Clone the repository into a ROS workspace and build it using [colcon](https://colcon.readthedocs.io/).


## License

rcl_executor is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open source components included in rcl_executor, see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).


## Quality assurance

The colcon_test tool is used for quality assurances, which includes cpplint, uncrustify, flake8, xmllint and various other tools.

Unit tests based on [gtest](https://github.com/google/googletest) are located in the [rcl_executor/test](rcl_executor/test) folder.


## Known issues/limitations

Please notice the following issues/limitations:

*   rcl_executor only supports the ROS handle types subscriptions and timers. Other handle types, namely guard_conditions, services and clients are not supported. 