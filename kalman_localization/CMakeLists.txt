cmake_minimum_required(VERSION 3.0.2)
project(kalman_localization)
find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(PROGRAMS
  scripts/gps_to_odom.py
  scripts/drive_pattern.py
  scripts/compute_metrics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_experiment.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY urdf/   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)
