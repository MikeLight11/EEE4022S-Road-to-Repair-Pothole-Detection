cmake_minimum_required(VERSION 3.16)
# Fixed version so the build doesn't run `git describe`, which hangs while
# OneDrive is syncing the .git folder.
set(PROJECT_VER "imu_raw_1")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tilt_angle)
