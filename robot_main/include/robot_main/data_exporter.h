#pragma once

#include <string>
#include <sensor_msgs/LaserScan.h>
#include "robot_driver/robot_driver.h"

void export_correction_report(CorrectionReport report, std::string file_name);

void export_laser_scan(sensor_msgs::LaserScanConstPtr scan, std::ofstream log_file);
void export_laser_scan(sensor_msgs::LaserScanConstPtr scan, std::string file_name);