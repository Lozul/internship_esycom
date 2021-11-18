#include "robot_main/data_exporter.h"

void export_correction_report(CorrectionReport report, std::string file_name)
{
    // Get laser scan from report
    if (!report.last_scan)
    {
        ROS_ERROR("DataExporter: correction report does not contains last scan");
        return;
    }

    auto scan = *report.last_scan;

    // Open log file
    std::ofstream log_file;
    log_file.open(file_name);

    // Write target data
    log_file << report.target.range << "," << report.target.theta_angle << std::endl;

    if (report.target.points.size() > 0)
    {
        log_file << report.target.points.front().angle << "," << report.target.points.front().range << std::endl;
        log_file << report.target.points.back().angle << "," << report.target.points.back().range << std::endl;
    }
    else
    {
        log_file << "0,0" << std::endl << "0,0" << std::endl;
    }

    if (report.polyfit.size() == 2)
    {
        log_file << report.polyfit[1] << "," << report.polyfit[0] << std::endl;
    }
    else
    {
        log_file << "0,0" << std::endl;
    }

    export_laser_scan(scan, log_file);

    log_file.close();

    ROS_INFO("DataExporter: '%s' saved.", file_name.c_str());
}

void export_laser_scan(sensor_msgs::LaserScanConstPtr scan, std::ofstream &log_file)
{
    // Write scan data
    for (int i = 0; i < scan->ranges.size(); i++)
    {
        float range = scan->ranges[i];

        if (range < scan->range_min || range > scan->range_max)
            continue;

        float angle = scan->angle_min + scan->angle_increment * i;

        log_file << angle << "," << range << std::endl;
    }
}

void export_laser_scan(sensor_msgs::LaserScanConstPtr scan, std::string &file_name)
{
    std::ofstream log_file;
    log_file.open(file_name);

    export_laser_scan(scan, log_file);

    log_file.close();

    ROS_INFO("DataExporter: '%s' saved.", file_name.c_str());
}
