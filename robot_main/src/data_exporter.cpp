#include "data_exporter.h"

void export_correction_report(CorrectionReport report, std::string file_name)
{
    // Get laser scan from report
    if (!report.last_scan)
    {
        ROS_ERROR("DataExporter: correction report does not contains last scan");
        continue;
    }

    auto scan = report.last_scan->get();

    // Open log file
    std::ofstream log_file;
    log_file.open(file_name);

    // Write entries of the target search algorithm
    log_file << report.target_distance << "," << report.theta_angle << std::endl;

    // Write target data if any
    if (report.first)
    {
        Point first = report.first.value();
        log_file << first.angle << "," << first.range << std::endl;
    }
    else
        log_file << "0,0" << std::endl;

    if (report.second)
    {
        Point second = report.second.value();
        log_file << second.angle << "," << second.range << std::endl;
    }
    else
        log_file << "0,0" << std::endl;

    if (report.correction_point)
    {
        auto p = report.correction_point.value();
        log_file << p.angle << "," << p.range << std::endl;
    }
    else
        log_file << "0,0" << std::endl;

    export_laser_scan(scan, log_file);

    log_file.close();

    ROS_INFO("DataExporter: '%f' saved.", file_name.c_str());
}

void export_laser_scan(sensor_msgs::LaserScanConstPtr scan, std::ofstream log_file)
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

void export_laser_scan(sensor_msgs::LaserScanConstPtr scan, std::string file_name)
{
    std::ofstream log_file;
    log_file.open(file_name);

    export_laser_scan(scan, log_file);

    log_file.close();

    ROS_INFO("DataExporter: '%f' saved.", file_name.c_str());
}