// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_H_
#define _LIDAR_H_

//define lidar parameter

namespace lidar {

class Lidar
{
  public:
    Lidar();

    void setLines(double num_lines_in);
    void setScanPeriod(double scan_period_in);
    //by default is 100. pls do not change
    void setMaxDistance(double max_distance_in);
    void setMinDistance(double min_distance_in);

    int num_lines;
    double scan_period;
    double max_distance;
    double min_distance;
    int points_per_line;
};

}

#endif // _LIDAR_H_
