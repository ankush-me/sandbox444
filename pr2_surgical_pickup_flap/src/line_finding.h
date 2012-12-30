#ifndef _LINE_FINDING_
#define _LINE_FINDING_

#include <vector>
#include <filter_cloud_color/utils/pcl_typedefs.h>

std::vector<float> getLineCoeffsRansac(ColorCloudPtr cloud);

#endif
