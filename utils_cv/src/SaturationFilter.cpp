#include "utils_cv/SaturationFilter.hpp"
#include <vector>

/** Simple class to filter an openCV mat in Saturation SPACE. */
SaturationFilter::SaturationFilter(uint8_t s_min, uint8_t s_max) {
  _s_min = s_min;
  _s_max = s_max;
}

void SaturationFilter::set_max(uint8_t max) {_s_max = max;}
void SaturationFilter::set_min(uint8_t min) {_s_min = min;}

uint8_t SaturationFilter::get_max() { return _s_max;}
uint8_t SaturationFilter::get_min() { return _s_min;}

/** Filters the input RGB image, SRC
    based on hue and stores the result in DEST.

    if ISMASK is set to true, it saves a binary
    [0==0, 1==255] image in DEST, else saves
    the inlier pixels into DEST [RGB values].

    An inlier is:
    _s_min <= pixel.saturation <= _s_max    */
void SaturationFilter::filter(cv::Mat &src, cv::Mat &dest, bool isMask) {
  cv::Mat imgHSV;
  if (src.channels() == 3)
    cv::cvtColor(src,imgHSV,CV_BGR2HSV);
  else
    src.copyTo(imgHSV);

  std::vector<cv::Mat> channels;
  cv::split(imgHSV, channels);
  cv::Mat sat_channel =  channels[1];
 
  cv::Mat mask(sat_channel.size(), sat_channel.type());
  cv::inRange(sat_channel, _s_min, _s_max, mask);

  if (isMask)
    mask.copyTo(dest);
  else
    src.copyTo(dest, mask);

  return;
}
