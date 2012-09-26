/** Author: Ankush Gupta
    Date  : 4th Sept, 2012. */

#ifndef _IMAGE_PROCESSOR_UTILS_CV_H_
#define _IMAGE_PROCESSOR_UTILS_CV_H_

#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>


/** A generic class for defining openCV functions
    which operate on images. */
class ImageProcessor {
public:
  
  /** Shared pointer to an ImageProcessor. */
  typedef boost::shared_ptr<ImageProcessor> Ptr;
  
  /** The core interface to the external users. */
  virtual void process(cv::Mat &src, cv::Mat &dst) =0;
};

/** Identity Processor : DOES NOTHING. */
class IdentityProcessor : public ImageProcessor {
public:
  void process (cv::Mat &src, cv::Mat &dst) {dst = src;}
};

#endif
