/** Author: Ankush Gupta
    Date  : 4th Sept, 2012. */

#include <opencv2/core/core.hpp>
#include <boost/shared_ptr.hpp>


/** A generic class for defining openCV functions
    which operate on images. */
class ImageProcessor {
public:
  
  /** Shared pointer to an ImageProcessor. */
  typedef boost::shared_ptr<ImageProcessor> Ptr;
  
  /** The core interface to the external users. */
  virtual void process(cv::Mat &src, cv::Mat &dst);
};
