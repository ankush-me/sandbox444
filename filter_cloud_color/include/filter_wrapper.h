#ifndef _FILTER_WRAPPERS_
#define _FILTER_WRAPPERS_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/shared_ptr.hpp>

#include "utils/utils_pcl.h"
#include "cloud_ops.h"


/* Parent wrapper class for filter functions.
 Children of filter_wrapper must contain the function "filter" 
 to suit the actual filter it is wrapping. */
template<typename PointT> 
class filter_wrapper {
public:
  
  /* Filter function with pre-determined format. 
    Must be defined in child classes.
    
    Input: Shared pointers pointclouds, in and out.
    Output: Output points to filtered pointcloud. */
  virtual void filter (const typename pcl::PointCloud<PointT>::Ptr in, 
		       typename pcl::PointCloud<PointT>::Ptr out) {};
};


/* Wrapper for downsampling clouds. */
class downsample_wrapper : public filter_wrapper<ColorPoint> {
  float _size;

public:
  
  //Default constructor. Voxel size = 1cm: means 1pt/1cm^3
  downsample_wrapper() : _size(0.01) {};
  
  //Parametrized constructor : set the voxel side length to SIZE.
  downsample_wrapper(float size) : _size(size) {};

  //Accessors and modifiers for filter arguments
  inline void setSize(float size) {_size = size;}
  inline float getSize() {return _size;}
  
  /*Function filter:
    Input: in and out are ColorCloudPtr's
    Output: out points to filtered ColorCloud */
  void filter (const ColorCloudPtr in, ColorCloudPtr out) {
    ColorCloudPtr outCluster = downsampleCloud(in, _size);
    *out = *outCluster;
  }
};


/* Wrapper for clusterFilter.
   Requires a specific type of Point for function pointers.
   So using ColorPoint (pcl::PointXYZRGB).
   Look at pcl_typedefs.h for other type definitions. */
class clusterFilter_wrapper : public filter_wrapper<ColorPoint> {
  float tol_;
  int minSize_;
  
public:
  
  //Default constructor.
  clusterFilter_wrapper() : tol_(0.001), minSize_(100) {};
  
  //Parametrized constructor.
  clusterFilter_wrapper(float tolerance, int minSize);

  //Accessors and modifiers for filter arguments
  inline void setTolerance (float tolerance) {tol_ = tolerance;}
  inline float getTolerance () {return tol_;}
  
  inline void setMinsize (int minSize) {minSize_ = minSize;}
  inline int getMinsize () {return minSize_;}

  /*
    Function filter:
    Performs an Euclidean Cluster Extraction on the input pointcloud.
    Input: in and out are ColorCloudPtr's
    Output: out points to filtered ColorCloud
  */
  void filter (const ColorCloudPtr in, ColorCloudPtr out) {
    ColorCloudPtr outCluster = clusterFilter(in, tol_, minSize_);
    *out = *outCluster;
  }
};


/*  Wrapper for colorSpaceFilter.
    Requires a specific type of Point for function pointers.
    So using ColorPoint (pcl::PointXYZRGB).
    Look at pcl_typedefs.h for other type definitions. */
class colorSpaceFilter_wrapper : public filter_wrapper<ColorPoint> {
  
  uint8_t minx_;
  uint8_t maxx_;
  uint8_t miny_;
  uint8_t maxy_;
  uint8_t minz_;
  uint8_t maxz_;
  int code_;
  bool negative_;
  
public:
  
  //Default constructor.
  colorSpaceFilter_wrapper(): 
    minx_(), maxx_(), miny_(), maxy_(), minz_(), 
    maxz_(), code_(), negative_(false) {};
  
  //Parametrized constructor.
  colorSpaceFilter_wrapper (uint8_t minx, uint8_t maxx, uint8_t miny, 
			 uint8_t maxy, uint8_t minz, uint8_t maxz, 
			 int code, bool negative=false): 
    minx_(minx), maxx_(maxx), miny_(miny), maxy_(maxy), 
    minz_(minz), maxz_(maxz), code_(code), negative_(negative) {};

  //Accessors and modifiers for filter arguments
  inline void setMinx (uint8_t minx) {minx_ = minx;}
  inline uint8_t getMinx () {return minx_;}
  
  inline void setMaxx (uint8_t maxx) {maxx_ = maxx;}
  inline uint8_t getMaxx () {return maxx_;}

  inline void setMiny (uint8_t miny) {miny_ = miny;}
  inline uint8_t getMiny () {return miny_;}
  
  inline void setMaxy (uint8_t maxy) {maxy_ = maxy;}
  inline uint8_t getMaxy () {return maxy_;}

  inline void setMinz (uint8_t minz) {minz_ = minz;}
  inline uint8_t getMinz () {return minz_;}
  
  inline void setMaxz (uint8_t maxz) {maxz_ = maxz;}
  inline uint8_t getMaxz () {return maxz_;}

  inline void setCode (int code) {code_ = code;}
  inline int getCode () {return code_;}

  inline void setNegative (bool negative) {negative_ = negative;}
  inline bool getNegative () {return negative_;}

  /*
    Function filter:
    Performs an Euclidean Cluster Extraction on the input pointcloud.
    Input: in and out are ColorCloudPtr's
    Output: out points to filtered ColorCloud
  */
  void filter (const ColorCloudPtr in, ColorCloudPtr out) {
    ColorCloudPtr outCluster = colorSpaceFilter(in, minx_, maxx_, miny_, maxy_, 
						minz_, maxz_, code_, negative_);
    *out = *outCluster;
  }
};



/* Wrapper for hueFilter.
   Requires a specific type of Point for function pointers.
   So using ColorPoint (pcl::PointXYZRGB).
   Look at pcl_typedefs.h for other type definitions. */
class hueFilter_wrapper : public filter_wrapper<ColorPoint> {
  
  uint8_t minHue_;
  uint8_t maxHue_;
  uint8_t minSat_;
  uint8_t maxSat_;
  uint8_t minVal_;
  uint8_t maxVal_;
  bool negative_;
  
 public:
  
  //Default constructor.
 hueFilter_wrapper(): 
  minHue_(0), maxHue_(180), minSat_(0), maxSat_(255), 
    minVal_(0), maxVal_(255), negative_(false) {};
  
  //Parametrized constructor.
  hueFilter_wrapper (uint8_t minHue, uint8_t maxHue, uint8_t minSat, 
		     uint8_t maxSat, uint8_t minVal, uint8_t maxVal, 
		     bool negative): 
    minHue_(minHue), maxHue_(maxHue), minSat_(minSat), maxSat_(maxSat), 
    minVal_(minVal), maxVal_(maxVal), negative_(negative) {};

  //Accessors and modifiers for filter arguments
  inline void setMinHue (uint8_t minHue) {minHue_ = minHue;}
  inline uint8_t getMinHue () {return minHue_;}
  
  inline void setMaxHue (uint8_t maxHue) {maxHue_ = maxHue;}
  inline uint8_t getMaxHue () {return maxHue_;}

  inline void setMinSat (uint8_t minSat) {minSat_ = minSat;}
  inline uint8_t getMinSat () {return minSat_;}
  
  inline void setMaxSat (uint8_t maxSat) {maxSat_ = maxSat;}
  inline uint8_t getMaxSat () {return maxSat_;}

  inline void setMinVal (uint8_t minVal) {minVal_ = minVal;}
  inline uint8_t getMinVal () {return minVal_;}
  
  inline void setMaxVal (uint8_t maxVal) {maxVal_ = maxVal;}
  inline uint8_t getMaxVal () {return maxVal_;}

  inline void setNegative (bool negative) {negative_ = negative;}
  inline bool getNegative () {return negative_;}

  /**
    Function filter:
    Performs an Euclidean Cluster Extraction on the input pointcloud.
    Input: in and out are ColorCloudPtr's
    Output: out points to filtered ColorCloud
  **/
  void filter (const ColorCloudPtr in, ColorCloudPtr out) {
    ColorCloudPtr outCluster = hueFilter(in, minHue_, maxHue_, minSat_, maxSat_,
					 minVal_, maxVal_, negative_);
    *out = *outCluster;
  }
};


/** Filters out a point iff, point.z \notin [LOW,HIGH].
    LOW and HIGH are generally in meters. */
class filterZ_wrapper : public filter_wrapper<ColorPoint> {
  float _low, _high;
public:
  filterZ_wrapper(): _low(0), _high(100) { }
  filterZ_wrapper(float low, float high) : _low(low), _high(high) { }

  inline void setLowVal (float low) {_low = low;}
  inline float getLowVal () {return _low;}

  inline void setHighVal (float high) {_high = high;}
  inline float getHighVal () {return _high;}

  void filter (const ColorCloudPtr in, ColorCloudPtr out) {
    ColorCloudPtr filtered_pc =  filterZ(in, _low, _high);
    *out = *filtered_pc;
  }
};


/** Filters out a point iff the distance to the query point
    is greater than STD_THRESHOLD standard deviations
    from the mean distance. NUM NEIGHBORS number of
    points are used for each query. */
class removeOutliers_wrapper : public filter_wrapper<ColorPoint> {
  float _std_threshold;
  int _num_neighbors;

public:

 removeOutliers_wrapper(): _std_threshold(1),
    _num_neighbors(15) { }
 removeOutliers_wrapper(float thresh, int k) : _std_threshold(thresh),
    _num_neighbors(k) { }

  inline void setThreshold (float thresh) {_std_threshold = thresh;}
  inline float getThreshold () {return _std_threshold;}

  inline void setNumNeighbors (int k) {_num_neighbors = k;}
  inline int getNumNeighbors () {return _num_neighbors;}

  void filter (const ColorCloudPtr in, ColorCloudPtr out) {
    ColorCloudPtr filtered_pc =  removeOutliers(in, _std_threshold,
						_num_neighbors);
    *out = *filtered_pc;
  }
};


/** Sequentially applies filters given in the input. */
class filter_cascader : public filter_wrapper<ColorPoint> {
  std::vector<boost::shared_ptr < filter_wrapper <ColorPoint> > > *_filters;

public:
 filter_cascader() {
    _filters =
      new std::vector<boost::shared_ptr <filter_wrapper <ColorPoint> > >;
  }
  
 filter_cascader(std::vector< boost::shared_ptr <filter_wrapper <ColorPoint> > > *filters) {
    *_filters = *filters;
  }

  /** Clears the list of the stored filters. */
  inline void removeAll () {
    _filters =
      new std::vector< boost::shared_ptr < filter_wrapper<ColorPoint> > >;
  }

  /** Appends a filter onto the cascading list. */
  inline void appendFilter(boost::shared_ptr < filter_wrapper <ColorPoint> > filter) {
    _filters->push_back(filter);
  }

  /** Returns an iterator to the filters (from the beginning). */
  inline std::vector< boost::shared_ptr < filter_wrapper <ColorPoint> > >::iterator getFilterIter() {
    return _filters->begin();
  }

  void filter(const ColorCloudPtr in, ColorCloudPtr out) {
    ColorCloudPtr tmp1(new ColorCloud);
    ColorCloudPtr tmp2(new ColorCloud);

    *tmp1 = *in;
    for(int i = 0; i < _filters->size(); i += 1) {
      _filters->at(i)->filter(tmp1, tmp2);
      *tmp1 = *tmp2;
    }
    *out = *tmp1;
  }
};

#endif

/**
int main (int argc, char **argv) {
  std::cout<<"hello world!"<<std::endl;
  colorSpaceFilter_wrapper *csfw = new colorSpaceFilter_wrapper();
}**/
