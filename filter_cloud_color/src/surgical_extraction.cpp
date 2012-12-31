#pragma once
#include <vector>

#include "filter_wrapper.h"
#include "surgical_units.hpp"

/*
  Variables for storing the holes/cuts/suturing needle.
*/
std::vector<Hole::Ptr> holes;
std::vector<Cut::Ptr> cuts;
Needle::Ptr needle (new Needle());

//Scale for stddev of hue to find points of similar color
int HUE_STD_SCALE = 2;

/*
  Returns a point cloud displaying only the hole specified by the index.
  Runs a hueFilter and assumes holes are of unique colors.
*/
ColorCloudPtr showHole (ColorCloudPtr in, int index) {
  Hole::Ptr hole = holes[index-1];
  
  uint8_t hole_minH = (hole->_H - HUE_STD_SCALE*hole->_Hstd)%180;
  uint8_t hole_maxH = (hole->_H + HUE_STD_SCALE*hole->_Hstd)%180;
  uint8_t hole_minS = (hole->_S - hole->_Sstd > 0) ? hole->_S - hole->_Sstd : 0;
  uint8_t hole_maxS = (hole->_S + hole->_Sstd < 255) ? hole->_S + hole->_Sstd : 255;
  uint8_t hole_minV = (hole->_V - hole->_Vstd > 0) ? hole->_V - hole->_Vstd : 0;
  uint8_t hole_maxV = (hole->_V + hole->_Vstd < 255) ? hole->_V + hole->_Vstd : 255;
  
  filter_cascader hole_cascader;
  
  boost::shared_ptr<hueFilter_wrapper> hole_HF 
    (new hueFilter_wrapper (hole_minH, hole_maxH, hole_minS, hole_maxS,
			    hole_minV, hole_maxV, false));

  boost::shared_ptr<removeOutliers_wrapper> 
    hole_OR (new removeOutliers_wrapper());
  
  hole_cascader.appendFilter(hole_HF);
  hole_cascader.appendFilter(hole_OR);

  ColorCloudPtr out (new ColorCloud());
  
  hole_cascader.filter(in, out);
  
  return out;
}

/*
  Returns a point cloud displaying only the cut specified by the index.
  Runs a hueFilter and assumes cuts are of unique colors.
*/
ColorCloudPtr showCut (ColorCloudPtr in, int index) {
  Cut::Ptr cut = cuts[index-1];
  
  uint8_t cut_minH = (cut->_H - HUE_STD_SCALE*cut->_Hstd)%180;
  uint8_t cut_maxH = (cut->_H + HUE_STD_SCALE*cut->_Hstd)%180;
  uint8_t cut_minS = (cut->_S - cut->_Sstd > 0) ? cut->_S - cut->_Sstd : 0;
  uint8_t cut_maxS = (cut->_S + cut->_Sstd < 255) ? cut->_S + cut->_Sstd : 255;
  uint8_t cut_minV = (cut->_V - cut->_Vstd > 0) ? cut->_V - cut->_Vstd : 0;
  uint8_t cut_maxV = (cut->_V + cut->_Vstd < 255) ? cut->_V + cut->_Vstd : 255;
  
  filter_cascader cut_cascader;

  boost::shared_ptr<hueFilter_wrapper> cut_HF 
    (new hueFilter_wrapper (cut_minH, cut_maxH, cut_minS, cut_maxS,
			    cut_minV, cut_maxV, false));
  
  boost::shared_ptr<removeOutliers_wrapper> 
    cut_OR (new removeOutliers_wrapper());

  cut_cascader.appendFilter(cut_HF);
  cut_cascader.appendFilter(cut_OR);

  ColorCloudPtr out (new ColorCloud());
  
  cut_cascader.filter(in, out);
  
  return out;
}

/*
  Returns a point cloud displaying only the suturing needle.
  Runs a hueFilter and assumes the needle is of a unique color.
*/
ColorCloudPtr showNeedle (ColorCloudPtr in) {

  uint8_t needle_minH = (needle->_H - HUE_STD_SCALE*needle->_Hstd)%180;
  uint8_t needle_maxH = (needle->_H + HUE_STD_SCALE*needle->_Hstd)%180;
  uint8_t needle_minS = (needle->_S - needle->_Sstd > 0) ? needle->_S - needle->_Sstd : 0;
  uint8_t needle_maxS = (needle->_S + needle->_Sstd < 255) ? needle->_S + needle->_Sstd : 255;
  uint8_t needle_minV = (needle->_V - needle->_Vstd > 0) ? needle->_V - needle->_Vstd : 0;
  uint8_t needle_maxV = (needle->_V + needle->_Vstd < 255) ? needle->_V + needle->_Vstd : 255;
  
  filter_cascader needle_cascader;

  boost::shared_ptr<hueFilter_wrapper> needle_HF
    (new hueFilter_wrapper (needle_minH, needle_maxH, needle_minS, needle_maxS,
			    needle_minV, needle_maxV, false));

  boost::shared_ptr<removeOutliers_wrapper> 
    needle_OR (new removeOutliers_wrapper());
  
  needle_cascader.appendFilter(needle_HF);
  needle_cascader.appendFilter(needle_OR);

  ColorCloudPtr out (new ColorCloud());
  
  needle_cascader.filter(in, out);
  
  return out;
}

/*
  Function that returns with pointCloud cuts/holes/suturing needle as specified.
  Takes pointCloud.
*/
ColorCloudPtr extractSurgicalUnits (ColorCloudPtr in,
				    vector<int> *holeInds,
				    vector<int> *cutInds,
				    int needleFlag) {
  
  ColorCloudPtr out (new ColorCloud());
  bool holeFlag = false, cutFlag = false;

  if (holeInds->at(0) == 0) {
    for (int i = 0; i < holes.size(); i++) 
      *out = *out + *showHole(in, i+1);
  } else {
    for (int i = 0; i < holeInds->size(); i++) 
      if (holeInds->at(i) <= holes.size()) {
    	holeFlag = true;
    	*out = *out + *showHole(in, holeInds->at(i));
      }
    if (!holeFlag)
      for (int i = 0; i < holes.size(); i++)
    	*out = *out + *showHole(in, i+1);
  }

  if (cutInds->at(0) == 0)
    for (int i = 0; i < cuts.size(); i++)
      *out = *out + *showCut(in, i+1);
  else {
    for (int i = 0; i < cutInds->size(); i++)
      if (cutInds->at(i) <= cuts.size()) {
        *out = *out + *showCut(in, cutInds->at(i));
    	cutFlag = true;
      }
    if (!cutFlag)
      for (int i = 0; i < cuts.size(); i++)
        *out = *out + *showCut(in, i+1);
  }

  if (needleFlag)
    *out = *out + *showNeedle(in);
  
  return out;
}