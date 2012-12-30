#pragma once
#include <vector>

#include "filter_wrapper.h"
#include "surgical_units.hpp"

/*
  Variables for storing the holes/cuts/suture.
*/
std::vector<Hole::Ptr> holes;
std::vector<Cut::Ptr> cuts;
Suture::Ptr suture (new Suture());

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
  Returns a point cloud displaying only the suture.
  Runs a hueFilter and assumes the suture is of a unique color.
*/
ColorCloudPtr showSuture (ColorCloudPtr in) {

  uint8_t suture_minH = (suture->_H - HUE_STD_SCALE*suture->_Hstd)%180;
  uint8_t suture_maxH = (suture->_H + HUE_STD_SCALE*suture->_Hstd)%180;
  uint8_t suture_minS = (suture->_S - suture->_Sstd > 0) ? suture->_S - suture->_Sstd : 0;
  uint8_t suture_maxS = (suture->_S + suture->_Sstd < 255) ? suture->_S + suture->_Sstd : 255;
  uint8_t suture_minV = (suture->_V - suture->_Vstd > 0) ? suture->_V - suture->_Vstd : 0;
  uint8_t suture_maxV = (suture->_V + suture->_Vstd < 255) ? suture->_V + suture->_Vstd : 255;
  
  filter_cascader suture_cascader;

  boost::shared_ptr<hueFilter_wrapper> suture_HF
    (new hueFilter_wrapper (suture_minH, suture_maxH, suture_minS, suture_maxS,
			    suture_minV, suture_maxV, false));

  boost::shared_ptr<removeOutliers_wrapper> 
    suture_OR (new removeOutliers_wrapper());
  
  suture_cascader.appendFilter(suture_HF);
  suture_cascader.appendFilter(suture_OR);

  ColorCloudPtr out (new ColorCloud());
  
  suture_cascader.filter(in, out);
  
  return out;
}

/*
  Function that returns with pointCloud cuts/holes/suture as specified.
  Takes pointCloud.
*/
ColorCloudPtr extractSurgicalUnits (ColorCloudPtr in,
				    vector<int> *holeInds,
				    vector<int> *cutInds,
				    int sutureFlag) {
  
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

  if (sutureFlag) 
    *out = *out + *showSuture(in);
  
  return out;
}
