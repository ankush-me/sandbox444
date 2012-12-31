#include "filter_config.h"
/*
  Default values for command line options.
  The ones in comments are for red table. And with some downsampling.
  The ones not in comments are for foam block.
*/
std::string LocalConfig::pcTopic = "/camera/depth_registered/points";
//float LocalConfig::downsample = 0.008;
float LocalConfig::downsample = 0.0;
//int LocalConfig::tableMaxH = 10;
int LocalConfig::tableMaxH = 180;
//int LocalConfig::tableMinH = 170;
int LocalConfig::tableMinH = 0;
//int LocalConfig::tableMaxS = 255;
int LocalConfig::tableMaxS = 150;
//int LocalConfig::tableMinS = 150;
int LocalConfig::tableMinS = 0;
int LocalConfig::tableMaxV = 255;
int LocalConfig::tableMinV = 0;
bool LocalConfig::tableNeg = false;
float LocalConfig::zClipLow = -0.05;
float LocalConfig::zClipHigh = 0.5;
int LocalConfig::maxH = 180;
int LocalConfig::minH = 0;
int LocalConfig::maxS = 255;
int LocalConfig::minS = 0;
int LocalConfig::maxV = 255;
int LocalConfig::minV = 0;
bool LocalConfig::useHF = false;
int LocalConfig::holes = 0;
int LocalConfig::cuts = 0;
int LocalConfig::needle = 0;
int LocalConfig::debugging = 0;
int LocalConfig::display = 1;
