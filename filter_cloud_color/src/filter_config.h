#pragma once
#include "utils/config.h"
#include <vector>

/* 
   Configuration for command line options
*/
struct LocalConfig : Config {
  static std::string pcTopic;
  static float downsample;
  static int tableMaxH;
  static int tableMinH;
  static int tableMaxS;
  static int tableMinS;
  static int tableMaxV;
  static int tableMinV;
  static bool tableNeg;
  static float zClipHigh;
  static float zClipLow;
  static int maxH;
  static int minH;
  static int maxS;
  static int minS;
  static int maxV;
  static int minV;
  static bool useHF;
  static int holes;
  static int cuts;
  static int needle;
  static int debugging;
  static int display;
  static int useGUI;

  LocalConfig() : Config() {
    params.push_back(new Parameter<std::string>
		     ("pcTopic", &pcTopic, "Point Cloud Topic"));
    params.push_back(new Parameter<float>
		     ("downsample", &downsample, "Downsampling"));
    params.push_back(new Parameter<int>
		     ("tableMaxH", &tableMaxH, "Maximum table hue"));
    params.push_back(new Parameter<int>
		     ("tableMinH", &tableMinH, "Minimum table hue"));
    params.push_back(new Parameter<int>
		     ("tableMaxS", &tableMaxS, "Maximum table saturation"));
    params.push_back(new Parameter<int>
		     ("tableMinS", &tableMinS, "Minimum table saturation"));
    params.push_back(new Parameter<int>
		     ("tableMaxV", &tableMaxV, "Maximum table value"));
    params.push_back(new Parameter<int>
		     ("tableMinV", &tableMinV, "Minimum table value"));
    params.push_back(new Parameter<bool>
		     ("tableNeg", &tableNeg, "Filter out/in table")); 
    params.push_back(new Parameter<float>
		     ("zClipHigh", &zClipHigh, "Clip above this"));
    params.push_back(new Parameter<float>
		     ("zClipLow", &zClipLow, "Clip below this"));
    params.push_back(new Parameter<int>("maxH", &maxH, "Maximum hue"));
    params.push_back(new Parameter<int>("minH", &minH, "Minimum hue"));
    params.push_back(new Parameter<int>("maxS", &maxS, "Maximum saturation"));
    params.push_back(new Parameter<int>("minS", &minS, "Minimum saturation"));
    params.push_back(new Parameter<int>("maxV", &maxV, "Maximum value"));
    params.push_back(new Parameter<int>("minV", &minV, "Minimum value"));
    params.push_back(new Parameter<bool>("useHF", &useHF, "Use hue filter"));
    params.push_back(new Parameter<int>
		     ("holes", &holes, "Indices for displayed holes(0 - all)"));
    params.push_back(new Parameter<int>
		     ("cuts", &cuts, "Indices for displayed cuts(0 - all)"));
    params.push_back(new Parameter<int>
		     ("needle", &needle, "Flag to display suturing needle"));
    params.push_back(new Parameter<int>
		     ("debugging", &debugging, "Debug flag: 1/0 - Yes/No"));
    params.push_back(new Parameter<int>
			 ("display", &display, "Display on: 1/0 - Yes/No"));
    params.push_back(new Parameter<int>
    		 ("useGUI", &useGUI, "useGUI: 1/0 - Yes/No"));
  }
};
