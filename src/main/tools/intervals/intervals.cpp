#include "intervals.h"

using namespace motion_planner;
using namespace motion_planner::tools;


IntervalsAllDims RangeLinksMotion::m_ranges = realRangesMotion;

bool RangeLinksMotion::canBothDirections[ manip::links_groups ] { false };
