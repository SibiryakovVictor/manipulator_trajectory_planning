/**************************************************************************************************
Описание

Файл, отвечающий за определение диапазона движения звеньев 
(определены в константе realRangesMotion)

Разработчик: Сибиряков Виктор
Заметки
для доступа к константе с диапазонами нужно инклуднуть данный файл и вызвать 
motion_planner::tools::RangeLinksMotion::get() 
**************************************************************************************************/



#include "intervals.h"

using namespace motion_planner;
using namespace motion_planner::tools;


IntervalsAllDims RangeLinksMotion::m_ranges = realRangesMotion;

bool RangeLinksMotion::canBothDirections[ manip::links_groups ] { false };
