#pragma once

#include <cinttypes>

#include "main/config_space/conf_space_dims.h"



namespace motion_planner
{
    namespace config_space
    {
        namespace graph
        {
            const uint16_t start_node_pos = 0;
            const uint16_t goal_node_pos = 1;

            const uint16_t start_comp = start_node_pos;
            const uint16_t goal_comp = goal_node_pos;

            const uint16_t edges_limit = 4;

            const uint16_t capacity = 200;
        }
    }
}

