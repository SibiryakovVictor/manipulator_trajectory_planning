/**************************************************************************************************
Описание

Определение типов индексов вершин и рёбер класса config_space::graph::Graph

Разработчик: Сибиряков Виктор
Заметки
* комментарии прилагаются
**************************************************************************************************/


#pragma once

#include <cinttypes>


namespace motion_planner
{
    namespace config_space
    {
        namespace graph
        {
            // !!! ВАЖНО !!!: при изменении типа установить соответствующее ему максимальное значение
            typedef uint8_t NodeId;
            const NodeId empty_node = UINT8_MAX;

            // !!! ВАЖНО !!!: при изменении типа установить соответствующее ему максимальное значение
            typedef uint8_t EdgeId;
            const EdgeId empty_edge = UINT8_MAX;
        }
    }
}

