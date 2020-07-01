/**************************************************************************************************
Описание

Генератор случайных точек конфигурационного пространства

Разработчик: Сибиряков Виктор
Заметки
* для генерации используются диапазоны движения звеньев, определенные в "tools/intervals.h"
* генерация в методе generate
**************************************************************************************************/



#include "uniform_config_gen.h"


using namespace motion_planner::config_space::graph::generation;
using namespace motion_planner;



config_space::Point UniformConfigGen::generate() const
{
    config_space::Point result;

    for ( uint16_t curDim = 0; curDim != conf_space_dims; curDim++ )
    {
        result[ curDim ] = 
            flt_op::getRandomFloat( m_ranges[ curDim ].getlb(), m_ranges[ curDim ].gethb() );
    }

    return result;
}



void UniformConfigGen::changeRanges( const IntervalsAllDims & newRanges )
{
    m_ranges = newRanges;
}
