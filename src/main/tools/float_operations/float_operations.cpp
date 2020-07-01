/**************************************************************************************************
Описание

Вспомогательные функции

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/



#include "float_operations.h"

#include <random>

using namespace motion_planner;


// seeds: default, 20, 25, 2009, 2512, 200998, 251297
static std::mt19937 randomGenerator( 20 );



float flt_op::getRandomFloat( float lowLimit, float highLimit )
{
    std::uniform_real_distribution< float > dis( lowLimit, highLimit );

    return dis( randomGenerator );
}



float flt_op::cvtDegToRad( float degrees )
{
    static const float coeffDegToRad = flt_op::mathPI / 180.f;

    return degrees * coeffDegToRad;
}



float flt_op::cvtRadToDeg( float radians )
{
    static const float coeffRadToDeg = 180.f / flt_op::mathPI;

    return radians * coeffRadToDeg;
}

