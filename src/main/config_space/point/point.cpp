#include "point.h"
#include "main/tools/float_operations/float_operations.h"

#include <cmath>
#include <algorithm>


using namespace motion_planner::config_space;

/**************************************************************************************************
Описание:
определяет евклидово расстояниt между точками
Аргументы:
* p1, p2: точки
Возврат: евклидово расстояниt между точками
**************************************************************************************************/
float Point::calcDistance( const Point & p1, const Point & p2 )
{
    return std::sqrtf( calcDistNoSqrt( p1, p2 ) );
}



/**************************************************************************************************
Описание:
определяет квадрат (потому что не извлекается корень) евклидово расстояния между точками
Аргументы:
* p1, p2: точки
Возврат: квадрат евклидово расстояния между точками
**************************************************************************************************/
float Point::calcDistNoSqrt( const Point & p1, const Point & p2 )
{
    return std::inner_product( p1.begin(), p1.end(), p2.begin(), 0.f,
        []( float arg, float result ){ return result + arg; },
        []( float arg1, float arg2 ) 
        {
            float diff = arg1 - arg2;

            return diff * diff; 
        } );
}



void Point::cvtDegsToRads()
{
    std::for_each( m_pointData.dimVals, m_pointData.dimVals + conf_space_dims,
        []( float & dimVal )
        {
            dimVal = flt_op::cvtDegToRad( dimVal );
        } );
}



/**************************************************************************************************
Описание:
определяет точки как равные, если расстояние между ними меньше заданного порога
Аргументы:
* p1, p2: точки
Возврат: равны друг другу точки или нет
**************************************************************************************************/
bool motion_planner::config_space::operator==( const Point & p1, const Point & p2 )
{
    static const float threshold_points_equal = 0.0001f;

    return motion_planner::config_space::Point::calcDistance( p1, p2 ) < threshold_points_equal;
}

