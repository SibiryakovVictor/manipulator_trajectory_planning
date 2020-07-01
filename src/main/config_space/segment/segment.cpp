/**************************************************************************************************
Описание

Определение отрезка конфигурационного пространства (две его точки, соединенные прямой)

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/



#include "segment.h"

#include <cmath>
#include <cfloat>
#include <algorithm>


using namespace motion_planner::config_space;



/**************************************************************************************************
Описание:
возвращает точку отрезка, рассчитывая её из переданной величины параметра 
(по аналогии с параметрическим уравнением прямой:
параметр 0 - начало отрезка, 1 - конец отрезка, 0.5 - середина и т.д.)
Аргументы: величина параметра
Возврат: точка отрезка
**************************************************************************************************/
Point Segment::getPoint( float param ) const
{
	Point result;

	auto dim = 0;
	std::for_each( result.begin(), result.end(), [ &dim, param, this ]( float & dimValue )
		{

			dimValue = m_start[ dim ] + param * m_intervals[ dim ];

			dim++;

		} );

	return result;
}




float Segment::getMaxInterval() const
{
	return std::fabsf( m_intervals[ m_maxInterPos ] );
}



/**************************************************************************************************
Описание:
определяет величины проекций расстояния между точками на оси конфигурационного пространства,
запоминая номер измерения с максимальной величиной
Аргументы:
Возврат:
**************************************************************************************************/
void Segment::calcIntervals()
{
	auto maxInterval = FLT_MIN;

	for ( auto dim = 0; dim != conf_space_dims; dim++ )
	{
		m_intervals[ dim ] = m_end[ dim ] - m_start[ dim ];

		auto interAbs = std::fabsf( m_intervals[ dim ] );

		if ( maxInterval < interAbs )
		{
			maxInterval = interAbs;
			m_maxInterPos = dim;
		}
	}
}





void Segment::changeStartEnd( const Point & start, const Point & end )
{
	m_start = start;

	m_end = end;

	calcIntervals();
}



/**************************************************************************************************
Описание:
определяет величину параметра такую, чтобы евклидово расстояние между точками не превышало prec_Rad
Аргументы:
* prec_Rad: точность или максимальное евклидово расстояние между выдаваемыми точками отрезка
Возврат:
величина параметра в соответствии с точностью
**************************************************************************************************/
float Segment::calcParamStep( float prec_Rad ) const
{
	auto amountPoints = std::fabsf( m_intervals[ m_maxInterPos ] ) / prec_Rad;

	if ( amountPoints < 1.f )
	{
		return 1.f;
	}

	return 1.f / amountPoints;
}

