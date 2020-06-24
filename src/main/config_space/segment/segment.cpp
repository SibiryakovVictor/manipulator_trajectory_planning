#include "segment.h"

#include <cfloat>
#include <algorithm>


using namespace motion_planner::config_space;



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




void Segment::calcIntervals()
{

	auto maxInterval = FLT_MIN;

	auto dim = 0;

	std::for_each( m_intervals, m_intervals + conf_space_dims, 
		[ &dim, &maxInterval, this ]( float& interval )
		{

			interval = m_end[ dim ] - m_start[ dim ];

			auto interAbs = std::fabsf( interval );

			if ( maxInterval < interAbs )
			{
				maxInterval = interAbs;
				m_maxInterPos = dim;
			}

			dim++;

		} );

}





void Segment::changeStartEnd( const Point & start, const Point & end )
{

	m_start = start;

	m_end = end;

	calcIntervals();

}




float Segment::calcParamStep( float prec ) const
{
	auto amountPoints = std::fabsf( m_intervals[ m_maxInterPos ] ) / prec;

	if ( amountPoints < 1.f )
	{
		return 1.f;
	}

	return 1.f / amountPoints;
}
