#include "trajectory_iterator.h"

using namespace motion_planner;
using namespace motion_planner::path;



void TrajectoryIterator::setTrajectory( const config_space::Point & startPoint, 
	const config_space::Point & endPoint,
	float step )
{

	m_segment.changeStartEnd( startPoint, endPoint );

	m_paramStep = m_segment.calcParamStep( step );

	m_curParamValue = 0.f;

}




void TrajectoryIterator::changePrec( float prec )
{

	m_paramStep =  m_segment.calcParamStep( prec );

	m_curParamValue = 0.f;

}




config_space::Point TrajectoryIterator::getNextPoint()
{
	if ( isPassed() )
	{
		return end();
	}

	auto paramValue = m_curParamValue;

	m_curParamValue += m_paramStep;

	return m_segment.getPoint( paramValue );
}




config_space::Point TrajectoryIterator::start() const
{
	return m_segment.getPoint( 0.f );
}




config_space::Point TrajectoryIterator::end() const
{
	return m_segment.getPoint( 1.f );
}




bool TrajectoryIterator::isPassed() const
{
	return ( m_curParamValue > 0.995f );
}




void TrajectoryIterator::resetStep()
{
	m_curParamValue = 0.f;
}

