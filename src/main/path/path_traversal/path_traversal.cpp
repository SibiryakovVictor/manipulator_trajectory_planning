#include "path_traversal.h"


using namespace motion_planner;
using namespace motion_planner::path;



Path & PathTraversal::getPathStorage()
{
    return m_path;
}



void PathTraversal::setPointsAmount( uint16_t pathLength )
{
	m_pointsAmount = pathLength;
}



uint16_t PathTraversal::getPointsAmount() const
{
	return m_pointsAmount;
}



float PathTraversal::getStep() const
{
	return m_step;
}



bool PathTraversal::isPathPassed() const
{
	return ( m_curPoint > m_pointsAmount - 2 );
}



config_space::Point PathTraversal::getNextPointPath() const
{

	if ( m_trajIt.isPassed() )
	{
		m_curPoint++;

		if ( m_curPoint > ( m_pointsAmount - 2 ) )
		{
			m_curPoint = m_pointsAmount - 1;

			return m_trajIt.end();
		}

		m_trajIt.setTrajectory(
			m_nodesList.getNodeConfig( m_path.sections[ m_curPoint ].point ),
			m_nodesList.getNodeConfig( m_path.sections[ m_curPoint + 1 ].point ),
			m_step
		);

		return m_trajIt.getNextPoint();
	}

	return m_trajIt.getNextPoint();

}



void PathTraversal::resetPointGetter() const
{
	m_curPoint = 0;

	m_trajIt.setTrajectory(
		m_nodesList.getNodeConfig( m_path.sections[ m_curPoint ].point ),
		m_nodesList.getNodeConfig( m_path.sections[ m_curPoint + 1 ].point ),
		m_step
	);
}



void PathTraversal::reset()
{
	PathElement fillElem{ UINT16_MAX, UINT8_MAX };
	std::fill( m_path.sections, m_path.sections + m_pointsAmount, fillElem );

	m_pointsAmount = 0;

	m_curPoint = 0;

	m_trajIt.resetStep();
}



PathElement PathTraversal::getCurPathElem() const
{
	return m_path.sections[ m_curPoint ];
}



PathElement PathTraversal::getPathElem( uint16_t elemIndex ) const
{
	return m_path.sections[ elemIndex ];
}



void PathTraversal::changeStep( float step ) const
{
	m_step = step;

	resetPointGetter();
}



void PathTraversal::setNextPathElem() const
{
	m_curPoint++;
	if ( m_curPoint > ( m_pointsAmount - 2 ) )
	{
		m_curPoint = m_pointsAmount - 1;

		return;
	}

	m_trajIt.setTrajectory(
		m_nodesList.getNodeConfig( m_path.sections[ m_curPoint ].point ),
		m_nodesList.getNodeConfig( m_path.sections[ m_curPoint + 1 ].point ),
		m_step
	);

}



PointId PathTraversal::getCurPos() const
{
	return m_curPoint;
}

