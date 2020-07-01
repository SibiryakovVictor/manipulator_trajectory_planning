/**************************************************************************************************
Описание

Генератор последовательных точек найденной траектории

Разработчик: Сибиряков Виктор
Заметки
* под шагом прохождения траекториии понимается максимально евклидово расстояние в конфигурационном пространстве
между точками траектории
**************************************************************************************************/


#pragma once

#include "main/path/path_typedefs.h"
#include "main/path/path_parameters.h"

#include "main/config_space/point/point.h"
#include "main/path/trajectory_iterator/trajectory_iterator.h"
#include "main/config_space/graph/i_nodes_list.h"



namespace motion_planner
{
	namespace path
	{
		class PathGenerator;
	}
}


class motion_planner::path::PathGenerator
{
public:

	explicit PathGenerator( const config_space::graph::INodesList & nodesList, 
		float step_Degree ) :
		m_nodesList( nodesList ),
		m_step_Degree( step_Degree )
	{
		std::fill( m_path.sections, m_path.sections + path::length_limit, empty_path_elem );
	}

	config_space::Point getNextPointPath() const;

	void resetPointGetter() const;

	void reset();

	Path & getPathStorage();

	void setPointsAmount( PathElemId pathNodesAmount );
	PathElemId getPointsAmount() const;

	bool isPathPassed() const;

	void changeStep( float step_Degree );
	float getStep_Degree() const;


	PathElement getCurPathElem() const;

	PathElement getPathElem( PathElemId elemPos ) const;


private:

	Path m_path;

	PathElemId m_pointsAmount = 0;

	float m_step_Degree = 1.f;

	float m_maxDimVal_Rad = calcMaxDimValue_Rad( m_step_Degree );

	mutable PathElemId m_curElem = 0;

	mutable TrajectoryIterator m_trajIt;

	const config_space::graph::INodesList & m_nodesList;

	void setCurElemTrajIt() const; 
};
