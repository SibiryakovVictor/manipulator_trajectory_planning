/**************************************************************************************************
Описание

Определяет наличие пересечение двух OBB (методом areObjectsCollided) в две фазы:
* грубая, используя сравнение расстояния между сферами, в которые заключаются OBB, 
с суммой их радиусов
* точная, с применением SAT (separating axis theorem)
Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/



#pragma once

#include "main/env/primitives/obb/obb.h"
#include "Eigen/Core"



namespace motion_planner
{
	namespace coll
	{
		class CollisionDetector;
	}
}


class motion_planner::coll::CollisionDetector
{
public:

	bool areObjectsCollided( const env::Obb & obj1, const env::Obb & obj2 ) const;

private:

	bool spherePhase( 
		const Eigen::Vector3f & centerPos1, const Eigen::Vector3f & centerPos2,
		float r1, float r2 
	) const;


	bool satPhase( const env::Obb & obj1, const env::Obb & obj2 ) const;


	bool checkSeparatingPlane(
		const Eigen::Vector3f & diffP, const Eigen::Vector3f & Plane,
		const Eigen::Matrix3f & axisOrient1, const Eigen::Matrix3f & axisOrient2,
		const env::Obb::Sizes & sizes1, const env::Obb::Sizes & sizes2
	) const;

};
