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
		const env::Position & centerPos1, const env::Position & centerPos2,
		float r1, float r2 
	) const;


	bool satPhase( const env::Obb & obj1, const env::Obb & obj2 ) const;


	bool checkSeparatingPlane(
		const env::Position & diffP, const env::Position & Plane,
		const env::Orient & axisOrient1, const env::Orient & axisOrient2,
		const env::Obb::Sizes & sizes1, const env::Obb::Sizes & sizes2
	) const;

	Eigen::Vector3f cross( const Eigen::Vector3f & v1, const Eigen::Vector3f & v2 ) const;
};
