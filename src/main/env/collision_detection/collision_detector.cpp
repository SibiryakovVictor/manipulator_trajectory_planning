/**************************************************************************************************
Описание

Определяет наличие пересечение двух OBB (методом areObjectsCollided) в две фазы:
* грубая, используя сравнение расстояния между сферами, в которые заключаются OBB, 
с суммой их радиусов
* точная, с применением SAT (separating axis theorem)
Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/



#include "collision_detector.h"
#include "main/tools/float_operations/float_operations.h"


using namespace motion_planner::env;
using namespace motion_planner::coll;



bool CollisionDetector::areObjectsCollided( const Obb & obj1, const Obb & obj2 ) const
{
	if ( spherePhase( obj1.getCenterPos(), obj2.getCenterPos(), 
		obj1.getMaxSize(), obj2.getMaxSize() ) )
	{
		return false;
	}

	return satPhase( obj1, obj2 );
}



bool CollisionDetector::spherePhase( 
	const Eigen::Vector3f & centerPos1, const Eigen::Vector3f & centerPos2,
	float r1, float r2 ) const
{

	Eigen::Vector3f && dist = centerPos1 - centerPos2;

	float distance = ( dist( 0 ) * dist( 0 ) ) + ( dist( 1 ) * dist( 1 ) ) + ( dist( 2 ) * dist( 2 ) );

	float sumRad = ( 1.1f * r1 + 1.1f * r2 );

	return distance > ( sumRad * sumRad );
}




bool CollisionDetector::satPhase( const Obb & obj1, const Obb & obj2 ) const
{

	Eigen::Vector3f diffPos( obj2.getCenterPos() - obj1.getCenterPos() );

	Eigen::Vector3f ao1row0( obj1.getOrient().row( 0 ) );
	Eigen::Vector3f ao1row1( obj1.getOrient().row( 1 ) );
	Eigen::Vector3f ao1row2( obj1.getOrient().row( 2 ) );
	Eigen::Vector3f ao2row0( obj2.getOrient().row( 0 ) );
	Eigen::Vector3f ao2row1( obj2.getOrient().row( 1 ) );
	Eigen::Vector3f ao2row2( obj2.getOrient().row( 2 ) );


	return !(
		checkSeparatingPlane( diffPos, ao1row0 , obj1.getOrient(), obj2.getOrient(), 
			obj1.getSizes(), obj2.getSizes() )
		||
		checkSeparatingPlane( diffPos, ao1row1, obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() ) 
		||
		checkSeparatingPlane( diffPos, ao1row2, obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() ) 
		||
		checkSeparatingPlane( diffPos, ao2row0, obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() )
		||
		checkSeparatingPlane( diffPos, ao2row1, obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() ) 
		||
		checkSeparatingPlane( diffPos, ao2row2, obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() ) 
		||
		checkSeparatingPlane( diffPos, ao1row0.cross( ao2row0 ), obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() )
		||
		checkSeparatingPlane( diffPos, ao1row0.cross( ao2row1 ), obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() )
		||
		checkSeparatingPlane( diffPos, ao1row0.cross( ao2row2 ), obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() )
		||
		checkSeparatingPlane( diffPos, ao1row1.cross( ao2row0 ), obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() )
		||
		checkSeparatingPlane( diffPos, ao1row1.cross( ao2row1 ), obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() )
		||
		checkSeparatingPlane( diffPos, ao1row1.cross( ao2row2 ), obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() )
		||
		checkSeparatingPlane( diffPos, ao1row2.cross( ao2row0 ), obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() )
		||
		checkSeparatingPlane( diffPos, ao1row2.cross( ao2row1 ), obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() )
		||
		checkSeparatingPlane( diffPos, ao1row2.cross( ao2row2 ), obj1.getOrient(), obj2.getOrient(),
			obj1.getSizes(), obj2.getSizes() )
		);

}




bool CollisionDetector::checkSeparatingPlane(
	const Eigen::Vector3f & diffP, const Eigen::Vector3f & Plane,
	const Eigen::Matrix3f & axisOrient1, const Eigen::Matrix3f & axisOrient2,
	const Obb::Sizes & sizes1, const Obb::Sizes & sizes2
) const
{

	return ( std::fabs( diffP.dot( Plane ) ) > (
		(	std::fabs( ( axisOrient1.row( 0 ) * sizes1.dims[ 0 ] ).dot( Plane ) ) ) +
		(	std::fabs( ( axisOrient1.row( 1 ) * sizes1.dims[ 1 ] ).dot( Plane ) ) ) +
		(	std::fabs( ( axisOrient1.row( 2 ) * sizes1.dims[ 2 ] ).dot( Plane ) ) ) +
		(	std::fabs( ( axisOrient2.row( 0 ) * sizes2.dims[ 0 ] ).dot( Plane ) ) ) +
		(	std::fabs( ( axisOrient2.row( 1 ) * sizes2.dims[ 1 ] ).dot( Plane ) ) ) +
		(	std::fabs( ( axisOrient2.row( 2 ) * sizes2.dims[ 2 ] ).dot( Plane ) ) )	)
		);

}
