#pragma once

#include <algorithm>
#include <numeric>

#include "main/config_space/conf_space_dims.h"



namespace motion_planner
{
	namespace config_space
	{
		struct PointData;

		class Point;		

		namespace point
		{
			void cvtDegsToRads( Point & p );
		}
	}
}



struct motion_planner::config_space::PointData
{
	float dimensionValues[ conf_space_dims ];


	PointData()
	{
		std::fill( dimensionValues, dimensionValues + conf_space_dims, 0.f );
	}


	PointData( float value )
	{
		std::fill( dimensionValues, dimensionValues + conf_space_dims, value );
	}


	PointData( const float (&dimValues)[ conf_space_dims ] )
	{
		std::copy( dimValues, dimValues + conf_space_dims,
			dimensionValues );
	}


	PointData( const PointData & pData )
	{
		std::copy( pData.dimensionValues, pData.dimensionValues + conf_space_dims,
			dimensionValues );
	}


	PointData & operator=( const PointData & pData )
	{
		if ( this == &pData )
		{
			return *this;
		}

		std::copy( pData.dimensionValues, pData.dimensionValues + conf_space_dims,
			dimensionValues );

		return *this;
	}

};

class motion_planner::config_space::Point
{

public:

	explicit Point( const PointData & pointData ) :

		m_pointData( pointData )
	{}


	explicit Point( PointData && pointData ) :

		m_pointData( pointData )
	{}


	explicit Point() :

		m_pointData( 0.f )
	{}

	explicit Point( const float (&dimValues)[ conf_space_dims ] ) :
		m_pointData( dimValues )
	{}

	Point( const Point & point ) :

		m_pointData( point.m_pointData )
	{}

		

	Point & operator=( const Point & point )
	{
		if ( &point == this )
		{
			return *this;
		}

		m_pointData = point.m_pointData;

		return *this;
	}

	float * begin()
	{
		return m_pointData.dimensionValues;
	}

	float * end()
	{
		return m_pointData.dimensionValues + conf_space_dims;
	}

	float const * begin() const
	{
		return m_pointData.dimensionValues;
	}

	float const * end() const
	{
		return m_pointData.dimensionValues + conf_space_dims;
	}

	float & operator[]( uint16_t index )
	{
		return m_pointData.dimensionValues[ index ];
	}

	float operator[]( uint16_t index ) const
	{
		return m_pointData.dimensionValues[ index ];
	}

	void alignToGrid( const float gridSize );

	static float calcDistance( const Point & p1, const Point & p2 );

	static float calcDistNoSqrt( const Point & p1, const Point & p2 );

	friend bool operator==( const Point & p1, const Point & p2 );

private:

	PointData m_pointData;
};

