/**************************************************************************************************
Описание

Определение точки конфигурационного пространства

Разработчик: Сибиряков Виктор
Заметки
* определение расстояния между точками - методы calcDist и calcDistNoSqrt
* определение равенства точек - operator==
**************************************************************************************************/



#pragma once

#include <cinttypes>
#include <algorithm>
#include <numeric>

#include "main/config_space/conf_space_dims.h"

namespace motion_planner
{
	namespace config_space
	{
		struct PointData;

		class Point;		
	}
}



struct motion_planner::config_space::PointData
{
	float dimVals[ conf_space_dims ];


	PointData()
	{
		std::fill( dimVals, dimVals + conf_space_dims, 0.f );
	}


	PointData( float value )
	{
		std::fill( dimVals, dimVals + conf_space_dims, value );
	}


	PointData( const float (&dimValues)[ conf_space_dims ] )
	{
		std::copy( dimValues, dimValues + conf_space_dims,
			dimVals );
	}


	PointData( const PointData & pData )
	{
		std::copy( pData.dimVals, pData.dimVals + conf_space_dims,
			dimVals );
	}


	PointData & operator=( const PointData & pData )
	{
		if ( this == &pData )
		{
			return *this;
		}

		std::copy( pData.dimVals, pData.dimVals + conf_space_dims,
			dimVals );

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
		return m_pointData.dimVals;
	}

	float * end()
	{
		return m_pointData.dimVals + conf_space_dims;
	}

	float const * begin() const
	{
		return m_pointData.dimVals;
	}

	float const * end() const
	{
		return m_pointData.dimVals + conf_space_dims;
	}

	float & operator[]( uint16_t index )
	{
		return m_pointData.dimVals[ index ];
	}

	float operator[]( uint16_t index ) const
	{
		return m_pointData.dimVals[ index ];
	}

	void cvtDegsToRads();

	static float calcDistance( const Point & p1, const Point & p2 );

	static float calcDistNoSqrt( const Point & p1, const Point & p2 );

	friend bool operator==( const Point & p1, const Point & p2 );

private:

	PointData m_pointData;
};
