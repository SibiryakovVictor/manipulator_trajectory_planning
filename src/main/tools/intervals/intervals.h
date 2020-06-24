#pragma once

#include "main/env/manipulator/manip_parameters.h"
#include "main/tools/float_operations/float_operations.h"
#include "main/config_space/point/point.h"


namespace motion_planner
{
    namespace tools
    {
        class Interval;

        template < unsigned arrayLength >
        class IntervalArray;     

        class RangeLinksMotion;
    }
}


class motion_planner::tools::Interval
{

public:

    explicit Interval() :
        m_lb( 0 ), 

        m_hb( 0 )
    {}

    explicit Interval( float lb, float hb ) :
        m_lb( lb ), 

        m_hb( hb )
    {}


    float getlb() const
    {
        return m_lb;
    }

    float gethb() const
    {
        return m_hb;
    }

    void setlb( float newlb )
    {
        m_lb = newlb;
    }

    void sethb( float newhb )
    {
        m_hb = newhb;
    }

    bool isInInterval( float number )
    {
        return ( number > m_lb ) && ( number < m_hb );
    }

private:

    float m_lb;
    float m_hb;

};


template < unsigned arrayLength >
class motion_planner::tools::IntervalArray
{

public:

    struct IntervalArrayData
    {
        Interval m_arrayIntr[ arrayLength ];
    };

    explicit IntervalArray() :
        m_intervals() 
    {}


    explicit IntervalArray( const IntervalArrayData & obj ) :
        m_intervals( obj )
    {}


    Interval operator[]( unsigned pos ) const
    {
        return m_intervals.m_arrayIntr[ pos ];
    }


    Interval & operator[]( unsigned pos )
    {
        return m_intervals.m_arrayIntr[ pos ];
    }


private:

    IntervalArrayData m_intervals;

};

typedef motion_planner::tools::IntervalArray< motion_planner::manip::links_groups > IntervalsAllDims;

const IntervalsAllDims realRangesMotion( IntervalsAllDims::IntervalArrayData{ {

        motion_planner::tools::Interval( flt_op::cvtDegToRad( -180.f ), flt_op::cvtDegToRad( 180.f ) ),

        motion_planner::tools::Interval( flt_op::cvtDegToRad( -90.f ), flt_op::cvtDegToRad( 90.f ) ),

        motion_planner::tools::Interval( flt_op::cvtDegToRad( -180.f ), flt_op::cvtDegToRad( 180.f ) ),

        motion_planner::tools::Interval( flt_op::cvtDegToRad( -180.f ), flt_op::cvtDegToRad( 180.f ) ),

        motion_planner::tools::Interval( flt_op::cvtDegToRad( -180.f ), flt_op::cvtDegToRad( 180.f ) ),

        motion_planner::tools::Interval( flt_op::cvtDegToRad( -14.f ), flt_op::cvtDegToRad( 50.f ) ),

        motion_planner::tools::Interval( flt_op::cvtDegToRad( -50.f ), flt_op::cvtDegToRad( 14.f ) )

    } }
);


class motion_planner::tools::RangeLinksMotion
{

public:

    static const IntervalsAllDims & get()
    {
        return m_ranges;
    }

    static void optimizeRangesLinksMotion( config_space::Point & startConfig,
                                            config_space::Point & goalConfig )
    {
        m_ranges = realRangesMotion;

        for ( uint16_t curDim = 0; curDim != conf_space_dims; curDim++ )
        {

            if ( ! canBothDirections[ curDim ] )
            {
                continue;
            }

            if ( ( startConfig[ curDim ] * goalConfig[ curDim ] ) > 0.f )
            {
                continue;
            }

            float distFirstDir = std::fabsf( goalConfig[ curDim ] - startConfig[ curDim ] );

            float distSecondDir = 0.f;
            if ( startConfig[ curDim ] > 0.f )
            {
                distSecondDir = 
                    std::fabsf( flt_op::mathPI - startConfig[ curDim ] ) + 
                    std::fabsf( -flt_op::mathPI - goalConfig[ curDim ] );

                if ( distSecondDir < distFirstDir )
                {
                    goalConfig[ curDim ] = 2 * flt_op::mathPI +
                        goalConfig[ curDim ];

                    m_ranges[ curDim ].setlb( 0.f );
                    m_ranges[ curDim ].sethb( flt_op::mathPI * 2.f );
                }
            }
            else
            {
                distSecondDir = 
                    std::fabsf( -flt_op::mathPI - startConfig[ curDim ] ) + 
                    std::fabsf( flt_op::mathPI - goalConfig[ curDim ] );

                if ( distSecondDir < distFirstDir )
                {
                    startConfig[ curDim ] = 2 * flt_op::mathPI +
                        startConfig[ curDim ];

                    m_ranges[ curDim ].setlb( 0.f );
                    m_ranges[ curDim ].sethb( flt_op::mathPI * 2.f );
                }
            }

        }
    }

    static void checkPossibleDirections()
    {

        std::fill( canBothDirections, canBothDirections + manip::links_groups,
            false );

        float lowBorder = flt_op::mathPI * -0.98f;
        float highBorder = flt_op::mathPI * 0.98f;

        for ( uint16_t curLink = 0; curLink != manip::links_groups; curLink++ )
        {

            Interval curRange = realRangesMotion[ curLink ];

            canBothDirections[ curLink ] = 
                ( curRange.getlb() < lowBorder ) && ( curRange.gethb() > highBorder ); 

        }

    }

private:

    static IntervalsAllDims m_ranges;

    static bool canBothDirections[ manip::links_groups ];

};
