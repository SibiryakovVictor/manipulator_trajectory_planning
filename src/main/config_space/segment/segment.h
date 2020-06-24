#pragma once

#include "main/config_space/point/point.h"


namespace motion_planner
{
    namespace config_space
    {
        class Segment;
    }
}


class motion_planner::config_space::Segment
{
public:

    explicit Segment()
    {}

    explicit Segment( const Point & start, const Point & end ) :
        m_start( start ), m_end( end )
    {
        calcIntervals();
    }

    explicit Segment( Point && start, Point && end ) :
        m_start( start ), m_end( end )
    {
        calcIntervals();
    }

    Point getPoint( float param ) const;

    float getMaxInterval() const;

    void changeStartEnd( const Point & start, const Point & end );

    float calcParamStep( float prec ) const;

private:

    Point m_start;
    Point m_end;

    float m_intervals[ conf_space_dims ] { 0.f };

    uint8_t m_maxInterPos = 0;

    void calcIntervals();
};
