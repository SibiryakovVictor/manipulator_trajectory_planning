#pragma once

#include "main/env/env_def.h"


namespace motion_planner
{
    namespace env
    {
        class Obb;
    }
}


class motion_planner::env::Obb
{
public:

    struct Sizes
    {
        float dims[ 3 ] = { 0.f, 0.f, 0.f };

        Sizes( const float (&dimValues)[ 3 ] )
        {
            std::copy( dimValues, dimValues + 3, dims );
        }
    };

    const Orient & getOrient() const;

    const Position & getCenterPos() const;

    const Obb::Sizes & getSizes() const;

    float getMaxSize() const;



    void setOrient( const Orient & orient );
    void setOrient( Orient && orient );

    void setCenterPos( const Position & pos );
    void setCenterPos( Position && pos );

    void setSizes( const Obb::Sizes & sizes );
    void setSizes( Obb::Sizes && sizes );



    explicit Obb()
    {}

    explicit Obb( const Obb::Sizes & sizes, const Position & centerPos );

    explicit Obb( Obb::Sizes && sizes, Position && centerPos );

    explicit Obb( const Obb::Sizes & sizes, const Position & centerPos, 
        const Orient & orient );

    explicit Obb( Obb::Sizes && sizes, Position && centerPos, Orient && orient );

    Obb( const Obb & obb );

    Obb & operator=( Obb && obb );

    Obb & operator=( const Obb & obb );

private:

    Obb::Sizes m_sizes = Obb::Sizes( { 0.f, 0.f, 0.f } );

    Position m_centerPos = Eigen::Vector3f::Zero();

    Orient m_orient = Eigen::Matrix3f::Identity();

    float m_maxSize = 0.f;


    float calcMaxSize( const float (&sizesList)[ 3 ] ) const;
};
