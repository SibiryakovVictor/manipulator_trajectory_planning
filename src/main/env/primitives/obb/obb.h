/**************************************************************************************************
Описание

Реализация Oriented Bounding Box

Разработчик: Сибиряков Виктор
Заметки
* метод getMaxSize используется для формирования сферы нужного радиуса для грубой проверки
на наличие пересечений между двумя OBB
**************************************************************************************************/


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

    const Eigen::Matrix3f & getOrient() const;

    const Eigen::Vector3f & getCenterPos() const;

    const Obb::Sizes & getSizes() const;

    float getMaxSize() const;



    void setOrient( const Eigen::Matrix3f & orient );
    void setOrient( Eigen::Matrix3f && orient );

    void setCenterPos( const Eigen::Vector3f & pos );
    void setCenterPos( Eigen::Vector3f && pos );

    void setSizes( const Obb::Sizes & sizes );
    void setSizes( Obb::Sizes && sizes );



    explicit Obb()
    {}

    explicit Obb( const Obb::Sizes & sizes, const Eigen::Vector3f & centerPos );

    explicit Obb( Obb::Sizes && sizes, Eigen::Vector3f && centerPos );

    explicit Obb( const Obb::Sizes & sizes, const Eigen::Vector3f & centerPos, 
        const Eigen::Matrix3f & orient );

    explicit Obb( Obb::Sizes && sizes, Eigen::Vector3f && centerPos, Eigen::Matrix3f && orient );

    Obb( const Obb & obb );

    Obb & operator=( Obb && obb );

    Obb & operator=( const Obb & obb );

private:

    Obb::Sizes m_sizes = Obb::Sizes( { 0.f, 0.f, 0.f } );

    Eigen::Vector3f m_centerPos = Eigen::Vector3f::Zero();

    Eigen::Matrix3f m_orient = Eigen::Matrix3f::Identity();

    float m_maxSize = 0.f;


    float calcMaxSize( const float (&sizesList)[ 3 ] ) const;
};

