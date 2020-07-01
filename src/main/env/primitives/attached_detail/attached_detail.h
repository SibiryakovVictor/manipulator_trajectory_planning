/**************************************************************************************************
Описание

Расширение класса Obb с возможностью вращения тела относительно точки, через которую будет
проходить ось вращения тела

Разработчик: Сибиряков Виктор
Заметки
* комментарий к методу rotateAroundAnchor (в attached_detail.cpp)
**************************************************************************************************/



#pragma once

#include "main/env/primitives/obb/obb.h"



namespace motion_planner
{
    namespace env
    {
        class AttachedDetail;
    }
}


class motion_planner::env::AttachedDetail : public motion_planner::env::Obb
{

public:

    void rotateAroundAnchor( float angle_Rad, const Eigen::Vector3f & rotAxis );
    void rotateAroundAnchor( float angle_Rad, Eigen::Vector3f && rotAxis );

    void resetOrient();

    explicit AttachedDetail()
    {}

    explicit AttachedDetail( const Obb & Obb );
    explicit AttachedDetail( Obb && Obb );

    explicit AttachedDetail( const Obb & Obb, const Eigen::Vector3f & mountPos );
    explicit AttachedDetail( Obb && Obb, Eigen::Vector3f && mountPos );

    AttachedDetail & operator=( const Obb & Obb );

private:

    Eigen::Vector3f m_mountPos = Obb::getCenterPos();

    Eigen::Vector3f m_initCenterPos = Obb::getCenterPos();

};

