/**************************************************************************************************
Описание

Расширение класса Obb с возможностью вращения тела относительно точки, через которую будет
проходить ось вращения тела

Разработчик: Сибиряков Виктор
Заметки
* комментарий к методу rotateAroundAnchor
**************************************************************************************************/



#include "attached_detail.h"
#include "main/env/motion_common/motion_common.h"


using namespace motion_planner::env;


/**************************************************************************************************
Описание:
выполняет поворот тела на угол angle относительно оси rotAxis, проходящей через точку m_mountPos.
!!! ВНИМАНИЕ !!!: каждый новый поворот выполняется относительно начальной позиции
(то есть без учёта предыдущего поворота), ориентация тела в пространстве устанавливается такой,
как будто выполнен только этот поворот.
Аргументы:
* conf: проверяемая конфигурация
Возврат: имеет ли пересечения данная конфигурация
**************************************************************************************************/
void AttachedDetail::rotateAroundAnchor( float angle_Rad, Eigen::Vector3f && rotAxis )
{
    auto && rotMat = getRotMat( angle_Rad, rotAxis );

    Obb::setCenterPos( m_mountPos + rotMat * ( m_initCenterPos - m_mountPos ) );

    Obb::setOrient( rotMat );
}

void AttachedDetail::rotateAroundAnchor( float angle_Rad, const Eigen::Vector3f & rotAxis )
{
    auto && rotMat = getRotMat( angle_Rad, rotAxis );

    Obb::setCenterPos( m_mountPos + rotMat * ( m_initCenterPos - m_mountPos ) );

    Obb::setOrient( rotMat );
}




void AttachedDetail::resetOrient()
{
    Obb::setCenterPos( m_initCenterPos );

    Obb::setOrient( Eigen::Matrix3f::Identity() );
}



AttachedDetail & AttachedDetail::operator=( const Obb & Obb )
{
    Obb::operator=( Obb );

    return *this;
}



AttachedDetail::AttachedDetail( const Obb & obbObj ) :
    Obb( obbObj )
{}

AttachedDetail::AttachedDetail( Obb && obbObj ) :
    Obb( obbObj )
{}

AttachedDetail::AttachedDetail( const Obb & obbObj, const Eigen::Vector3f & mountPos ) :
    Obb( obbObj ), m_mountPos( mountPos )
{}

AttachedDetail::AttachedDetail( Obb && obbObj, Eigen::Vector3f && mountPos ) :
    Obb( obbObj ), m_mountPos( mountPos )
{}



