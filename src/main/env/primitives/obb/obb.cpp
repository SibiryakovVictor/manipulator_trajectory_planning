/**************************************************************************************************
Описание

Реализация Oriented Bounding Box

Разработчик: Сибиряков Виктор
Заметки
* метод getMaxSize используется для формирования сферы нужного радиуса для грубой проверки
на наличие пересечений между двумя OBB
**************************************************************************************************/


#include "obb.h"


using namespace motion_planner::env;



const Eigen::Matrix3f & Obb::getOrient() const
{
    return m_orient;
}



const Eigen::Vector3f & Obb::getCenterPos() const
{
    return m_centerPos;
}



const Obb::Sizes & Obb::getSizes() const
{
    return m_sizes;
}



float Obb::getMaxSize() const
{
    return m_maxSize;
}





void Obb::setOrient( const Eigen::Matrix3f & orient )
{
    m_orient = orient;
}
void Obb::setOrient( Eigen::Matrix3f && orient )
{
    m_orient = orient;
}



void Obb::setCenterPos( const Eigen::Vector3f & pos )
{
    m_centerPos = pos;
}
void Obb::setCenterPos( Eigen::Vector3f && pos )
{
    m_centerPos = pos;
}



void Obb::setSizes( const Obb::Sizes & sizes )
{
    m_sizes = sizes;

    m_maxSize = *std::max_element( sizes.dims, sizes.dims + dim_space );
}
void Obb::setSizes( Obb::Sizes && sizes )
{
    m_sizes = sizes;

    m_maxSize = *std::max_element( sizes.dims, sizes.dims + dim_space );
}



float Obb::calcMaxSize( const float (&sizesList)[ 3 ] ) const
{
    return *std::max_element( sizesList, sizesList + dim_space );
}





Obb::Obb( const Obb::Sizes & sizes, const Eigen::Vector3f & centerPos ) :
    m_sizes( sizes ),
    m_centerPos( centerPos),
    m_maxSize( calcMaxSize( sizes.dims ) )
{}



Obb::Obb( Obb::Sizes && sizes, Eigen::Vector3f && centerPos ) :
    m_sizes( sizes ),
    m_centerPos( centerPos ),
    m_maxSize( calcMaxSize( sizes.dims ) )
{}



Obb::Obb( const Obb::Sizes & sizes, const Eigen::Vector3f & centerPos, 
    const Eigen::Matrix3f & orient ) :
    m_sizes( sizes ),
    m_centerPos( centerPos ),
    m_orient( orient ),
    m_maxSize( calcMaxSize( sizes.dims ) )
{}



Obb::Obb( Obb::Sizes && sizes, Eigen::Vector3f && centerPos, Eigen::Matrix3f && orient ) :
    m_sizes( sizes ),
    m_centerPos( centerPos ),
    m_orient( orient ),
    m_maxSize( calcMaxSize( sizes.dims ) )
{}



Obb::Obb( const Obb & Obb ) :
    m_sizes( Obb.m_sizes ),
    m_centerPos( Obb.m_centerPos ),
    m_orient( Obb.m_orient ),
    m_maxSize( calcMaxSize( Obb.m_sizes.dims ) )
{}



Obb & Obb::operator=( Obb && Obb )
{
    m_sizes = Obb.m_sizes;
    m_maxSize = calcMaxSize( Obb.m_sizes.dims );

    m_centerPos = Obb.m_centerPos;

    m_orient = Obb.m_orient;

    return *this;
}




Obb & Obb::operator=( const Obb & Obb )
{
    if ( this == &Obb )
    {
        return *this;
    }

    m_sizes = Obb.m_sizes;
    m_maxSize = calcMaxSize( Obb.m_sizes.dims );

    m_centerPos = Obb.m_centerPos;

    m_orient = Obb.m_orient;

    return *this;
}
