/**************************************************************************************************
Описание

Класс, основное назначение которого - размещение в пространстве ограничивающих объемов звеньев
в соответствии с заданной конфигурацией манипулятора

Разработчик: Сибиряков Виктор
Заметки
* Configurator - дружественный класс, чтобы менять ограничивающий объем звена
**************************************************************************************************/


#pragma once

#include "manip_parameters.h"
#include "main/env/env_def.h"

#include "main/env/primitives/obb/obb.h"
#include "main/env/info_loader/info_loader.h"
#include "main/tools/intervals/intervals.h"



namespace motion_planner
{
    class Configurator;

    namespace env
    {
        class ManipulatorController;
    }
}                                    


class motion_planner::env::ManipulatorController
{

public:

    friend class motion_planner::Configurator;

    static const uint16_t biasGroupId = UINT16_MAX / 4;
    static const uint16_t biasLinkId = 0;

    explicit ManipulatorController( ManipInitInfo && manipLinksInfo = load::getManipInitInfo() );

    void setConfigCurrentLink( const float & config );

    const Obb & getBody( uint16_t id ) const;

    uint16_t getCurGroupId() const;

    uint16_t getGroupId( uint16_t linkId ) const;

    uint16_t getLinkType( uint16_t linkId ) const;

    uint16_t getLinkNextGroup( uint16_t groupId ) const;

    uint16_t getFirstLinkGroup( uint16_t groupId ) const;

    void moveGroupCounter();

    void reset();


private:

    ManipInitInfo::LinkInitInfo m_initLinksInfo[ manip::links_amount ]; 

    Obb m_links[ manip::links_amount ];
    Obb m_complex[ manip::links_complex_amount ];

    Eigen::Vector3f m_mountPos[ manip::links_amount ];

    uint16_t m_linkId = 0;
    uint16_t m_complexId = 0;
    uint16_t m_orientCounter = 0;

    uint16_t m_linkType[ manip::links_amount ] { 0 };
    manip::OrientDependency m_linkOrientDep[ manip::links_amount ];
    uint16_t m_firstLinkGroup[ manip::links_groups ] { 0 };
    uint16_t m_groupsPosInComplex[ manip::links_groups ] { 0 };
    uint16_t m_linkInGroup[ manip::links_amount ] { 0 };


    void rotateCenterLink( uint16_t linkId, const float & config );

    void setConfig( uint16_t linkId, const float & config );


    void alignMountCenterPos( float angle, uint16_t refLinkId, 
        uint16_t startAlignPos, uint16_t endAlignPos );

    void initMountCenterPos( float angle );


    void appendInitPos( uint16_t linkId );
    Eigen::Matrix3f appendInitOrient( uint16_t linkId, const Eigen::Matrix3f & result );

    Obb::Sizes calcComplexSize( uint16_t linkId ) const;

    Eigen::Vector3f calcComplexPos( uint16_t linkId ) const;

    void defineLinksType();

    void defineLinksBelongGroups();

    void defineOrientDependencies();

    void defineFirstLinkGroups();

    void defineGroupsPos();


    void changeBody( uint16_t linkId, const Obb & body );
};

