#pragma once

#include "manip_parameters.h"
#include "main/env/env_def.h"

#include "main/env/primitives/obb/obb.h"
#include "main/env/info_loader/info_loader.h"
#include "main/tools/intervals/intervals.h"



namespace motion_planner
{
    class ConfigManager;

    namespace env
    {
        class Manipulator;
    }
}                                    


class motion_planner::env::Manipulator
{
public:

    friend class motion_planner::ConfigManager;

    static const uint16_t biasGroupId = UINT16_MAX / 4;
    static const uint16_t biasLinkId = 0;

    explicit Manipulator( ManipInitInfo && manipLinksInfo = load::getManipInitInfo() );

    uint16_t setConfigCurrentLink( const float & config );

    const Obb & getBodyCurLink() const;

    const Obb & getBody( uint16_t linkId ) const;

    const Obb & getBodyGroup( uint16_t groupId ) const;

    uint16_t getCurGroupId() const;

    uint16_t getGroupId( uint16_t linkId ) const;

    uint16_t getLinkType( uint16_t linkId ) const;

    uint16_t getLinkNextGroup( uint16_t groupId ) const;

    uint16_t getFirstLinkGroup( uint16_t groupId ) const;

    uint16_t getFirstLinkComplex( uint16_t complex ) const;

    void moveGroupCounter();

    void reset();


private:

    ManipInitInfo::LinkInitInfo m_initLinksInfo[ manip::links_amount ]; 

    Obb m_links[ manip::links_amount ];
    Obb m_groups[ manip::links_complex_amount ];

    Position m_mountPos[ manip::links_amount ];

    uint16_t m_linkId = 0;
    uint16_t m_groupId = 0;
    uint16_t m_orientCounter = 0;

    uint16_t m_linkType[ manip::links_amount ] { 0 };
    manip::OrientDependency m_linkOrientDep[ manip::links_amount ];
    uint16_t m_firstLinkGroup[ manip::links_groups ] { 0 };
    uint16_t m_groupsPos[ manip::links_groups ] { 0 };
    uint16_t m_linkInGroup[ manip::links_amount ] { 0 };
    uint16_t m_complexFirstLink[ manip::links_complex_amount ] { 0 };

    void rotateCenterLink( uint16_t linkId, const float & config );

    void setConfig( uint16_t linkId, const float & config );


    void alignMountCenterPos( float angle, uint16_t refLinkId, 
        uint16_t startAlignPos, uint16_t endAlignPos );

    void initMountCenterPos( float angle );


    void appendInitPos( uint16_t linkId );
    Orient appendInitOrient( uint16_t linkId, const env::Orient & result );

    Obb::Sizes calcComplexSize( uint16_t linkId ) const;

    Position calcComplexPos( uint16_t linkId ) const;

    void defineLinksType();

    void defineLinksBelongGroups();

    void defineOrientDependencies();

    void defineFirstLinkGroups();

    void defineGroupsPos();

    void defineFirstLinkComplex();


    void changeBody( uint16_t linkId, const Obb & body );
};

