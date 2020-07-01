/**************************************************************************************************
Описание

Реализация класса, проверяющего принадлежность точек и отрезков конфигурационного пространства 
к его свободным регионам (к тем, в которых не происходит пересечений с препятствиями и звеньев между собой)

Разработчик: Сибиряков Виктор
Заметки
* максимальное евклидово расстояние между проверяемыми точками отрезка по умолчанию: 
константа check_traj_prec_Degrees в "path/path_parameters.h", но можно задать с помощью
setSegCheckPrec (в градусах);
* Configurator - дружественный класс, чтобы рулить таблицей сталкиваемых объектов
**************************************************************************************************/



#include "free_cspace_validator.h"
#include "main/tools/float_operations/float_operations.h"
#include "main/path/path_parameters.h"

#include <cfloat>

using namespace motion_planner::config_space;
using namespace motion_planner::path;
using namespace motion_planner::env;
using namespace motion_planner::coll;




/**************************************************************************************************
Описание:
проверяет точки, входящие в отрезок с концами start и end, на наличие пересечений в две фазы,
сначала грубой фазой (checkSegmentRoughPhase), где проверяется возрастающее количество точек отрезка 
(4 и 15), после чего точной фазой (с точностью m_segDimStep_Rad для каждого измерения) 
Аргументы:
* start, end: концы отрезка
Возврат: имеет ли отрезок точки с пересечениями
**************************************************************************************************/
bool FreeCSpaceValidator::isSegmentInFreeCSpace( const Point & start, const Point & end )
{
    Segment segment( start, end );

    if ( ! checkSegmentRoughPhase( segment ) )
    {
        return false;
    }

    return 
        ( ! isSegmentCollided_Precision( segment, segment.calcParamStep( m_segDimStep_Rad ) ) );
}



bool FreeCSpaceValidator::checkSegmentRoughPhase( const Segment & segment )
{
    static const auto stages = 2;
    static const unsigned amount_points[ stages ] = { 4, 15 };
    
    for ( auto stage = 0; stage != stages; stage++ )
    {
        if ( isSegmentCollided_Points( segment, amount_points[ stage ] ) )
        {
            return false;
        }
    }    

    return true;
}



bool FreeCSpaceValidator::isSegmentCollided_Points( const Segment & segment, uint16_t pointsCheck )
{
    return isSegmentCollided_Precision( segment, 1.f / static_cast< float >( pointsCheck ) );
}



bool FreeCSpaceValidator::isSegmentCollided_Precision( const Segment & segment, float prec )
{
    static const auto param_border_value = 1.f;

    auto step = prec;

    while ( step < param_border_value )
    {
        auto point( segment.getPoint( step ) );

        if ( ! isConfigInFreeCSpace( point ) )
        {
            return true;
        }

        step += prec;
    }

    return false;
}





/**************************************************************************************************
Описание:
проверяет отсутствие пересечений в конфигурации, проверяя звено/группу звеньев сразу, как только
задана его/их конфигурация, поэтому важно правильно заполнить таблицу в 
"env/collision_detection/collision_check_pairs.h"
Аргументы:
* conf: проверяемая конфигурация
Возврат: имеет ли пересечения данная конфигурация
**************************************************************************************************/
bool FreeCSpaceValidator::isConfigInFreeCSpace( const Point & conf )
{
    static config_space::Point prevConfig( config_space::PointData( FLT_MAX ) );
    static bool isPrevResultCorrect = false;

    const auto threshold_config_change = m_segDimStep_Rad * 0.5f;

    m_manip.reset();

    uint8_t configPos = 0;
    uint16_t pairPos = 0;

    auto isDiffFromPrev = false;

    while ( configPos < conf_space_dims )
    {
        m_manip.setConfigCurrentLink( conf[ configPos ] );

        if ( isPrevResultCorrect && ( ! isDiffFromPrev ) )
        {
            if ( std::fabsf( prevConfig[ configPos ] - conf[ configPos ] ) < 
                threshold_config_change )
            {
                pairPos = moveToNextGroup( pairPos, configPos );

                configPos++;

                m_manip.moveGroupCounter();

                continue;
            }
            else
            {
                isDiffFromPrev = true;
            }
        }

        configPos++;

        while ( pairPos != m_listPosConfigChange[ configPos ] )
        {
            if ( ! m_collTable.isPairActivated( pairPos ) )
            {
                pairPos++;

                continue;
            }

            auto pair = m_collTable.getPair( pairPos );

            bool checkResult = true;
            if ( pair.body2 >= env::Obstacles::biasObstId )
            {
                checkResult = checkManipVsObst( pair.body1, pair.body2 );
            }
            else
            {
                checkResult = checkManipVsManip( pair.body1, pair.body2 );
            }


            if ( ! checkResult )
            {
                isPrevResultCorrect = false;

                return false;
            }

            pairPos++;
        }

        m_manip.moveGroupCounter();
    }

    prevConfig = conf;
    isPrevResultCorrect = true;
    return true;
}




bool FreeCSpaceValidator::checkManipVsManip( uint16_t id1, uint16_t id2 ) const
{
    const auto & body1 = m_manip.getBody( id1 );
    const auto & body2 = m_manip.getBody( id2 );

    if ( ! m_collDetect.areObjectsCollided( body1, body2 ) )
    {
        return true;
    }

    bool isId1Link = id1 < env::ManipulatorController::biasGroupId;
    bool isId2Link = id2 < env::ManipulatorController::biasGroupId;

    if ( isId1Link && isId2Link )
    {
        return false;
    }

    if ( ( ! isId1Link ) && isId2Link )
    {
        return checkGroupVsLink( id1, id2 );
    }

    if ( isId1Link && ( ! isId2Link ) )
    {
        return checkGroupVsLink( id2, id1 );
    }

    return checkGroupVsGroup( id1, id2 );
}




bool FreeCSpaceValidator::checkManipVsObst( uint16_t manipId, uint16_t obstId ) const
{
    const auto & body1 = m_manip.getBody( manipId );
    const auto & body2 = m_obst.getBody( obstId );

    if ( ! m_collDetect.areObjectsCollided( body1, body2 ) )
    {
        return true;
    }

    bool isLink = manipId < env::ManipulatorController::biasGroupId;

    if ( isLink )
    {
        return false;
    }

    return checkGroupVsObst( manipId, obstId );
}



bool FreeCSpaceValidator::checkGroupVsLink( uint16_t groupId, uint16_t linkId ) const
{
    auto firstLinkGroup = m_manip.getFirstLinkGroup( groupId - m_manip.biasGroupId );
    auto linkNextGroup = m_manip.getLinkNextGroup( groupId - m_manip.biasGroupId );

    const auto & linkBody = m_manip.getBody( linkId ); 

    for ( auto link = firstLinkGroup; link != linkNextGroup; link++ )
    {
        if ( m_collDetect.areObjectsCollided( m_manip.getBody( link + m_manip.biasLinkId ), linkBody ) )
        {
            return false;
        }
    }

    return true;
}



bool FreeCSpaceValidator::checkGroupVsGroup( uint16_t groupId1, uint16_t groupId2 ) const
{
    auto firstLinkGroup1 = m_manip.getFirstLinkGroup( groupId1 - m_manip.biasGroupId );
    auto firstLinkGroup2 = m_manip.getFirstLinkGroup( groupId2 - m_manip.biasGroupId );

    auto linkNextGroup1 = m_manip.getLinkNextGroup( groupId1 - m_manip.biasGroupId );
    auto linkNextGroup2 = m_manip.getLinkNextGroup( groupId2 - m_manip.biasGroupId );

    for ( auto links1 = firstLinkGroup1; links1 != linkNextGroup1; links1++ )
    {
        const auto & body1 = m_manip.getBody( links1 + m_manip.biasLinkId );

        for ( auto links2 = firstLinkGroup2; links2 != linkNextGroup2; links2++ )
        {
            const auto & body2 = m_manip.getBody( links2 + m_manip.biasLinkId );

            if ( m_collDetect.areObjectsCollided( body1, body2 ) )
            {
                return false;
            }
        }
    }

    return true;
}



bool FreeCSpaceValidator::checkGroupVsObst( uint16_t groupId, uint16_t obstId ) const
{
    static const auto biasLinkId = m_manip.biasLinkId;

    auto firstLinkGroup = m_manip.getFirstLinkGroup( groupId - m_manip.biasGroupId );
    auto linkNextGroup = m_manip.getLinkNextGroup( groupId - m_manip.biasGroupId );

    const auto & obstBody = m_obst.getBody( obstId );

    for ( auto link = firstLinkGroup; link != linkNextGroup; link++ )
    {
        if ( m_collDetect.areObjectsCollided( 
            m_manip.getBody( link + m_manip.biasLinkId ), obstBody ) )
        {
            return false;
        }
    }

    return true;
}



uint16_t FreeCSpaceValidator::moveToNextGroup( uint16_t pairPos, uint16_t confPos ) const
{
    confPos++;

    if ( confPos == conf_space_dims )
    {
        return pairsAmount;
    }

    return m_listPosConfigChange[ confPos ];
}




void FreeCSpaceValidator::fillListPosConfigChange()
{
    auto posTable = 0;

    auto curConf = 0;

    m_listPosConfigChange[ 0 ] = 0;
    auto posList = 1;

    while ( curConf != conf_space_dims )
    {
        auto linkFirstId = m_manip.getFirstLinkGroup( curConf );
        auto linkNextGroupId = m_manip.getLinkNextGroup( curConf );

        auto groupCurId = curConf;
        auto groupNextId = curConf + 1;

        bool isConfigChanged = false;
        while ( ! isConfigChanged )
        {
            const auto & pair = m_collTable.getPair( posTable );

            bool isIdLink = pair.body1 < m_manip.biasGroupId;

            if ( isIdLink && ( pair.body1 < linkNextGroupId ) )
            {
                posTable++;

                continue;
            }

            if ( ( ! isIdLink ) && ( ( pair.body1 - m_manip.biasGroupId ) < groupNextId ) )
            {
                posTable++;

                continue;
            }

            isConfigChanged = true;
        }


        m_listPosConfigChange[ posList ] = posTable;
        posList++;

        curConf++;
    }


}




void FreeCSpaceValidator::setSegCheckPrec( float segCheckPrec_Degree )
{
    m_segDimStep_Rad = path::calcMaxDimValue_Rad( segCheckPrec_Degree );
}




float FreeCSpaceValidator::getSegCheckPrec_Degree() const
{
    return flt_op::cvtRadToDeg( 
        std::sqrtf( std::powf( m_segDimStep_Rad, 2.f ) * conf_space_dims ) );
}
