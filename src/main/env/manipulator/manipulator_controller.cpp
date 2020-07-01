/**************************************************************************************************
Описание

Класс, основное назначение которого - размещение в пространстве ограничивающих объемов звеньев
в соответствии с заданной конфигурацией манипулятора

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/



#include "manipulator_controller.h"
#include "main/env/motion_common/motion_common.h"

#include <cfloat>

using namespace motion_planner;
using namespace motion_planner::env;
using namespace motion_planner::manip;




/**************************************************************************************************
Описание:
устанавливает конфигурацию текущего звена/группы звеньев (или угол поворота, в случае вращательной пары).
Помимо этого делается следующее:
* осуществляется аналогичный поворот для всех звеньев, у которых ориентация в пространстве зависит
от звена, поворот которого требуется установить.
* если конфигурация устанавливается для группы звеньев, то изменяется также позиция и ориентация
ограничивающего объема всей группы звеньев.
Аргументы:
* config: конфигурация текущего звена/группы звеньев
Возврат:
**************************************************************************************************/
void ManipulatorController::setConfigCurrentLink( const float & config )
{
	setConfig( m_linkId, config );

	if ( m_linkId )
	{
		alignMountCenterPos( config,
			m_linkId, m_linkId + 1, manip::links_align_last[ m_linkId ] );
	}
	else
	{
		initMountCenterPos( config );
	}


	if ( m_linkType[ m_linkId ] > 1 )
	{
		m_complex[ m_complexId ].setCenterPos( calcComplexPos( m_linkId ) );
	}
}




void ManipulatorController::setConfig( uint16_t linkId, const float & config )
{
	for ( uint16_t linkCount = linkId; linkCount != linkId + m_linkType[ linkId ];
		linkCount++ )
	{
		appendInitPos( linkCount );	
	}


	rotateCenterLink( linkId, config );


	Eigen::Matrix3f && tempOrient = rotateOrient( m_links[ linkId ].getOrient(), config,
		getUnitAxis( m_initLinksInfo[ linkId ].axisRot ) );

	manip::OrientDependency orDep = m_linkOrientDep[ m_orientCounter ];
	while ( orDep.src == linkId )
	{
		m_links[ orDep.dst ].setOrient( tempOrient );
		
		m_orientCounter++;

		orDep = m_linkOrientDep[ m_orientCounter ];
	}


	if ( m_initLinksInfo[ linkId ].areAnglesUsed )
	{
		m_links[ linkId ].setOrient( rotateOrient( 
			appendInitOrient( linkId, m_links[ linkId ].getOrient() ),
			config,
			getUnitAxis( m_initLinksInfo[ linkId ].axisRot ) ) );
	}
	else
	{
		m_links[ linkId ].setOrient( tempOrient );
	}


	if ( m_linkType[ m_linkId ] > 1 )
	{
		m_complex[ m_complexId ].setOrient( m_links[ m_linkId ].getOrient() );
	}
}




void ManipulatorController::alignMountCenterPos( float angle, uint16_t refLinkId, 
	uint16_t startAlignPos, uint16_t endAlignPos )
{

	Eigen::Matrix3f && rotMat = getRotMat( -angle,
		m_links[ refLinkId ].getOrient().row( 
			m_initLinksInfo[ refLinkId ].axisRot ) );


	for ( uint8_t alignLink = startAlignPos; alignLink != endAlignPos; alignLink++ )
	{
		m_mountPos[ alignLink ] = m_mountPos[ refLinkId ] +
			( rotMat * ( m_mountPos[ alignLink ] - m_mountPos[ refLinkId ] ) );

		m_links[ alignLink ].setCenterPos( m_mountPos[ refLinkId ] +
			( rotMat * ( m_links[ alignLink ].getCenterPos() - m_mountPos[ refLinkId ] ) ) );
	}

}



void ManipulatorController::initMountCenterPos( float angle )
{
	Eigen::Matrix3f && rotMat = getRotMat( -angle,
		m_links[ 0 ].getOrient().row( m_initLinksInfo[ 0 ].axisRot ) );

	for ( uint8_t alignLink = 1; alignLink != manip::links_amount; alignLink++ )
	{
		m_mountPos[ alignLink ] = m_mountPos[ 0 ] +
			( rotMat * ( m_initLinksInfo[ alignLink ].initMountPos - m_mountPos[ 0 ] ) );

		m_links[ alignLink ].setCenterPos( m_mountPos[ 0 ] +
			( rotMat * ( m_initLinksInfo[ alignLink ].body.getCenterPos() - m_mountPos[ 0 ] ) ) );
	}
}



void ManipulatorController::rotateCenterLink( uint16_t linkId, const float & config )
{
	Eigen::Vector3f && axisRot = m_links[ linkId ].getOrient().row( 
		m_initLinksInfo[ linkId ].axisRot );

	m_links[ linkId ].setCenterPos( m_mountPos[ linkId ] + rotateVector( 
		m_links[ linkId ].getCenterPos() - m_mountPos[ linkId ], -config, axisRot ) );
}




/**************************************************************************************************
Описание:
смещает счётчики тел звеньев и групп звеньев на следующую группу, чтобы при вызове setConfigCurrentLink
устанавливалась конфигурация следующего звена/группы звеньев
Аргументы: 
Возврат:
**************************************************************************************************/
void ManipulatorController::moveGroupCounter()
{
	if ( m_linkType[ m_linkId ] > 1 )
	{
		m_complexId++;
	}

	m_linkId += m_linkType[ m_linkId ];
}



/**************************************************************************************************
Описание:
Возвращает ограничивающий объем требуемого звена или группы звеньев
Аргументы: 
* id: идентификатор требуемого ограничивающего объема, который должен быть задан 
с использованием констант biasLinkId (для звена) и biasGroupId (для группы звеньев)
Возврат: ограничивающий объем в зависимости от идентификатора id
**************************************************************************************************/
const Obb & ManipulatorController::getBody( uint16_t id ) const
{
	if ( id < biasGroupId )
	{
		return m_links[ id - biasLinkId ];
	}

	const auto complexPos = m_groupsPosInComplex[ id - biasGroupId ];

	if ( complexPos == UINT16_MAX )
	{
		while ( true )
		{
			; //запрошенная группа состоит из одного звена и не имеет комплексного тела
		}
	}

	return m_complex[ complexPos ];
}




/**************************************************************************************************
Описание:
Возвращает тип звена ( 1 - единственное в группе, 0 - входит в группу, не являясь её первым звеном,
2 и больше - первое в группе звеньев звено )
Аргументы: 
* linkId: порядковый номер звена
Возврат: ограничивающий объем в зависимости от идентификатора id
**************************************************************************************************/
uint16_t ManipulatorController::getLinkType( uint16_t linkId ) const
{
	return m_linkType[ linkId ];
}



/**************************************************************************************************
Описание:
Возвращает порядковый номер первого звена следующей группы за groupId 
Аргументы: 
* groupId: идентификатор группы
Возврат: порядковый номер звена следующей группы
**************************************************************************************************/
uint16_t ManipulatorController::getLinkNextGroup( uint16_t groupId ) const
{
	return m_firstLinkGroup[ groupId ] + m_linkType[ m_firstLinkGroup[ groupId ] ];
}




/**************************************************************************************************
Описание:
Возвращает порядковый номер первого звена группы groupId 
Аргументы: 
* groupId: идентификатор группы
Возврат: порядковый первого номер звена в группе
**************************************************************************************************/
uint16_t ManipulatorController::getFirstLinkGroup( uint16_t groupId ) const
{
	return m_firstLinkGroup[ groupId ];
}




/**************************************************************************************************
Описание:
возвращает id текущей группы (которой будет задана конфигурация при вызове setConfigCurrentLink)
Аргументы:
Возврат: id текущей группы
**************************************************************************************************/
uint16_t ManipulatorController::getCurGroupId() const
{
	return getGroupId( m_linkId );
}




/**************************************************************************************************
Описание:
возвращает идентификатор группы, к которой принадлежит звено linkId
Аргументы: 
* linkId: порядковый номер звена
Возврат: идентификатор группы требуемого звена
**************************************************************************************************/
uint16_t ManipulatorController::getGroupId( uint16_t linkId ) const
{
	return m_linkInGroup[ linkId ];
}




void ManipulatorController::appendInitPos( uint16_t linkId )
{

	if ( m_initLinksInfo[ linkId ].areAnglesUsed )
	{

		for ( int8_t curDim = 2; curDim != -1; curDim-- )
		{
			if ( ! m_initLinksInfo[ linkId ].isAngleSet[ curDim ] )
			{
				continue;
			}

			m_links[ linkId ].setCenterPos( m_mountPos[ linkId ] + rotateVector(
				m_links[ linkId ].getCenterPos() - m_mountPos[ linkId ], 
				m_initLinksInfo[ linkId ].anglesOrient.angles[ curDim ],
				m_links[ linkId ].getOrient().row( curDim )
			) );
		}

	}

}



Eigen::Matrix3f ManipulatorController::appendInitOrient( uint16_t linkId, 
	const Eigen::Matrix3f & inputOrient )
{
	auto result( inputOrient );

	for ( int8_t curDim = 2; curDim != -1; curDim-- )
	{
		if ( ! m_initLinksInfo[ linkId ].isAngleSet[ curDim ] )
		{
			continue;
		}

		rotateOrient( result, -m_initLinksInfo[ linkId ].anglesOrient.angles[ curDim ],
			getUnitAxis( curDim ) );
	}

	return result;
}


/**************************************************************************************************
Описание:
сбрасывает все счетчики и устанавливает позицию первого звена равной начальной;
используется перед заданием новой конфигурации манипулятора
Аргументы: 
Возврат:
**************************************************************************************************/
void ManipulatorController::reset()
{
	m_links[ 0 ].setCenterPos( m_initLinksInfo[ 0 ].body.getCenterPos() );

	m_links[ 0 ].setOrient( Eigen::Matrix3f::Identity() );

	m_mountPos[ 0 ] = m_initLinksInfo[ 0 ].initMountPos;

	m_linkId = 0;

	m_complexId = 0;

	m_orientCounter = 0;
}



Obb::Sizes ManipulatorController::calcComplexSize( uint16_t linkId ) const
{
	float minCoords[ 3 ] = { FLT_MAX, FLT_MAX, FLT_MAX };
	float maxCoords[ 3 ] = { FLT_MIN, FLT_MIN, FLT_MIN };


	for ( uint16_t curBody = linkId; 
		curBody != linkId + m_linkType[ linkId ]; curBody++ )
	{

		const env::Obb::Sizes & sizes = m_links[ curBody ].getSizes();
		const Eigen::Vector3f & initCenterPos = m_initLinksInfo[ curBody ].body.getCenterPos();


		for ( uint8_t curDim = 0; curDim != dim_space; curDim++ )
		{

			auto curPosBias = ( initCenterPos( curDim ) + sizes.dims[ curDim ] );
			auto curNegBias = ( initCenterPos( curDim ) - sizes.dims[ curDim ] );

			if ( curPosBias > maxCoords[ curDim ] )
			{
				maxCoords[ curDim ] = curPosBias;
			}

			if ( curNegBias < minCoords[ curDim ] )
			{
				minCoords[ curDim ] = curNegBias;
			}
		}
	}


	return env::Obb::Sizes( { 
		std::fabsf( maxCoords[ 0 ] - minCoords[ 0 ] ),
		std::fabsf( maxCoords[ 1 ] - minCoords[ 1 ] ),
		std::fabsf( maxCoords[ 2 ] - minCoords[ 2 ] )
		} );
}




Eigen::Vector3f ManipulatorController::calcComplexPos( uint16_t linkId ) const
{

	Eigen::Vector3f resultPos( 0.f, 0.f, 0.f );

	for ( uint16_t curBody = linkId; 
		curBody != linkId + m_linkType[ linkId ]; curBody++ )
	{

		resultPos += m_links[ curBody ].getCenterPos();

	}

	resultPos /= m_linkType[ linkId ];

	return resultPos;

}




ManipulatorController::ManipulatorController( env::ManipInitInfo && manipLinksInfo )
{

	std::copy( manipLinksInfo.linksData, 
		manipLinksInfo.linksData + manip::links_amount, m_initLinksInfo );

	for ( uint16_t link = 0; link != manip::links_amount; link++ )
	{
		m_links[ link ].setSizes( m_initLinksInfo[ link ].body.getSizes() );
	}

	reset();

	defineLinksType();

	defineLinksBelongGroups();

	defineOrientDependencies();

	defineFirstLinkGroups();

	defineGroupsPos();

	for ( uint16_t curLink = 0, complexCounter = 0; 
		( curLink != manip::links_amount ) && 
		( complexCounter != manip::links_complex_amount ); curLink++ )
	{
		if ( m_linkType[ curLink ] < 2 )
		{
			continue;
		}

		m_complex[ complexCounter ].setSizes( calcComplexSize( curLink ) );
		complexCounter++;
	}

}




void ManipulatorController::changeBody( uint16_t linkId, const env::Obb & body )
{
	m_links[ linkId ] = body;

	if ( m_linkType[ linkId ] == 1 )
	{
		return;
	}

	uint16_t firstLinkGroup = m_firstLinkGroup[ m_linkInGroup[ linkId ] ];

	calcComplexPos( firstLinkGroup );

	calcComplexSize( firstLinkGroup );
}





void ManipulatorController::defineLinksType()
{

	for ( uint16_t groupId = 0, pos = 0; 
		( groupId < manip::links_amount ) && ( pos < manip::links_amount ); 
		groupId++, pos++ )
	{
		uint16_t resultGroup = std::count_if( m_initLinksInfo, 
			m_initLinksInfo + manip::links_amount, 
			[ groupId ]( env::ManipInitInfo::LinkInitInfo & linkInfo )
			{
				return linkInfo.groupId == groupId;
			} );

		if ( ! resultGroup )
		{
			continue;
		}

		if ( resultGroup == 1 )
		{
			m_linkType[ pos ] = 1;
		}
		else
		{
			m_linkType[ pos ] = resultGroup;

			uint16_t nextPos = groupId;

			while ( resultGroup != 1 )
			{
				pos++;

				m_linkType[ pos ] = 0;

				resultGroup--;
			}
		}

	}

}



void ManipulatorController::defineOrientDependencies()
{

	for ( uint16_t groupId = 0, depPos = 0;
		( groupId < manip::links_amount ) && ( depPos < manip::links_amount );
		groupId++ )
	{
		uint16_t searchPos = groupId + 1;

		while ( searchPos < manip::links_amount )
		{
			searchPos = std::distance( m_initLinksInfo,
				std::find_if( m_initLinksInfo + searchPos,  
					m_initLinksInfo + manip::links_amount,
					[ &groupId ]( env::ManipInitInfo::LinkInitInfo & linkInfo )
					{
						return ( linkInfo.mountId == groupId );
					} ) );

			if ( searchPos == manip::links_amount )
			{
				break;
			}

			m_linkOrientDep[ depPos ] = OrientDependency( groupId, searchPos );

			searchPos++;
			depPos++;
		}

	}

}




void ManipulatorController::defineFirstLinkGroups()
{

	for ( uint16_t groupId = 0, pos = 0;
		(groupId < manip::links_amount) && (pos < manip::links_groups );
		groupId++)
	{

		uint16_t findResult = std::distance( m_initLinksInfo,
			std::find_if( m_initLinksInfo + groupId,  
				m_initLinksInfo + manip::links_amount,
				[ &groupId ]( env::ManipInitInfo::LinkInitInfo & linkInfo )
				{
					return ( linkInfo.groupId == groupId );
				} ) );

		if ( findResult == manip::links_amount )
		{
			continue;
		}

		m_firstLinkGroup[ pos ] = findResult;
		pos++;

	}
}




void ManipulatorController::defineGroupsPos()
{
	std::fill( m_groupsPosInComplex, m_groupsPosInComplex + manip::links_groups, UINT16_MAX );

	uint16_t link = 0;
	uint16_t groupPos = 0;

	std::for_each( m_firstLinkGroup, m_firstLinkGroup + manip::links_groups,
		[ this, &link, &groupPos ]( uint16_t firstLink )
		{
			if ( m_linkType[ firstLink ] > 1 )
			{
				m_groupsPosInComplex[ link ] = groupPos;
				groupPos++;
			}

			link++;
		} );
}



void ManipulatorController::defineLinksBelongGroups()
{
	uint16_t groupId = 0;
	uint16_t link = 1;


	std::for_each( m_linkType + 1, m_linkType + manip::links_amount,
		[ this, &link, &groupId ]( uint16_t linkType )
		{
			if ( linkType != 0 )
			{
				groupId++;
			}

			m_linkInGroup[ link ] = groupId;

			link++;
		} );
}

