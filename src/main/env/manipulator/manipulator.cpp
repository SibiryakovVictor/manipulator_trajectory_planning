#include "manipulator.h"
#include "main/env/motion_common/motion_common.h"

#include <cfloat>

using namespace motion_planner;
using namespace motion_planner::env;
using namespace motion_planner::manip;




const Obb & Manipulator::getBodyCurLink() const
{
	if ( m_linkType[ m_linkId ] > 1 )
	{
		return m_groups[ m_groupId ];
	}

	return m_links[ m_linkId ];
}



uint16_t Manipulator::setConfigCurrentLink( const float & config )
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
		m_groups[ m_groupId ].setCenterPos( calcComplexPos( m_linkId ) );
	}

	return m_linkId;
}




void Manipulator::setConfig( uint16_t linkId, const float & config )
{

	for ( uint16_t linkCount = linkId; linkCount != linkId + m_linkType[ linkId ];
		linkCount++ )
	{
		appendInitPos( linkCount );	
	}


	rotateCenterLink( linkId, config );


	Orient && tempOrient = rotateOrient( m_links[ linkId ].getOrient(), config,
		getUnitAxis( m_initLinksInfo[ linkId ].axisRot_ ) );

	manip::OrientDependency orDep = m_linkOrientDep[ m_orientCounter ];
	while ( orDep.src == linkId )
	{
		m_links[ orDep.dst ].setOrient( tempOrient );

		m_orientCounter++;

		orDep = m_linkOrientDep[ m_orientCounter ];
	}


	if ( m_linkType[ m_linkId ] > 1 )
	{
		m_groups[ m_groupId ].setOrient( tempOrient );
	}


	if ( m_initLinksInfo[ linkId ].areAnglesUsed_ )
	{
		m_links[ linkId ].setOrient( rotateOrient( 
			appendInitOrient( linkId, m_links[ linkId ].getOrient() ),
			config,
			getUnitAxis( m_initLinksInfo[ linkId ].axisRot_ ) ) );
	}
	else
	{
		m_links[ linkId ].setOrient( tempOrient );
	}

}




void Manipulator::alignMountCenterPos( float angle, uint16_t refLinkId, 
	uint16_t startAlignPos, uint16_t endAlignPos )
{

	Eigen::Matrix3f && rotMat = getRotMat( -angle,
		m_links[ refLinkId ].getOrient().row( 
			m_initLinksInfo[ refLinkId ].axisRot_ ) );


	for ( uint8_t alignLink = startAlignPos; alignLink != endAlignPos; alignLink++ )
	{
		m_mountPos[ alignLink ] = m_mountPos[ refLinkId ] +
			( rotMat * ( m_mountPos[ alignLink ] - m_mountPos[ refLinkId ] ) );

		m_links[ alignLink ].setCenterPos( m_mountPos[ refLinkId ] +
			( rotMat * ( m_links[ alignLink ].getCenterPos() - m_mountPos[ refLinkId ] ) ) );
	}

}



void Manipulator::initMountCenterPos( float angle )
{

	Eigen::Matrix3f && rotMat = getRotMat( -angle,
		m_links[ 0 ].getOrient().row( m_initLinksInfo[ 0 ].axisRot_ ) );


	for ( uint8_t alignLink = 1; alignLink != manip::links_amount; alignLink++ )
	{
		m_mountPos[ alignLink ] = m_mountPos[ 0 ] +
			( rotMat * ( m_initLinksInfo[ alignLink ].mountPos_ - m_mountPos[ 0 ] ) );

		m_links[ alignLink ].setCenterPos( m_mountPos[ 0 ] +
			( rotMat * ( m_initLinksInfo[ alignLink ].body_.getCenterPos() - m_mountPos[ 0 ] ) ) );
	}

}



void Manipulator::rotateCenterLink( uint16_t linkId, const float & config )
{

	Eigen::Vector3f && axisRot = m_links[ linkId ].getOrient().row( 
		m_initLinksInfo[ linkId ].axisRot_ );

	m_links[ linkId ].setCenterPos( m_mountPos[ linkId ] + rotateVector( 
		m_links[ linkId ].getCenterPos() - m_mountPos[ linkId ], -config, axisRot ) );

}




void Manipulator::moveGroupCounter()
{
	if ( m_linkType[ m_linkId ] > 1 )
	{
		m_groupId++;
	}

	m_linkId += m_linkType[ m_linkId ];
}



const Obb & Manipulator::getBody( uint16_t id ) const
{
	if ( id < biasGroupId )
	{
		return m_links[ id - biasLinkId ];
	}

	return m_groups[ id - biasGroupId ];
}



const Obb & Manipulator::getBodyGroup( uint16_t groupId ) const
{
	if ( m_groupsPos[ groupId ] != UINT16_MAX )
	{
		return m_groups[ m_groupsPos[ groupId ] ];
	}

	return m_links[ m_firstLinkGroup[ groupId ] ];
}



uint16_t Manipulator::getLinkType( uint16_t linkId ) const
{
	return m_linkType[ linkId ];
}



uint16_t Manipulator::getLinkNextGroup( uint16_t groupId ) const
{
	return m_firstLinkGroup[ groupId ] + m_linkType[ m_firstLinkGroup[ groupId ] ];
}



uint16_t Manipulator::getFirstLinkGroup( uint16_t groupId ) const
{
	return m_firstLinkGroup[ groupId ];
}




uint16_t Manipulator::getFirstLinkComplex( uint16_t complex ) const
{
	return m_complexFirstLink[ complex - biasGroupId ];
}




uint16_t Manipulator::getCurGroupId() const
{
	return m_linkInGroup[ m_linkId ];
}



uint16_t Manipulator::getGroupId( uint16_t linkId ) const
{
	return m_linkInGroup[ linkId ];
}



void Manipulator::appendInitPos( uint16_t linkId )
{

	if ( m_initLinksInfo[ linkId ].areAnglesUsed_ )
	{

		for ( int8_t curDim = 2; curDim != -1; curDim-- )
		{
			if ( ! m_initLinksInfo[ linkId ].isAngleSet_[ curDim ] )
			{
				continue;
			}

			m_links[ linkId ].setCenterPos( m_mountPos[ linkId ] + rotateVector(
				m_links[ linkId ].getCenterPos() - m_mountPos[ linkId ], 
				m_initLinksInfo[ linkId ].anglesOrient_.angles[ curDim ],
				m_links[ linkId ].getOrient().row( curDim )
			) );
		}

	}

}



Orient Manipulator::appendInitOrient( uint16_t linkId, 
	const env::Orient & inputOrient )
{
	auto result( inputOrient );

	for ( int8_t curDim = 2; curDim != -1; curDim-- )
	{
		if ( ! m_initLinksInfo[ linkId ].isAngleSet_[ curDim ] )
		{
			continue;
		}

		rotateOrient( result, -m_initLinksInfo[ linkId ].anglesOrient_.angles[ curDim ],
			getUnitAxis( curDim ) );
	}

	return result;
}



void Manipulator::reset()
{
	m_links[ 0 ].setCenterPos( m_initLinksInfo[ 0 ].body_.getCenterPos() );

	m_links[ 0 ].setOrient( Eigen::Matrix3f::Identity() );

	m_mountPos[ 0 ] = m_initLinksInfo[ 0 ].mountPos_;

	m_linkId = 0;

	m_groupId = 0;

	m_orientCounter = 0;
}



Obb::Sizes Manipulator::calcComplexSize( uint16_t linkId ) const
{
	float minCoords[ 3 ] = { FLT_MAX, FLT_MAX, FLT_MAX };
	float maxCoords[ 3 ] = { FLT_MIN, FLT_MIN, FLT_MIN };


	for ( uint16_t curBody = linkId; 
		curBody != linkId + m_linkType[ linkId ]; curBody++ )
	{

		const env::Obb::Sizes & sizes = m_links[ curBody ].getSizes();
		const env::Position & initCenterPos = m_initLinksInfo[ curBody ].body_.getCenterPos();


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




Position Manipulator::calcComplexPos( uint16_t linkId ) const
{

	Position resultPos( 0.f, 0.f, 0.f );

	for ( uint16_t curBody = linkId; 
		curBody != linkId + m_linkType[ linkId ]; curBody++ )
	{

		resultPos += m_links[ curBody ].getCenterPos();

	}

	resultPos /= m_linkType[ linkId ];

	return resultPos;

}



void Manipulator::defineLinksType()
{

	for ( uint16_t groupId = 0, pos = 0; 
		( groupId < manip::links_amount ) && ( pos < manip::links_amount ); 
		groupId++, pos++ )
	{
		uint16_t resultGroup = std::count_if( m_initLinksInfo, 
			m_initLinksInfo + manip::links_amount, 
			[ groupId ]( env::ManipInitInfo::LinkInitInfo & linkInfo )
			{
				return linkInfo.groupId_ == groupId;
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



void Manipulator::defineOrientDependencies()
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
						return ( linkInfo.mountId_ == groupId );
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




void Manipulator::defineFirstLinkGroups()
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
					return ( linkInfo.groupId_ == groupId );
				} ) );

		if ( findResult == manip::links_amount )
		{
			continue;
		}

		m_firstLinkGroup[ pos ] = findResult;
		pos++;

	}
}




void Manipulator::defineGroupsPos()
{

	std::fill( m_groupsPos, m_groupsPos + manip::links_groups - 1, UINT16_MAX );

	uint16_t link = 0;
	uint16_t groupPos = 0;

	std::for_each( m_firstLinkGroup, m_firstLinkGroup + manip::links_groups,
		[ this, &link, &groupPos ]( uint16_t firstLink )
		{
			if ( m_linkType[ firstLink ] > 1 )
			{
				m_groupsPos[ link ] = groupPos;
				groupPos++;
			}

			link++;
		} );

}



void Manipulator::defineLinksBelongGroups()
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



void Manipulator::defineFirstLinkComplex()
{
	uint16_t linkPos = 0;

	uint16_t complexList = 0;

	std::for_each( m_linkType, m_linkType + manip::links_amount,
		[ this, &linkPos, &complexList ]( uint16_t linkType )
		{
			if ( linkType > 1 )
			{
				m_complexFirstLink[ complexList ] = linkPos;

				complexList++;
			}

			linkPos++;
		} );
}



Manipulator::Manipulator( env::ManipInitInfo && manipLinksInfo )
{

	std::copy( manipLinksInfo.linksData, 
		manipLinksInfo.linksData + manip::links_amount, m_initLinksInfo );

	for ( uint16_t link = 0; link != manip::links_amount; link++ )
	{
		m_links[ link ].setSizes( m_initLinksInfo[ link ].body_.getSizes() );
	}

	reset();

	defineLinksType();

	defineFirstLinkComplex();

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

		m_groups[ complexCounter ].setSizes( calcComplexSize( curLink ) );
		complexCounter++;
	}

}




void Manipulator::changeBody( uint16_t linkId, const env::Obb & body )
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

