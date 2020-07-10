#include "info_loader.h"

using namespace motion_planner;


env::ManipInitInfo env::load::getManipInitInfo()
{

	ManipInitInfo::LinkInitInfo link0Info(
		Obb::Sizes( { 0.025000000372529, 0.029999999329448, 0.047504499554634 } ), /* sizes */
		Eigen::Vector3f( -0.03999999910593, 0.10000000149012, 0.17500001192093 ), /* initCenterPos */
		Eigen::Vector3f( -0.03999999910593, 0.10000000149012, 0.17500001192093 ), /* initMountPos */
		0, /* mountId */
		11, /* lastAlign */
		2, /* axisRot */
		0, /* groupId */
		{ -0, 0, -0 }, /* anglesOrient */
		{ false, false, false }, /* isAngleSet */
		false /* areAnglesUsed */
	);

	ManipInitInfo::LinkInitInfo link2Info(
		Obb::Sizes( { 0.021250000223517, 0.03999999910593, 0.17742499709129 } ), /* sizes */
		Eigen::Vector3f( -0.018749998882413, 0.099999971687794, 0.3374300301075 ), /* initCenterPos */
		Eigen::Vector3f( -0.018699999898672, 0.10000000149012, 0.19000001251698 ), /* initMountPos */
		0, /* mountId */
		11, /* lastAlign */
		0, /* axisRot */
		1, /* groupId */
		{ -0, 0, -0 }, /* anglesOrient */
		{ false, false, false }, /* isAngleSet */
		false /* areAnglesUsed */
	);

	ManipInitInfo::LinkInitInfo link3Info(
		Obb::Sizes( { 0.039250001311302, 0.03999999910593, 0.03999999910593 } ), /* sizes */
		Eigen::Vector3f( -0.00074999779462814, 0.099999971687794, 0.51485002040863 ), /* initCenterPos */
		Eigen::Vector3f( -0.00070000067353249, 0.10000000149012, 0.51480001211166 ), /* initMountPos */
		2, /* mountId */
		11, /* lastAlign */
		0, /* axisRot */
		1, /* groupId */
		{ -0, 0, -0 }, /* anglesOrient */
		{ false, false, false }, /* isAngleSet */
		false /* areAnglesUsed */
	);

	ManipInitInfo::LinkInitInfo link4Info(
		Obb::Sizes( { 0.019999999552965, 0.019999999552965, 0.15060000121593 } ), /* sizes */
		Eigen::Vector3f( 0.015500001609325, 0.10000000149012, 0.69173991680145 ), /* initCenterPos */
		Eigen::Vector3f( 0.015500001609325, 0.10000000149012, 0.53979992866516 ), /* initMountPos */
		2, /* mountId */
		11, /* lastAlign */
		0, /* axisRot */
		2, /* groupId */
		{ -0, 0, -0 }, /* anglesOrient */
		{ false, false, false }, /* isAngleSet */
		false /* areAnglesUsed */
	);

	ManipInitInfo::LinkInitInfo link5Info(
		Obb::Sizes( { 0.025800000876188, 0.049258500337601, 0.03397149965167 } ), /* sizes */
		Eigen::Vector3f( -0.029999999329448, 0.075699999928474, 0.85163998603821 ), /* initCenterPos */
		Eigen::Vector3f( -0.0044999979436398, 0.099999971687794, 0.8422999382019 ), /* initMountPos */
		4, /* mountId */
		11, /* lastAlign */
		0, /* axisRot */
		3, /* groupId */
		{ -0, 1.7453332930017e-06, 5.6843418860808e-14 }, /* anglesOrient */
		{ false, false, false }, /* isAngleSet */
		false /* areAnglesUsed */
	);

	ManipInitInfo::LinkInitInfo link7Info(
		Obb::Sizes( { 0.0062500000931323, 0.02250099927187, 0.0062500000931323 } ), /* sizes */
		Eigen::Vector3f( -0.013000000268221, 0.15590000152588, 0.82836997509003 ), /* initCenterPos */
		Eigen::Vector3f( -0.012999993748963, 0.13510000705719, 0.82839977741241 ), /* initMountPos */
		6, /* mountId */
		9, /* lastAlign */
		2, /* axisRot */
		5, /* groupId */
		{ 1.5987211554602e-14, 1.4901075040541e-08, -0.2617994248867 }, /* anglesOrient */
		{ false, false, true }, /* isAngleSet */
		true /* areAnglesUsed */
	);

	ManipInitInfo::LinkInitInfo link8Info(
		Obb::Sizes( { 0.0062500000931323, 0.022500002756715, 0.0062500000931323 } ), /* sizes */
		Eigen::Vector3f( -0.0031999973580241, 0.19547000527382, 0.82839941978455 ), /* initCenterPos */
		Eigen::Vector3f( -0.0029000013601035, 0.17499999701977, 0.82839965820313 ), /* initMountPos */
		7, /* mountId */
		9, /* lastAlign */
		2, /* axisRot */
		5, /* groupId */
		{ -2.2204460492503e-16, 5.4400928206633e-15, 1.6543618167517e-24 }, /* anglesOrient */
		{ false, false, false }, /* isAngleSet */
		false /* areAnglesUsed */
	);

	ManipInitInfo::LinkInitInfo link9Info(
		Obb::Sizes( { 0.0062501044012606, 0.02250050008297, 0.0062501044012606 } ), /* sizes */
		Eigen::Vector3f( -0.048372000455856, 0.15590000152588, 0.82836985588074 ), /* initCenterPos */
		Eigen::Vector3f( -0.048372000455856, 0.13510000705719, 0.8283998966217 ), /* initMountPos */
		6, /* mountId */
		11, /* lastAlign */
		2, /* axisRot */
		6, /* groupId */
		{ -3.460133655292e-14, 1.093395122706e-13, 0.2617994248867 }, /* anglesOrient */
		{ false, false, true }, /* isAngleSet */
		true /* areAnglesUsed */
	);

	ManipInitInfo::LinkInitInfo link10Info(
		Obb::Sizes( { 0.0062500000931323, 0.02250000089407, 0.0062500000931323 } ), /* sizes */
		Eigen::Vector3f( -0.05816999822855, 0.19547000527382, 0.82836997509003 ), /* initCenterPos */
		Eigen::Vector3f( -0.05816999822855, 0.17499999701977, 0.82840001583099 ), /* initMountPos */
		9, /* mountId */
		11, /* lastAlign */
		2, /* axisRot */
		6, /* groupId */
		{ -0, 0, -0 }, /* anglesOrient */
		{ false, false, false }, /* isAngleSet */
		false /* areAnglesUsed */
	);

	ManipInitInfo::LinkInitInfo link1Info(
		Obb::Sizes( { 0.035000000149012, 0.04500000551343, 0.075000002980232 } ), /* sizes */
		Eigen::Vector3f( -0.10000000149012, 0.10000000149012, 0.24999994039536 ), /* initCenterPos */
		Eigen::Vector3f( -0.10000000149012, 0.10000000149012, 0.25 ), /* initMountPos */
		0, /* mountId */
		11, /* lastAlign */
		2, /* axisRot */
		0, /* groupId */
		{ -0, 0, -0 }, /* anglesOrient */
		{ false, false, false }, /* isAngleSet */
		false /* areAnglesUsed */
	);

	ManipInitInfo::LinkInitInfo link6Info(
		Obb::Sizes( { 0.03125, 0.0099999997764826, 0.012500001117587 } ), /* sizes */
		Eigen::Vector3f( -0.030500000342727, 0.13510000705719, 0.82840001583099 ), /* initCenterPos */
		Eigen::Vector3f( -0.030500000342727, 0.13510000705719, 0.82840001583099 ), /* initMountPos */
		5, /* mountId */
		11, /* lastAlign */
		1, /* axisRot */
		4, /* groupId */
		{ -0, 0, -0 }, /* anglesOrient */
		{ false, false, false }, /* isAngleSet */
		false /* areAnglesUsed */
	);

	return ManipInitInfo{
		{ link0Info, link1Info, link2Info, link3Info, link4Info, link5Info, link6Info, link7Info, link8Info, link9Info, link10Info }
	};
}

env::ObstInitInfo env::load::getObstInitInfo()
{

	Obb::Sizes obst5Sizes( Obb::Sizes( { 0.125, 0.19249999523163, 0.050000000745058 } ) );
	Eigen::Vector3f obst5Pos( Eigen::Vector3f( -0.035000000149012, 0.0099999997764826, 0.07600000500679 ) );
	Eigen::Vector3f obst5Mount( Eigen::Vector3f( -0.035000000149012, 0.0099999997764826, 0.07600000500679 ) );
	Eigen::Matrix3f obst5Orient;
	obst5Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst4Sizes( Obb::Sizes( { 1, 1, 0.050000000745058 } ) );
	Eigen::Vector3f obst4Pos( Eigen::Vector3f( 0, 0, -0.026999995112419 ) );
	Eigen::Vector3f obst4Mount( Eigen::Vector3f( 0, 0, -0.026999995112419 ) );
	Eigen::Matrix3f obst4Orient;
	obst4Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst3Sizes( Obb::Sizes( { 0.025000000372529, 0.0625, 0.125 } ) );
	Eigen::Vector3f obst3Pos( Eigen::Vector3f( -0.18600000441074, 0.17499999701977, 0.15000000596046 ) );
	Eigen::Vector3f obst3Mount( Eigen::Vector3f( -0.15999999642372, 0.17499999701977, 0.08500000089407 ) );
	Eigen::Matrix3f obst3Orient;
	obst3Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst2Sizes( Obb::Sizes( { 0.025000000372529, 0.0625, 0.125 } ) );
	Eigen::Vector3f obst2Pos( Eigen::Vector3f( 0.11500000208616, 0.17499999701977, 0.15000000596046 ) );
	Eigen::Vector3f obst2Mount( Eigen::Vector3f( 0.090000003576279, 0.17499999701977, 0.08500000089407 ) );
	Eigen::Matrix3f obst2Orient;
	obst2Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst1Sizes( Obb::Sizes( { 0.025000000372529, 0.0625, 0.125 } ) );
	Eigen::Vector3f obst1Pos( Eigen::Vector3f( -0.18600000441074, -0.14499999582767, 0.15000000596046 ) );
	Eigen::Vector3f obst1Mount( Eigen::Vector3f( -0.15999999642372, -0.15999999642372, 0.08500000089407 ) );
	Eigen::Matrix3f obst1Orient;
	obst1Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst0Sizes( Obb::Sizes( { 0.025000000372529, 0.0625, 0.125 } ) );
	Eigen::Vector3f obst0Pos( Eigen::Vector3f( 0.11500000208616, -0.15999999642372, 0.15000000596046 ) );
	Eigen::Vector3f obst0Mount( Eigen::Vector3f( 0.090000003576279, -0.15999999642372, 0.08500000089407 ) );
	Eigen::Matrix3f obst0Orient;
	obst0Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	return ObstInitInfo( {
		AttachedDetail( Obb( obst0Sizes, obst0Pos, obst0Orient ), obst0Mount ),
		AttachedDetail( Obb( obst1Sizes, obst1Pos, obst1Orient ), obst1Mount ),
		AttachedDetail( Obb( obst2Sizes, obst2Pos, obst2Orient ), obst2Mount ),
		AttachedDetail( Obb( obst3Sizes, obst3Pos, obst3Orient ), obst3Mount ),
		AttachedDetail( Obb( obst4Sizes, obst4Pos, obst4Orient ), obst4Mount ),
		AttachedDetail( Obb( obst5Sizes, obst5Pos, obst5Orient ), obst5Mount )
	} );
}


env::Obb env::load::getOtherBody1_link1()
{	Obb::Sizes sizes( { 0.050000000745058, 0.050000000745058, 0.050000000745058 } );
	Eigen::Vector3f pos( 0.39999997615814, 0.1499999910593, 0.22499999403954 );
	Eigen::Matrix3f orient;
	orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	return Obb( sizes, pos, orient );
}



env::Obb env::load::getOtherBody2_link1()
{	Obb::Sizes sizes( { 0.050000000745058, 0.050000000745058, 0.050000000745058 } );
	Eigen::Vector3f pos( 0.30000001192093, 0.32500001788139, 0.30000004172325 );
	Eigen::Matrix3f orient;
	orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;

	return Obb( sizes, pos, orient );
}

