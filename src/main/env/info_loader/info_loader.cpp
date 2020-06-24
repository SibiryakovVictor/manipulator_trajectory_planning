#include "info_loader.h"

using namespace motion_planner;


env::ManipInitInfo env::load::getManipInitInfo()
{

	ManipInitInfo::LinkInitInfo link0Info(
		Obb::Sizes( { 0.025000000372529, 0.029999999329448, 0.047504499554634 } ),
		Position( -0.03999999910593, 0.10000000149012, 0.17500000596046 ),
		Position( -0.03999999910593, 0.10000000149012, 0.17500000596046 ),
		0,
		2,
		0,
		{ -0, 0, -0 },
		{ false, false, false },
		false
	);

	ManipInitInfo::LinkInitInfo link1Info(
		Obb::Sizes( { 0.035000000372529, 0.0450999999329448, 0.07504499554634 } ),
		Position( -0.100999910593, 0.10000000149012, 0.2500000596046 ),
		Position( -0.100999910593, 0.10000000149012, 0.2500000596046 ),
		0,
		2,
		0,
		{ -0, 0, -0 },
		{ false, false, false },
		false
	);


	ManipInitInfo::LinkInitInfo link2Info(
		Obb::Sizes( { 0.021250000223517, 0.03999999910593, 0.17742499709129 } ),
		Position( -0.018749998882413, 0.099999971687794, 0.33743002414703 ),
		Position( -0.018699999898672, 0.10000000149012, 0.19000000655651 ),
		0,
		0,
		1,
		{ -0, 0, -0 },
		{ false, false, false },
		false
	);


	ManipInitInfo::LinkInitInfo link3Info(
		Obb::Sizes( { 0.039250001311302, 0.03999999910593, 0.03999999910593 } ),
		Position( -0.00074999779462814, 0.099999971687794, 0.51494998464584 ),
		Position( -0.00070000067353249, 0.10000000149012, 0.5148000061512 ),
		2,
		0,
		1,
		{ -0, 0, -0 },
		{ false, false, false },
		false
	);


	ManipInitInfo::LinkInitInfo link4Info(
		Obb::Sizes( { 0.019999999552965, 0.019999999552965, 0.15060000121593 } ),
		Position( 0.015500001609325, 0.10000000149012, 0.69173994064331 ),
		Position( 0.015500001609325, 0.10000000149012, 0.53989995250702 ),
		2,
		0,
		2,
		{ -0, 0, -0 },
		{ false, false, false },
		false
	);


	ManipInitInfo::LinkInitInfo link5Info(
		Obb::Sizes( { 0.025800000876188, 0.049258501023054, 0.03397149965167 } ),
		Position( -0.029999999329448, 0.075799986994267, 0.85614000988007 ),
		Position( -0.0044999979436398, 0.099999971687794, 0.84329996204376 ),
		4,
		0,
		3,
		{ 1.7453292002756e-06, 1.7453292002756e-06, 1.7453292002756e-06 },
		{ false, false, false },
		false
	);


	ManipInitInfo::LinkInitInfo link6Info(
		Obb::Sizes( { 0.031250000876188, 0.010058501023054, 0.01250049965167 } ),
		Position( -0.0305000999329448, 0.135109986994267, 0.82840000988007 ),
		Position( -0.0305000999329448, 0.135109986994267, 0.82840000988007 ),
		5,
		1,
		4,
		{ 1.7453292002756e-06, 1.7453292002756e-06, 1.7453292002756e-06 },
		{ false, false, false },
		false
	);


	ManipInitInfo::LinkInitInfo link7Info(
		Obb::Sizes( { 0.0062500000931323, 0.02250099927187, 0.0062500000931323 } ),
		Position( -0.013000000268221, 0.1559999812603, 0.82846993932724 ),
		Position( -0.012999993748963, 0.13519997189045, 0.82849980125427 ),
		6,
		2,
		5,
		{ 1.5987211554602e-14, 1.4901075040541e-08, -0.2617994248867 },
		{ false, false, true },
		true
	);


	ManipInitInfo::LinkInitInfo link8Info(
		Obb::Sizes( { 0.0062500000931323, 0.022500002756715, 0.0062500000931323 } ),
		Position( -0.0031999973580241, 0.19557001481056, 0.8284994436264 ),
		Position( -0.0029000013601035, 0.17509999165535, 0.82849968204498 ),
		7,
		2,
		5,
		{ -2.2204460492503e-16, 5.4400928206633e-15, 1.6543618167517e-24 },
		{ false, false, false },
		false
	);


	ManipInitInfo::LinkInitInfo link9Info(
		Obb::Sizes( { 0.0062501044012606, 0.02250050008297, 0.0062501044012606 } ),
		Position( -0.048372000455856, 0.15599999616146, 0.8284698797226 ),
		Position( -0.048372000455856, 0.13510001659393, 0.82849986085892 ),
		6,
		2,
		6,
		{ -3.460133655292e-14, 1.093395122706e-13, 0.2617994248867 },
		{ false, false, true },
		true
	);


	ManipInitInfo::LinkInitInfo link10Info(
		Obb::Sizes( { 0.0062500000931323, 0.02250000089407, 0.0062500000931323 } ),
		Position( -0.05816999822855, 0.1954699999094, 0.82846993932724 ),
		Position( -0.05816999822855, 0.17500000655651, 0.82849998006821 ),
		9,
		2,
		6,
		{ -0, 0, -0 },
		{ false, false, false },
		false
	);


	return ManipInitInfo{
		{ link0Info, link1Info, link2Info, link3Info, link4Info,
		  link5Info, link6Info, link7Info, link8Info, link9Info, link10Info }
	};
}




env::ObstInitInfo env::load::getObstInitInfo()
{

	Obb::Sizes obst0Sizes( Obb::Sizes( { 0.075000002980232, 0.075000002980232, 0.037500001490116 } ) );
	Position obst0Pos( Position( -0.035000000149012, -0.049999997019768, 0.16500000059605 ) );
	Position obst0Mount( Position( -0.035000000149012, -0.049999997019768, 0.16500000059605 ) );
	Orient obst0Orient;
	obst0Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst1Sizes( Obb::Sizes( { 0.125, 0.1925999403954, 0.050000000745058 } ) );
	Position obst1Pos( Position( -0.035000000149012, 0.0099999997764826, 0.076099999046326 ) );
	Position obst1Mount( Position( -0.035000000149012, 0.0099999997764826, 0.076099999046326 ) );
	Orient obst1Orient;
	obst1Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst2Sizes( Obb::Sizes( { 1, 1, 0.050000000745058 } ) );
	Position obst2Pos( Position( 0, 0, -0.027000001072884 ) );
	Position obst2Mount( Position( 0, 0, -0.027000001072884 ) );
	Orient obst2Orient;
	obst2Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst3Sizes( Obb::Sizes( { 0.025000000372529, 0.0625, 0.125 } ) );
	Position obst3Pos( Position( -0.18600000441074, 0.17499999701977, 0.1500000607967 ) );
	Position obst3Mount( Position( -0.1600000441074, 0.17499999701977, 0.0850000607967 ) );
	Orient obst3Orient;
	obst3Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst4Sizes( Obb::Sizes( { 0.025000000372529, 0.0625, 0.125 } ) );
	Position obst4Pos( Position( 0.11500000208616, 0.17499999701977, 0.15000000607967 ) );
	Position obst4Mount( Position( 0.0900000208616, 0.17499999701977, 0.08500000607967 ) );
	Orient obst4Orient;
	obst4Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst5Sizes( Obb::Sizes( { 0.025000000372529, 0.0625, 0.125 } ) );
	Position obst5Pos( Position( -0.18600000441074, -0.14499999582767, 0.15000000607967 ) );
	Position obst5Mount( Position( -0.1600000441074, -0.16009999582767, 0.085000000607967 ) );
	Orient obst5Orient;
	obst5Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	Obb::Sizes obst6Sizes( Obb::Sizes( { 0.025000000372529, 0.0625, 0.125 } ) );
	Position obst6Pos( Position( 0.11500000208616, -0.15999999642372, 0.15000000607967 ) );
	Position obst6Mount( Position( 0.09000000208616, -0.15999999642372, 0.08500000607967 ) );
	Orient obst6Orient;
	obst6Orient <<
		1, 0, 0,
		0, 1, 0,
		0, 0, 1;


	return ObstInitInfo( {
		AttachedDetail( Obb( obst6Sizes, obst6Pos, obst6Orient ), obst6Mount ),
		AttachedDetail( Obb( obst5Sizes, obst5Pos, obst5Orient ), obst5Mount ),
		AttachedDetail( Obb( obst4Sizes, obst4Pos, obst4Orient ), obst4Mount ),
		AttachedDetail( Obb( obst3Sizes, obst3Pos, obst3Orient ), obst3Mount ),
		AttachedDetail( Obb( obst2Sizes, obst2Pos, obst2Orient ), obst2Mount ),
		AttachedDetail( Obb( obst1Sizes, obst1Pos, obst1Orient ), obst1Mount ),
		AttachedDetail( Obb( obst0Sizes, obst0Pos, obst0Orient ), obst0Mount )
		} );

}
