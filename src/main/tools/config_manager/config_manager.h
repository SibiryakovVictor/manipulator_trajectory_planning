#pragma once

#include "main/env/manipulator/manipulator.h"
#include "main/env/obstacles/obstacles.h"
#include "main/config_space/free_cspace_verifier/free_cspace_verifier.h"


namespace motion_planner
{
    class ConfigManager;
}


class motion_planner::ConfigManager
{

public:

    explicit ConfigManager( env::Manipulator & links,
                            env::Obstacles & obsts,
                            config_space::FreeCSpaceVerifier & verifier ) :
        m_links( links ),
        m_obsts( obsts ),
        m_verifier( verifier )
    {}

    void changeObjectBody( uint16_t objId, env::Obb body );

    void rotateObst( uint16_t obstId, float angle, Eigen::Vector3f axis );

    void activateCollPair( uint16_t pairIndex );

    void deactivateCollPair( uint16_t pairIndex );

private:

    env::Manipulator & m_links;

    env::Obstacles & m_obsts;

    config_space::FreeCSpaceVerifier & m_verifier;

};

