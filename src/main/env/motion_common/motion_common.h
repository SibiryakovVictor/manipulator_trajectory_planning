/**************************************************************************************************
Описание

Набор функций для вращения векторов и матриц, а также обертка функционала Eigen
для получения матрицы поворота вокруг заданной оси на заданной угол

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/


#pragma once

#include "main/env/env_def.h"


namespace motion_planner
{
    namespace env
    {
        const Eigen::Vector3f & getUnitAxis( uint8_t axis );


        Eigen::Matrix3f getRotMat( const float & angle_Rad, const Eigen::Vector3f & rotAxis );
        Eigen::Matrix3f getRotMat( const float & angle_Rad, Eigen::Vector3f && rotAxis );


        void rotateVector( Eigen::Vector3f & v, const float & angle_Rad, 
            const Eigen::Vector3f & rotAxis );
        Eigen::Vector3f rotateVector( const Eigen::Vector3f & v, const float & angle_Rad, 
            const Eigen::Vector3f & rotAxis );
        Eigen::Vector3f rotateVector( Eigen::Vector3f && v, const float & angle_Rad, 
            const Eigen::Vector3f & rotAxis );


        void rotateOrient( Eigen::Matrix3f & m, const float & angle_Rad,
            const Eigen::Vector3f & rotAxis );
        Eigen::Matrix3f rotateOrient( const Eigen::Matrix3f & m, 
            const float & angle_Rad, const Eigen::Vector3f & rotAxis );
        Eigen::Matrix3f rotateOrient( Eigen::Matrix3f && m, 
            const float & angle_Rad, const Eigen::Vector3f & rotAxis );
    }
}
