/**************************************************************************************************
Описание

Некоторые определения для модулей пространства имён env

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/


#pragma once

#include "Eigen/Eigen"
#include "Eigen/Geometry"

#include <algorithm>


namespace motion_planner
{
    namespace env
    {
        const uint16_t dim_space = 3;

        struct EulerAngles
        {
            float angles[ 3 ] = { 0.f, 0.f, 0.f };

            explicit EulerAngles()
            {}

            explicit EulerAngles( const float (&anglesList)[ 3 ] )
            {
                std::copy( anglesList, anglesList + 3, angles );
            }

            void sum( const EulerAngles & a )
            {
                std::transform( a.angles, a.angles + 3, angles, angles, 
                    []( const float & arg1, const float & arg2 )
                    {
                        return arg1 + arg2;
                    } );
            }

            void sub( const EulerAngles & a )
            {
                std::transform( a.angles, a.angles + 3, angles, angles, 
                    []( const float & arg1, const float & arg2 )
                    {
                        return arg2 - arg1;
                    } );
            }
        };
    }
}

