/**************************************************************************************************
Описание

Определение типов данных для занесения информации о манипуляторе и препятствиях.
Используются в функциях getManipInitInfo и getObstInitInfo, 
определенных в "env/info_loader.cpp"

Разработчик: Сибиряков Виктор
Заметки
* !!! ОЧЕНЬ ВАЖНО !!!
-- размер массива linksData должен быть равен числу структур LinkInitInfo 
в функции getManipInitInfo (env/info_loader/info_loader.cpp)
(для этого настраивать константу links_amount в env/manip_parameters.h)
-- размер массива obsts должен быть равен числу структур AttachedDetail, которые
создаются в функции getObstInitInfo в return (env/info_loader/info_loader.cpp)
(для этого настраивать константу obst_amount в env/obst_parameters.h)

* Структура LinkInitInfo позволяет добавлять звенья, состоящие более чем из одного
ограничивающего объёма, используя для этого числовой идентификатор groupId. 
У структур LinkInitInfo, которые должны рассматриваться как группа, то есть часть 
одного составного звена, значения этого идентификатора равны, в таком случае для всех 
этих частей будет задаваться одна общая конфигурация. Также структура содержит идентификатор 
оси вращения звена axisRot (идентификаторы 0, 1, 2 обозначают оси X, Y, Z соответственно), 
тело звена body, содержащее начальную центральную позицию и габариты ограничивающего параллелепипеда, 
начальную позицию точки крепления initMountPos к звену с идентификатором mountId, 
относительно которого происходит поворот звена, начальную ориентацию в пространстве 
в виде углов Эйлера в радианах anglesOrient, а также специальные флаги 
areAnglesUsed и isAngleSet, которые показывают, используется ли начальная ориентация, 
то есть имеется ли хотя бы один угол Эйлера, не равный нулю, и какие из углов заданы, 
то есть какие отличны от нуля. Эти флаги позволяют ускорить установку 
конфигурации манипулятора в процессе планирования траектории.
**************************************************************************************************/



#pragma once

#include "main/env/env_def.h"
#include "main/env/primitives/obb/obb.h"
#include "main/env/primitives/attached_detail/attached_detail.h"
#include "main/env/manipulator/manip_parameters.h"
#include "main/env/obstacles/obst_parameters.h"


namespace motion_planner
{
    namespace env
    {
        struct ManipInitInfo
        {
            struct LinkInitInfo
            {
                Obb body;

                Eigen::Vector3f initMountPos = Eigen::Vector3f::Zero();

                uint8_t axisRot = 2;

                uint16_t groupId = 0;

                uint16_t mountId = 0;

                EulerAngles anglesOrient = EulerAngles( { 0.f, 0.f, 0.f } );
                bool isAngleSet[3] = { false, false, false };
                bool areAnglesUsed = false;

                explicit LinkInitInfo( 
                    Obb::Sizes sizes_,
                    Eigen::Vector3f centerPos_,
                    Eigen::Vector3f mountPos_,
                    uint16_t mountId_,
                    uint8_t axis_,
                    uint16_t id_,
                    const float (&angles_)[ 3 ],
                    const bool (&isAngleSet_)[ 3 ],
                    bool areAnglesUsed_
                ) :
                    body( sizes_, centerPos_ ),
                    initMountPos( mountPos_ ),
                    mountId( mountId_ ),
                    axisRot( axis_ ),
                    groupId( id_ ),
                    areAnglesUsed( areAnglesUsed_ ),
                    anglesOrient( { angles_[ 0 ], angles_[ 1 ], angles_[ 2 ] } )
                {
                    std::copy( isAngleSet_, isAngleSet_ + 3, isAngleSet );
                }

                explicit LinkInitInfo()
                {}
            };
            LinkInitInfo linksData[ manip::links_amount ];
        };

        struct ObstInitInfo;
    }
}



struct motion_planner::env::ObstInitInfo
{
    AttachedDetail obsts[ obst::obst_amount ];
};

