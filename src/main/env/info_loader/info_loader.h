/**************************************************************************************************
Описание

Определение функций, которые используют конструкторы классов ManipulatorController и Obstacles
для загрузки информации о манипуляторе и препятствиях

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/



#pragma once

#include "init_typedefs.h"


namespace motion_planner
{
    namespace env
    {
        namespace load
        {
            ManipInitInfo getManipInitInfo();

            ObstInitInfo getObstInitInfo();
        }
    }
}

