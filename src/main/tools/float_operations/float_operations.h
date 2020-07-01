/**************************************************************************************************
Описание

Вспомогательные функции

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/


#pragma once


namespace motion_planner
{
    namespace flt_op
    {
        const float mathPI = 3.141592653589793238462643f;

        float getRandomFloat( float lowLimit, float highLimit );

        float cvtDegToRad( float degrees );

        float cvtRadToDeg( float radians );
    }
}

