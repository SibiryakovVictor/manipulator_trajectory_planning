/**************************************************************************************************
Описание

Определение параметров пути

Разработчик: Сибиряков Виктор
Заметки
* комментарии прилагаются
**************************************************************************************************/


#pragma once

#include <cmath>

#include "main/tools/float_operations/float_operations.h"
#include "main/config_space/conf_space_dims.h"



namespace motion_planner
{
    namespace path
    {
        // число секций пути (или размер массива sections структуры Path, определенной в "path/path_typedefs.h"
        // взято с запасом, обычно путь состоит из 5-9 секций, максимум встречалось 15...
        // ПРИ ИЗМЕНЕНИИ уточнить тип PathElemId в "path/path_typedefs.h" (должен вмещать length_limit)
        const uint8_t length_limit = 60;


        // точность найденной планировщиком траектории манипулятора ПО УМОЛЧАНИЮ
        // (можно менять методом changeTrajStep класса MotionPlanner)
        // обозначает то, что евклидово расстояние между точками траектории 
        // не будет превышать данную величину
        const float traj_prec_Degrees = 1.f;


        // отношение точности траектории к шагу проверки точек траектории на наличие пересечений
        // (просто чтобы значение этого шага по умолчанию как-то коррелировало с точностью траектории)
        const float ratio_traj_prec_check = 2.f;


        // шаг проверки точек траектории (значение обозначает то же, что и значение traj_prec_Degrees)
        const float check_traj_prec_Degrees = traj_prec_Degrees / ratio_traj_prec_check;



        
        inline float calcMaxDimValue_Rad( float prec_Degree )
        {
            return std::sqrtf( 
                std::powf( flt_op::cvtDegToRad( prec_Degree ), 2.f ) / static_cast< float >( conf_space_dims ) );
        }
    }
}
