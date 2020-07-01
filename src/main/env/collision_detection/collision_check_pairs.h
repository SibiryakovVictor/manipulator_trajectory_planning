/**************************************************************************************************
Описание

Определение таблицы сталкиваемых объектов

Разработчик: Сибиряков Виктор
Заметки
Правила заполнения таблицы:
*** Вид объекта (препятствие, звено или группа звеньев) в паре указывается с помощью
специального числового идентификатора (obsts, links, или groups соответственно).
Конкретный объект внутри вида указывается в соответствии с порядком, в котором 
они определены в функции getManipInitInfo или getObstInitInfo.
Таким образом, для задания объекта в паре необходимо записать 
выражение "идентификатор вида объекта + порядковый номер объекта".
*** Первый объект в паре должен принадлежать манипулятору 
(то есть иметь идентификатор вида объекта, равный links или groups).
*** Пары в таблицу заносятся в порядке, при котором величина идентификатора первого в паре объекта
возрастает. При занесении группы звеньев для соблюдения этого правила необходимо сравнивать
величину порядкового номера первого звена в этой группе с величинами идентификатора 
первого объекта в парах, уже имеющихся в таблице. В случае занесения группы проверка 
осуществляется между всеми звеньями этой группы и вторым объектом в паре.
*** Если оба объекта в паре принадлежат манипулятору, то величина идентификатора 
первого объекта в паре всегда должна быть больше второго. Если среди объектов в паре есть 
группа, то сравнение с другим объектом необходимо проводить, используя величину 
порядкового номера первого звена в этой группе.
**************************************************************************************************/


#pragma once

#include "main/env/obstacles/obstacles.h"
#include "main/env/manipulator/manipulator_controller.h"


namespace motion_planner
{
    namespace coll
    {
        struct CheckPair
        {
            uint16_t body1 = UINT16_MAX;
            uint16_t body2 = UINT16_MAX;
            
            CheckPair()
            {}
            
            explicit CheckPair( uint16_t b1, uint16_t b2 ) :
                body1( b1 ),
                body2( b2 )
            {}
                
        };

        static const uint16_t pairsAmount =
            manip::links_amount * obst::obst_amount;

        static const uint16_t obsts = env::Obstacles::biasObstId;
        static const uint16_t links = env::ManipulatorController::biasLinkId;
        static const uint16_t groups = env::ManipulatorController::biasGroupId;


        static const coll::CheckPair list_check_pairs[ pairsAmount ] = {
            CheckPair( links + 1, obsts + 2 ), CheckPair( links + 1, obsts + 3 ),
            CheckPair( groups + 1, obsts + 0 ), CheckPair( groups + 1, obsts + 1 ), 
            CheckPair( groups + 1, obsts + 2 ), CheckPair( groups + 1, obsts + 3 ),
            CheckPair( links + 4, obsts + 0 ), CheckPair( links + 4, obsts + 1 ),
            CheckPair( links + 4, obsts + 2 ), CheckPair( links + 4, obsts + 3 ),
            CheckPair( links + 5, obsts + 0 ), CheckPair( links + 5, obsts + 1 ),
            CheckPair( links + 5, obsts + 2 ), CheckPair( links + 5, obsts + 3 ),
            CheckPair( links + 5, obsts + 4 ), CheckPair( links + 5, obsts + 5 ),
            CheckPair( links + 5, links + 1 ), CheckPair( links + 5, links + 2 ),
            CheckPair( links + 6, links + 4 ),
            CheckPair( groups + 5, obsts + 0 ), CheckPair( groups + 5, obsts + 1 ),
            CheckPair( groups + 5, obsts + 2 ), CheckPair( groups + 5, obsts + 3 ),
            CheckPair( groups + 5, obsts + 4 ), CheckPair( groups + 5, obsts + 5 ),
            CheckPair( groups + 5, links + 1 ), CheckPair( groups + 5, links + 2 ),
            CheckPair( links + 8, links + 4 ),
            CheckPair( groups + 6, obsts + 0 ), CheckPair( groups + 6, obsts + 1 ),
            CheckPair( groups + 6, obsts + 2 ), CheckPair( groups + 6, obsts + 3 ),
            CheckPair( groups + 6, obsts + 4 ), CheckPair( groups + 6, obsts + 5 ),
            CheckPair( groups + 6, links + 1 ), CheckPair( groups + 6, links + 2 ),
            CheckPair( links + 10, links + 4 ), CheckPair( links + 10, links + 8 )
        };

    }
}
