/**************************************************************************************************
Описание

Описывает контейнер препятствий

Разработчик: Сибиряков Виктор
Заметки
* Configurator - дружественный класс, чтобы менять ограничивающий объем звена и осуществлять
поворот вокруг заданной оси на заданный угол;
**************************************************************************************************/



#include "obstacles.h"

using namespace motion_planner::env;
using namespace motion_planner;



/**************************************************************************************************
Описание:
возвращает OBB препятствия по смещенному идентификатору, заданному относительно константы biasObstId
пример запроса OBB (значения параметра obstBiasedId) на позиции k в контейнере: biasObstId + k; 
Аргументы:
* config: конфигурация текущего звена/группы звеньев
Возврат:
**************************************************************************************************/
const Obb & Obstacles::getBody( uint16_t obstBiasedId ) const
{
    return m_obstInfo.obsts[ obstBiasedId - biasObstId ];
}



void Obstacles::changeBody( uint16_t obstBiasedId, const env::Obb & body )
{
    m_obstInfo.obsts[ obstBiasedId - biasObstId ] = body;
}



void Obstacles::rotate( uint16_t obstBiasedId, float angle, Eigen::Vector3f axis )
{
    m_obstInfo.obsts[ obstBiasedId - biasObstId ].rotateAroundAnchor( angle, axis );
}
