/**************************************************************************************************
Описание

Класс, позволяющий настраивать объекты планировщика MotionPlanner

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/


#include "configurator.h"

using namespace motion_planner;



/**************************************************************************************************
Описание:
изменяет ограничивающий объем заданного объекта;
для задания объекта использовать константы biasObstId (для препятствия) и biasLinkId (для звена)

пример значения параметра objId при обращении к препятствию на позиции X в списке контейнера Obstacles:
Obstacles::biasObstId + X

пример значения параметра objId при обращении к телу манипулятора на позиции Y в списке m_initLinksInfo
объекта класса ManipulatorController: ManipulatorController::biasLinkId + Y

Аргументы:
* objId: идентификатор объекта
* body: заменяющий ограничивающий объем
Возврат:
**************************************************************************************************/
void Configurator::changeObjectBody( uint16_t objId, env::Obb body )
{
    if ( objId >= env::Obstacles::biasObstId )
    {
        m_obst.changeBody( objId, body );
    }
    else
    {
        m_obst.changeBody( objId, body );
    }
}



/**************************************************************************************************
Описание:
выполняет поворот препятствия на позиции obstId в контейнере Obstacles
(про обращение к объектам см. описание метода changeObjectBody)
на заданный угол angleRad относительно оси axis
Аргументы:
* obstId: идентификатор препятствия
* angle_Rad: угол поворота
* axis: ось поворота
Возврат:
**************************************************************************************************/
void Configurator::rotateObst( uint16_t obstId, float angle_Rad, Eigen::Vector3f axis )
{
    m_obst.rotate( obstId, angle_Rad, axis );
}



/**************************************************************************************************
Описание:
активирует пару проверяемых на пересечение объектов в таблице сталкиваемых объектов,
то есть при проверке конфигурации на наличие пересечений эта пара будет учитываться
Аргументы:
* pairIndex: позиция пары в массиве list_check_pairs файла "env/collision_detection/collision_check_pairs.h"
Возврат:
**************************************************************************************************/
void Configurator::activateCollPair( uint16_t pairIndex )
{
    m_validator.m_collTable.activatePair( pairIndex );
}



/**************************************************************************************************
Описание:
деактивирует пару проверяемых на пересечение объектов в таблице сталкиваемых объектов,
то есть при проверке конфигурации на наличие пересечений эта пара не будет учитываться
Аргументы:
* pairIndex: позиция пары в массиве list_check_pairs файла "env/collision_detection/collision_check_pairs.h"
Возврат:
**************************************************************************************************/
void Configurator::deactivateCollPair( uint16_t pairIndex )
{
    m_validator.m_collTable.deactivatePair( pairIndex );
}



/**************************************************************************************************
Описание:
меняет параметр алгоритма EST - радиус r поиска ближайших соседей от опорной вершины
Аргументы:
* radiusGen: значение параметра
Возврат:
**************************************************************************************************/
void Configurator::changeEstRadiusGen( float radiusGen )
{
    m_planner.m_radiusGen = m_planner.calcRadiusGen( radiusGen );
}



/**************************************************************************************************
Описание:
меняет параметр алгоритма EST - расстояние d между парой вершин, принадлежащих разным деревьям, 
на котором деревья могут быть соединены
Аргументы:
* radiusGen: значение параметра
Возврат:
**************************************************************************************************/
void Configurator::changeEstRadiusConn( float radiusConn )
{
    m_planner.m_radiusConn = m_planner.calcRadiusConn( radiusConn );
}
