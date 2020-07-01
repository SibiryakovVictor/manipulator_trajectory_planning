/**************************************************************************************************
Описание

Генератор последовательных точек найденной траектории

Разработчик: Сибиряков Виктор
Заметки
* под шагом прохождения траекториии понимается максимально евклидово расстояние в конфигурационном пространстве
между точками траектории
**************************************************************************************************/



#include "path_generator.h"


using namespace motion_planner::path;
using namespace motion_planner::config_space;




/**************************************************************************************************
Описание:
возвращает ссылку на список секций пути Path
Аргументы:
Возврат:
**************************************************************************************************/
Path & PathGenerator::getPathStorage()
{
    return m_path;
}



/**************************************************************************************************
Описание:
устанавливает количество точек пути
Аргументы:
Возврат:
**************************************************************************************************/
void PathGenerator::setPointsAmount( PathElemId pathLength )
{
	m_pointsAmount = pathLength;
}



/**************************************************************************************************
Описание:
возвращает количество точек пути
Аргументы:
Возврат:
**************************************************************************************************/
PathElemId PathGenerator::getPointsAmount() const
{
	return m_pointsAmount;
}



/**************************************************************************************************
Описание:
возвращает шаг прохождения траектории (см. "Заметки" в шапке)
Аргументы:
Возврат: шаг прохождения траектории
**************************************************************************************************/
float PathGenerator::getStep_Degree() const
{
	return m_step_Degree;
}



/**************************************************************************************************
Описание:
проверяет, была ли достигнута последняя точка построенной траектории в результате вызовов
метода getNextPointPath
Аргументы:
Возврат: результат проверки на достижение последней точки построенной траектории
**************************************************************************************************/
bool PathGenerator::isPathPassed() const
{
	return ( m_curElem > m_pointsAmount - 2 );
}



/**************************************************************************************************
Описание:
возвращает следующую точку траектории, начиная со стартовой и заканчивая целевой
Аргументы:
Возврат: точка траектории
**************************************************************************************************/
Point PathGenerator::getNextPointPath() const
{
	if ( ! m_trajIt.isPassed() )
	{
		return m_trajIt.getNextPoint();
	}


	m_curElem++;

	if ( isPathPassed() )
	{
		m_curElem = m_pointsAmount - 1;

		return m_trajIt.end();
	}

	setCurElemTrajIt();

	return m_trajIt.getNextPoint();
}



/**************************************************************************************************
Описание:
сбрасывает значение m_curElem, в результате метод getNextPointPath начнёт 
выдавать точки траектории с начала
Аргументы:
Возврат:
**************************************************************************************************/
void PathGenerator::resetPointGetter() const
{
	m_curElem = 0;

	setCurElemTrajIt();
}



/**************************************************************************************************
Описание:
удаляет данные о найденном пути (очищает список m_path) и сбрасывает счётчик m_curElem
Аргументы:
Возврат:
**************************************************************************************************/
void PathGenerator::reset()
{
	std::fill( m_path.sections, m_path.sections + m_pointsAmount, empty_path_elem );

	m_pointsAmount = 0;

	m_curElem = 0;

	m_trajIt.resetStep();
}



/**************************************************************************************************
Описание:
возвращает текущую секцию пути (которая "проходится" методом getNextPointPath)
Аргументы:
Возврат: секция пути
**************************************************************************************************/
PathElement PathGenerator::getCurPathElem() const
{
	return m_path.sections[ m_curElem ];
}



/**************************************************************************************************
Описание:
возвращает секцию пути на позиции elemPos в списке секций m_path
Аргументы:
Возврат: секция пути
**************************************************************************************************/
PathElement PathGenerator::getPathElem( PathElemId elemPos ) const
{
	return m_path.sections[ elemPos ];
}



/**************************************************************************************************
Описание:
изменяет шаг прохождения траектории (см. "Заметки" в шапке); 
при этом поле m_curElem сбрасывается, и при вызове метода getNextPointPath точки траектории
будут выдаваться заново
Аргументы: шаг траектории
Возврат:
**************************************************************************************************/
void PathGenerator::changeStep( float step_Degree )
{
	m_step_Degree = step_Degree;

	m_maxDimVal_Rad = calcMaxDimValue_Rad( step_Degree );

	resetPointGetter();
}



void PathGenerator::setCurElemTrajIt() const
{
	m_trajIt.setTrajectory(
		m_nodesList.getNodeConfig( m_path.sections[ m_curElem ].point ),
		m_nodesList.getNodeConfig( m_path.sections[ m_curElem + 1 ].point ),
		m_maxDimVal_Rad
	);
}