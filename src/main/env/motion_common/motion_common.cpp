/**************************************************************************************************
Описание

Набор функций для вращения векторов и матриц, а также обертка функционала Eigen
для получения матрицы поворота вокруг заданной оси на заданной угол

Разработчик: Сибиряков Виктор
Заметки
**************************************************************************************************/



#include "motion_common.h"

using namespace motion_planner::env;



/**************************************************************************************************
Описание:
возвращает единичный вектор соответстующей оси (X, Y, Z)
Аргументы:
* axis: идентификатор оси ( 0 - X, 1 - Y, 2 - Z )
Возврат: единичный вектор соответстующей оси (X, Y, Z)
**************************************************************************************************/
const Eigen::Vector3f & motion_planner::env::getUnitAxis( uint8_t axis )
{
    static const Eigen::Vector3f unitX = Eigen::Vector3f::UnitX();
    static const Eigen::Vector3f unitY = Eigen::Vector3f::UnitY();
    static const Eigen::Vector3f unitZ = Eigen::Vector3f::UnitZ();

    switch ( axis )
    {

    case ( 0 ) :
    {
        return unitX;
    }
    case ( 1 ) :
    {
        return unitY;
    }
    default :
    {
        return unitZ;
    }

    }
}



/**************************************************************************************************
Описание:
возвращает матрицу поворота вокруг оси rotAxis на угол angle_Rad
Аргументы:
* axis: идентификатор оси ( 0 - X, 1 - Y, 2 - Z )
Возврат: единичный вектор соответстующей оси (X, Y, Z)
**************************************************************************************************/
Eigen::Matrix3f motion_planner::env::getRotMat( const float & angle_Rad, const Eigen::Vector3f & rotAxis )
{
    return Eigen::AngleAxis< float >( angle_Rad, rotAxis ).toRotationMatrix();
}
Eigen::Matrix3f motion_planner::env::getRotMat( const float & angle_Rad, Eigen::Vector3f && rotAxis )
{
    return Eigen::AngleAxis< float >( angle_Rad, rotAxis ).toRotationMatrix();
}



/**************************************************************************************************
Описание:
осуществляет поворот вектора v, переданного по ссылке, на заданный угол angle_Rad относительно 
оси rotAxis
Аргументы:
* v: поворачиваемый вектор;
* angle_Rad: угол поворота;
* rotAxis: ось вращения
Возврат:
**************************************************************************************************/
void motion_planner::env::rotateVector( Eigen::Vector3f & v, const float & angle_Rad, 
    const Eigen::Vector3f & rotAxis )
{
    v = getRotMat( angle_Rad, rotAxis ) * v;
}



/**************************************************************************************************
Описание:
осуществляет поворот вектора v, на заданный угол angle_Rad относительно оси rotAxis
с возвратом получившегося результата
Аргументы:
* v: поворачиваемый вектор;
* angle_Rad: угол поворота;
* rotAxis: ось вращения
Возврат: повернутый вектор
**************************************************************************************************/
Eigen::Vector3f motion_planner::env::rotateVector( const Eigen::Vector3f & v, const float & angle_Rad, 
    const Eigen::Vector3f & rotAxis )
{
    return getRotMat( angle_Rad, rotAxis ) * v;
}
Eigen::Vector3f motion_planner::env::rotateVector( Eigen::Vector3f && v, const float & angle_Rad, 
    const Eigen::Vector3f & rotAxis )
{
    return getRotMat( angle_Rad, rotAxis ) * v;
}



/**************************************************************************************************
Описание:
осуществляет поворот матрицы m, переданной по ссылке, на заданный угол angle_Rad относительно 
оси rotAxis
Аргументы:
* m: поворачиваемая матрица;
* angle_Rad: угол поворота;
* rotAxis: ось вращения
Возврат:
**************************************************************************************************/
void motion_planner::env::rotateOrient( Eigen::Matrix3f & m, const float & angle_Rad,
    const Eigen::Vector3f & rotAxis )
{
    m = getRotMat( angle_Rad, rotAxis ) * m;
}



/**************************************************************************************************
Описание:
осуществляет поворот матрицы m, на заданный угол angle_Rad относительно оси rotAxis
с возвратом получившегося результата
Аргументы:
* m: поворачиваемая матрица;
* angle_Rad: угол поворота;
* rotAxis: ось вращения
Возврат: повернутая матрица
**************************************************************************************************/
Eigen::Matrix3f motion_planner::env::rotateOrient( const Eigen::Matrix3f & m, const float & angle_Rad, 
    const Eigen::Vector3f & rotAxis )
{
    return getRotMat( angle_Rad, rotAxis ) * m;
}
Eigen::Matrix3f motion_planner::env::rotateOrient( Eigen::Matrix3f && m, const float & angle_Rad, 
    const Eigen::Vector3f & rotAxis )
{
    return getRotMat( angle_Rad, rotAxis ) * m;
}
