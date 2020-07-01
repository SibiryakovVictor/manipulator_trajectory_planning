/**************************************************************************************************
Описание

Определение параметров манипулятора

Разработчик: Сибиряков Виктор
Заметки
* комментарии прилагаются
**************************************************************************************************/


#pragma once

#include <cinttypes>

#include "main/config_space/conf_space_dims.h"



namespace motion_planner
{
    namespace manip
    {
        // тип, содержащий индекс звена src, которое влияет своей ориентацией
        // на звено под индексом dst
        struct OrientDependency
        {
            uint16_t src = 0;
            uint16_t dst = 0;
            
            explicit OrientDependency()
            {}
                
            explicit OrientDependency( uint16_t srcLinkId, uint16_t dstLinkId ) :
                src( srcLinkId ),
                dst( dstLinkId )
            {}
               
        };


        // число групп должно совпадать с размерностью конфигурационно пространства (см. conf_space_dims.h)
        const uint8_t links_groups = conf_space_dims;


        // количество звеньев (или, если точнее, количество ограничивающих объемов, описывающих
        // манипулятор). ДОЛЖНО!!! совпадать с числом структур LinkInitInfo, возвращаемых
        // функцией getManipInitInfo (env/info_loader.cpp)
        const uint8_t links_amount = 11;


        // количество групп звеньев из двух и более звеньев (или, если точнее, это число должно быть
        // равно количеству индексов групп, повторяющихся 2 и более раза в структурах LinkInitInfo,
        // возвращаемых функцией getManipInitInfo (env/info_loader.cpp)
        const uint8_t links_complex_amount = 4;


        // для каждой позиции звена в массиве должно возвращать позицию последнего звена, ориентация
        // в пространстве которого меняется при изменении ориентации звена на этой позиции
        const uint8_t links_align_last[ links_amount ] = { 
            links_amount, links_amount, links_amount, links_amount, links_amount,
            links_amount, links_amount, 9, 9, links_amount, links_amount };
    }
}
