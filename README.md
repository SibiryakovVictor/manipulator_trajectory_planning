# Решение для планирования траектории движения манипулятора в среде с препятствиями
## Платформа
STM32F407 Discovery
## IDE
Keil µVision
## Код проекта
src/main
## Ограничения в использовании
Возможна работа с манипулятором ТОЛЬКО с вращательными (одноподвижными) кинематическими парами
## Загрузка манипулятора и препятствий (ручной способ)
### Описание звеньев манипулятора
#### Занести информацию о звеньях манипулятора в функцию getManipInitInfo с помощью структур LinkInitInfo
- Определение функции getManipInitInfo: env/info_loader/info_loader.cpp
- Определение структуры данных LinkInitInfo: env/info_loader/init_typedefs.h
- Порядок занесения: от корня манипулятора и дальше в порядке присоединения звеньев друг к другу
- Пример заполнения структуры:

![Image alt](https://github.com/SibiryakovVictor/training_release_rep/raw/master/link_init_info_example.png)

#### Добавить все заполненные структуры в оператор возврата return функции getManipInitInfo (env/info_loader/info_loader.cpp)

Пример:

![Image alt](https://github.com/SibiryakovVictor/training_release_rep/raw/master/getManipInitInfo_return_example.png)

#### Уточнить параметры добавляемого манипулятора в env/manipulator/manip_parameters.h и config_space/conf_space_dims.h

#### Определить диапазоны движения звеньев в tools/intervals.h

Пример

![Image alt](https://github.com/SibiryakovVictor/training_release_rep/raw/master/ranges_motion_example.png)

### Описание препятствий
#### Занести информацию о препятствиях в функцию getObstInitInfo с помощью структур AttachedDetal
- Определение функции getObstInitInfo: env/info_loader/info_loader.cpp
- Определение структуры данных AttachedDetail: env/primitives/attached_detail
- Порядок занесения: произвольный
- Пример заполнения:

![Image alt](https://github.com/SibiryakovVictor/training_release_rep/raw/master/attached_detail_example.png)

#### Добавить все заполненные структуры в оператор возврата return функции getObstInitInfo (env/info_loader/info_loader.cpp)

Пример

![Image alt](https://github.com/SibiryakovVictor/training_release_rep/raw/master/getObstInitInfo_return_example.png)

#### Уточнить параметры препятствий в env/obstacles/obst_parameters.h и config_space/conf_space_dims.h

### Определение объектов, между которыми возможно столкновение

- указать пары сталкиваемых объектов в **env/collision_detection/collision_check_pairs.h**

## Загрузка манипулятора и препятствий (сцена в робосимуляторе V-REP)

*в процессе оформления*

## Пример использования

![Image alt](https://github.com/SibiryakovVictor/training_release_rep/raw/master/usage_MotionPlanner_example.png)

## Дополнительные возможности при использовании

- смотреть tools/configurator
