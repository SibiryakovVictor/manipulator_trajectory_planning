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

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/link_init_info_example.jpg)

#### Добавить все заполненные структуры в оператор возврата return функции getManipInitInfo (env/info_loader/info_loader.cpp)

Пример:

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/getManipInitInfo_return_example.png)

#### Уточнить параметры добавляемого манипулятора в env/manipulator/manip_parameters.h и config_space/conf_space_dims.h

#### Определить диапазоны движения звеньев в tools/intervals.h

Пример

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/ranges_motion_example.png)

### Описание препятствий
#### Занести информацию о препятствиях в функцию getObstInitInfo с помощью структур AttachedDetal
- Определение функции getObstInitInfo: env/info_loader/info_loader.cpp
- Определение структуры данных AttachedDetail: env/primitives/attached_detail
- Порядок занесения: произвольный
- Пример заполнения:

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/attached_detail_example.png)

#### Добавить все заполненные структуры в оператор возврата return функции getObstInitInfo (env/info_loader/info_loader.cpp)

Пример

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/getObstInitInfo_return_example.png)

#### Уточнить параметры препятствий в env/obstacles/obst_parameters.h и config_space/conf_space_dims.h

### Определение объектов, между которыми возможно столкновение

- указать пары сталкиваемых объектов в **env/collision_detection/collision_check_pairs.h**

## Загрузка манипулятора и препятствий (сцена в робосимуляторе CoppeliaSim (V-REP))

### Подготовка робосимулятора CoppeliaSim (V-REP)

#### Установить робосимулятор 

https://www.coppeliarobotics.com/downloads

#### Положить файл env_info_code_data.lua в папку lua, находящуюся в корне директории робосимулятора

#### Запустить робосимулятор

- открыть сцену **env_info_generator.ttt**, лежащую в корне репозитория

### Занесение информации о манипуляторе

#### Добавить на сцену ограничивающие параллелепипеды звеньев с помощью объектов Cuboid (Add -> Primitive shape -> Cuboid)

Пример (манипулятор из env_info_generator.ttt)

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/manip_img.jpg)

#### Добавить информацию о звеньях манипулятора с помощью изменения имён объектов их ограничивающих параллелепипедов

Правила именования объектов звеньев:

**link_idA_grB_moC_alD_axis**

- A: порядковый номер звена, начинающийся от 0 в корне манипулятора и возрастающий на 1 у каждого последующего звена.
- B: числовой идентификатор группы, к которой относится звено. У звеньев, которые должны рассматриваться как группа, то есть часть 
одного составного звена, значения этого идентификатора равны, в таком случае для всех 
этих частей будет задаваться одна общая конфигурация. Нумерация групп начинается с 0 и возрастает на 1 для каждой следующей группы.
- C: идентификатор A предыдущего звена, с которым механически связано текущее звено.
- D: идентификатор A последнего звена в иерархии манипулятора, ориентация в пространстве которого меняется с изменением ориентации текущего звена. Пример: при вращении корня манипулятора изменяется ориентация всех звеньев манипулятора, а при вращении левого пальца схвата ориентация правого пальца схвата остается неизменной (при задании конфигурации манипулятора, поэтому для пальцев схвата используются разные конфигурации; в случае пальцев схвата необходимо задавать одинаковые по величине и противоположные по знаку значения)
- axis: ось вращения (X, Y, Z)

Пример именования объектов звеньев (манипулятор из env_info_generator.ttt)

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/links_names.jpg)

#### Добавить точки поворота звеньев относительно друг друга (точки крепления) с помощью объектов Dummy (Add -> Dummy)

Пример (точка поворота звена, выделенного красным, относительно звена, выделенного синим)

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/mount_link_example.jpg)

Правило именования точек крепления:

**mount_link_idA**

- A: номер звена, которое поворачивается относительно этой точки

Пример именования точек крепления

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/mount_link_names.jpg)

#### Уточнить параметры добавляемого манипулятора в env/manipulator/manip_parameters.h и config_space/conf_space_dims.h

#### Определить диапазоны движения звеньев в tools/intervals.h

### Занесение информации о препятствиях

#### Добавить на сцену ограничивающие параллелепипеды препятствий с помощью объектов Cuboid (Add -> Primitive shape -> Cuboid)

Пример (препятствия из env_info_generator.ttt)

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/obsts_img.jpg)

#### Добавить информацию о препятствиях с помощью изменения имён объектов их ограничивающих параллелепипедов

Правила именования объектов препятствий:

1 вариант

**obst_idA_**

- A: порядковый номер препятствия, начинающийся от 0 и возрастающий на 1 у каждого следующего препятствия в списке.

2 вариант

**obst_idA_moNo**

- moNo: добавить, если не требуется задание точки крепления препятствия (относительно которой возможен поворот)

Пример именования объектов препятствий (препятствия из env_info_generator.ttt)

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/obsts_names.jpg)

#### Добавить точки поворота требуемых препятствий с помощью объектов Dummy (Add -> Dummy)

Пример (точка поворота препятствия, выделенного красным)

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/mount_obst_example.jpg)

Правило именования точек крепления:

**mount_obst_idA**

- A: номер препятствия, которое поворачивается относительно этой точки (имя препятствия должно быть вида **obst_idA_**)

Пример именования точек крепления

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/mount_obst_names.jpg)

#### Уточнить параметры препятствий в env/obstacles/obst_parameters.h и config_space/conf_space_dims.h

### Добавление ограничивающих объемов, используемых для замены объемов звеньев манипулятора или препятствий

#### Добавить на сцену дополнительные ограничивающие параллелепипеды с помощью объектов Cuboid (Add -> Primitive shape -> Cuboid)

#### Добавить информацию о дополнительных ограничивающих параллелепипедах с помощью изменения их имён

Пример

**otherBody_idA_objB**

- A: порядковый номер, начинающийся от 0 и возрастающий на 1 у каждого следующего дополнительного ограничивающего объема в списке.
- obj: link или obst в зависимости от назначения (звено или препятствие)
- B: порядковый номер указанного объекта

Пример именования дополнительных ограничивающих объемов

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/otherBody_names.jpg)

После добавления ограничивающего объема с определенным именем в коде проекта появится функция, возвращающая этот ограничивающий объем с именем:

**getOtherBodyA_objB**

Пример смены ограничивающего объема:

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/otherBody_example.jpg)

- про идентификаторы объектов смотреть **env/collision_detection/collision_check_pairs.h**

### Определение объектов, между которыми возможно столкновение

- указать пары сталкиваемых объектов в **env/collision_detection/collision_check_pairs.h**

## Пример использования

![Image alt](https://github.com/SibiryakovVictor/manipulator_trajectory_planning/raw/master/images/usage_MotionPlanner_example.jpg)

## Дополнительные возможности при использовании

- смотреть tools/configurator
