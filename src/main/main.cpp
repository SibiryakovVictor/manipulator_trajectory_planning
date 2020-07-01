#include "project_config.h"

#include "motion_planner/motion_planner.h"
#include "motion_planner/task_planning.h"




int main(void)
{
    using namespace motion_planner;
    
	// задание шага траектории в радианах и создание планировщика
	float stepRad = flt_op::cvtDegToRad( 1.f );
	MotionPlanner planner( stepRad );
	
	// задание стартовой и целевой конфигураций 
	config_space::Point startConfig( { -0.14404736472671, 1.553343034275, -0.25598925748466,
		-3.1241393610699, 0.79116183965575, -0.13657032990509, 0.13657032990509	} );

	config_space::Point goalConfig( { -3.1241393610699, -1.553343034275, -0.088351084933012,
		3.1241393610699, -3.1241393610699, 0.52323984496597, -0.52323984496597 } );

	planner.findPath( startConfig, goalConfig ); //запуск поиска пути

	if ( planner.isPathFound() )
	{
		while ( ! planner.isPathPassed() )
		{
			config_space::Point currentPointTraj( planner.getPointTraj() );

			// выполнять какие-либо манипуляции с точками траектории
		}
	}            
        
    while(1);
}

