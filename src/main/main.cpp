#include "project_config.h"

#include "motion_planner/motion_planner.h"
#include "motion_planner/task_planning.h"




int main(void)
{
    using namespace motion_planner;
    
    MotionPlanner planner( path::traj_prec );
    
    TaskPlanning task( config_space::Point( { -0.14404736472671, 1.553343034275, -0.25598925748466, -3.1241393610699, 0.79116183965575, -0.13657032990509, -0.1858882412183	 } ),
		config_space::Point( { -3.1241393610699, -1.553343034275, -0.088351084933012, 3.1241393610699, -3.1241393610699, 0.52323984496597, 0.23561944901923	 } ) );
        
    planner.findPath( task.start, task.goal );

    if ( planner.getSearchResult() == MotionPlanner::SearchResult::SUCCESS )
    {
        planner.resetStepPath();
    }            
        

    while(1);

    return 0;
}

