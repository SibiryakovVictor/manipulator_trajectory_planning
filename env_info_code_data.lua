include = '#include "info_loader.h"' .. '\n\n'

namespace = 'using namespace motion_planner;' .. '\n\n'

common_header_cpp = include .. namespace

common_header_h = '#pragma once\n\n#include "init_typedefs.h"\n\n' 
.. 'namespace motion_planner\n{\n\t' .. 'namespace env\n\t{\n\t\t' .. 'namespace load\n\t\t{\n\t\t\t' 
.. 'ManipInitInfo getManipInitInfo();\n\n\t\t\t' .. 'ObstInitInfo getObstInitInfo();\n\n\t\t\t'

common_footer_h = '\n\t\t}\n\t}\n}\n\n'



funcManipInfoHeader = '\n' .. 'env::ManipInitInfo env::load::getManipInitInfo()' .. '\n' .. '{' .. '\n'
initStructManip = '\n' .. '\t' .. 'ManipInitInfo manipInfo;' .. '\n'


funcObstInfoHeader = '\n' .. 'env::ObstInitInfo env::load::getObstInitInfo()' .. '\n' .. '{' .. '\n'



function fillLinkInfo ( linkId, sizes, centerPos, mountPos, mountId, lastAlign, axis, groupId, anglesOrient, isAngleSet, areAnglesUsed )
  
  local beforeId = '\t' .. 'ManipInitInfo::LinkInitInfo link'
  local afterId = 'Info(' .. '\n'
  local completedId = beforeId .. linkId .. afterId
  
  
  local beforeSizes = '\t\t' .. 'Obb::Sizes( { '
  local afterSizes = ' } ), '
  local commentSizes = '/* sizes */'
  local completedSizes = beforeSizes .. sizes[ 'x' ] .. ', ' .. sizes[ 'y' ] .. ', ' .. 
  sizes[ 'z' ] .. afterSizes .. commentSizes .. '\n'
  
  
  local beforeCenterPos = '\t\t' .. 'Eigen::Vector3f( '
  local afterCenterPos = ' ), '
  local commentCenterPos = '/* initCenterPos */'
  local completedCenterPos = beforeCenterPos .. centerPos[ 'x' ] .. ', ' .. centerPos[ 'y' ] .. ', ' .. centerPos[ 'z' ] 
  .. afterCenterPos .. commentCenterPos .. '\n'
  
  
  local beforeMountPos = '\t\t' .. 'Eigen::Vector3f( '
  local afterMountPos = ' ), '
  local commentMountPos = '/* initMountPos */'
  local completedMountPos = beforeMountPos .. mountPos[ 'x' ] .. ', ' .. mountPos[ 'y' ] .. ', ' .. mountPos[ 'z' ]
  .. afterMountPos .. commentMountPos .. '\n'
  
  
  local beforeMountId = '\t\t'
  local afterMountId = ', '
  local commentMountId = '/* mountId */'
  local completedMountId = beforeMountId .. mountId .. afterMountId .. commentMountId .. '\n'
  
  
  local beforeLastAlign = '\t\t'
  local afterLastAlign = ', '
  local commentLastAlign = '/* lastAlign */'
  local completedLastAlign = beforeLastAlign .. lastAlign .. afterLastAlign .. commentLastAlign .. '\n'
  
  
  local beforeAxisRot = '\t\t'
  local afterAxisRot = ', '
  local commentAxisRot = '/* axisRot */'
  local completedAxisRot = beforeAxisRot .. axis .. afterAxisRot .. commentAxisRot .. '\n'
  
  
  local beforeGroupId = '\t\t'
  local afterGroupId = ', '
  local commentGroupId = '/* groupId */'
  local completedGroupId = beforeGroupId .. groupId .. afterGroupId .. commentGroupId .. '\n'
  
  
  local beforeAnglesOrient = '\t\t{ '
  local afterAnglesOrient = ' }, '
  local commentAnglesOrient = '/* anglesOrient */'
  local completedAnglesOrient = beforeAnglesOrient .. anglesOrient[ 'x' ] .. ', ' .. anglesOrient[ 'y' ] .. ', ' 
  .. anglesOrient[ 'z' ] .. afterAnglesOrient .. commentAnglesOrient .. '\n'
  
  
  local beforeAngleSet = '\t\t{ '
  local afterAngleSet = ' }, '
  local commentAngleSet = '/* isAngleSet */'
  local completedAngleSet = beforeAngleSet .. tostring( isAngleSet[ 'x' ] ) .. ', ' .. tostring( isAngleSet[ 'y' ] ) .. ', ' .. tostring( isAngleSet[ 'z' ] ) .. afterAngleSet .. commentAngleSet .. '\n'
  
  
  local beforeAnglesUsed = '\t\t'
  local afterAnglesUsed = ' '
  local commentAnglesUsed = '/* areAnglesUsed */'
  local completedAnglesUsed = beforeAnglesUsed .. tostring( areAnglesUsed ) .. afterAnglesUsed .. commentAnglesUsed .. '\n'
  
  
  local result = '\n' .. completedId .. completedSizes .. completedCenterPos .. completedMountPos .. completedMountId
  .. completedLastAlign .. completedAxisRot .. completedGroupId .. completedAnglesOrient .. completedAngleSet 
  .. completedAnglesUsed .. '\t);' .. '\n'
  
  return result
end




function fillObstInfo ( obstId, sizes, centerPos, mountPos, orient )
  
  
  local beforeSizesId = '\t' .. 'Obb::Sizes obst' .. obstId
  local beforeSizes = 'Sizes( Obb::Sizes( { '
  local afterSizes = ' } ) );\n'
  
  local beforePosId = '\t' .. 'Eigen::Vector3f obst' .. obstId
  local beforeCenterPos = 'Pos( Eigen::Vector3f( '
  local afterCenterPos = ' ) );\n'
  
  local beforeMountId = '\t' .. 'Eigen::Vector3f obst' .. obstId
  local beforeMountPos = 'Mount( Eigen::Vector3f( '
  local afterMountPos = ' ) );\n'
  
  local beforeOrientId = '\t' .. 'Eigen::Matrix3f obst' ..obstId
  local beforeOrient = 'Orient;' .. '\n\t' .. 'obst' .. obstId .. 'Orient <<\n' 
  
  
  local completedSizes = beforeSizesId .. beforeSizes .. sizes[ 'x' ] .. ', ' .. 
  sizes[ 'y' ] .. ', ' .. sizes[ 'z' ] .. afterSizes
  
  local completedCenterPos = beforePosId .. beforeCenterPos .. centerPos[ 'x' ] .. ', ' .. centerPos[ 'y' ] .. ', ' 
  .. centerPos[ 'z' ] .. afterCenterPos
  
  local completedMountPos = beforeMountId .. beforeMountPos .. mountPos[ 'x' ] .. ', ' .. mountPos[ 'y' ] .. ', '
  .. mountPos[ 'z' ] .. afterMountPos
  
  local completedOrient = beforeOrientId .. beforeOrient ..
  '\t\t' .. orient[ 'x1' ] .. ', ' .. orient[ 'x2' ] .. ', ' .. orient[ 'x3' ] .. ',\n' ..
  '\t\t' .. orient[ 'y1' ] .. ', ' .. orient[ 'y2' ] .. ', ' .. orient[ 'y3' ] .. ',\n' ..
  '\t\t' .. orient[ 'z1' ] .. ', ' .. orient[ 'z2' ] .. ', ' .. orient[ 'z3' ] .. ';\n'
  
  local result = '\n' .. completedSizes .. completedCenterPos .. completedMountPos .. completedOrient .. '\n' 
  
  return result
end


function fillOtherBodyInfo( id, subject, sizes, centerPos, orient )
  
  local result = '\n\n' .. 'env::Obb env::load::getOtherBody' .. id .. '_' .. subject .. '()\n{'

  local beforeSizesId = '\t' .. 'Obb::Sizes '
  local beforeSizes = 'sizes( { '
  local afterSizes = ' } );\n'
  local completedSizes = beforeSizesId .. beforeSizes .. sizes[ 'x' ] .. ', ' .. 
  sizes[ 'y' ] .. ', ' .. sizes[ 'z' ] .. afterSizes
  
  
  local beforePosId = '\t' .. 'Eigen::Vector3f '
  local beforeCenterPos = 'pos( '
  local afterCenterPos = ' );\n'
  local completedCenterPos = beforePosId .. beforeCenterPos .. centerPos[ 'x' ] .. ', ' .. centerPos[ 'y' ] .. ', ' 
  .. centerPos[ 'z' ] .. afterCenterPos
  
  
  local beforeOrientId = '\t' .. 'Eigen::Matrix3f '
  local beforeOrient = 'orient;' .. '\n\t' .. 'orient <<\n' 
  local completedOrient = beforeOrientId .. beforeOrient ..
  '\t\t' .. orient[ 'x1' ] .. ', ' .. orient[ 'x2' ] .. ', ' .. orient[ 'x3' ] .. ',\n' ..
  '\t\t' .. orient[ 'y1' ] .. ', ' .. orient[ 'y2' ] .. ', ' .. orient[ 'y3' ] .. ',\n' ..
  '\t\t' .. orient[ 'z1' ] .. ', ' .. orient[ 'z2' ] .. ', ' .. orient[ 'z3' ] .. ';\n'
  
  
  result = result .. completedSizes .. completedCenterPos .. completedOrient
  
  result = result .. '\n\treturn Obb( sizes, pos, orient );\n}\n\n'
  
  return result
  
end


function funcObstInfoFooter ( amountObst )
  
  local result = '\n' .. '\t' .. 'return ObstInitInfo( {' .. '\n'
  
  for id=0,amountObst-1,1 do 
    
    result = result .. '\t\t' .. 'AttachedDetail( Obb( obst' .. id .. 'Sizes, obst' .. id .. 'Pos, obst' .. id .. 'Orient ),' 
    .. ' obst' .. id .. 'Mount ),' .. '\n'
    
  end 
  
  result = result .. '\t\t' .. 'AttachedDetail( Obb( obst' .. amountObst .. 'Sizes, obst' .. amountObst .. 'Pos, obst' .. amountObst .. 'Orient ),' .. ' obst' .. amountObst .. 'Mount )' .. '\n'
  
  result = result .. '\t' .. '} );' .. '\n' .. '}' .. '\n'
  
  return result
end



function funcManipInfoFooter ( amountManip )
  
  local result = '\n' .. '\t' .. 'return ManipInitInfo{' .. '\n\t\t{ '
  
  for id=0,amountManip-1 do
    
    result = result .. 'link' .. id .. 'Info, '
        
  end 
    
  --[[result = result .. 'link' .. amountManip .. 'Info },' .. '\n\t\t{ ']]--
  
  result = result .. 'link' .. amountManip .. 'Info }' .. '\n'
    
  result = result .. '\t' .. '};' .. '\n' .. '}' .. '\n'
  
  return result
end

