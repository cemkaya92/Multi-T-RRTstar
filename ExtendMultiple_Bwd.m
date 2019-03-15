% This file is created by U. Cem Kaya - Spring 2019
% This function tries to extend Backward tree branches multiple steps one
% at a time considering maximum step size. Currently, used only during the
% rewiring .
%% MULTIPLE EXTEND FUNCTION BACKWARD
function [AllTrees,PathUtilityAlongNearNode,NodeUtility,fail] ...
    = ExtendMultiple_Bwd(nearest,target,nearestNodeID,BackwardTreeID,AllTrees,AllParam,dist)   
             
fail = false;
stepSize = AllParam.Constraints.stepSize;
steps = ceil(dist/stepSize);
ExtTreeNo = BackwardTreeID + 1;

for k = 1:steps
    
    [AllTrees,extended] = Extend_1(nearest,nearestNodeID,target,AllTrees,ExtTreeNo,AllParam) ;  
   
    if (extended ~= 0)
        
        extendedNode = AllTrees.BackwardTrees{BackwardTreeID}(end);
        extendedNode.ID = length(AllTrees.BackwardTrees{BackwardTreeID});
        
        dist = Dist(nearest,extended);
        
        [PathUtilityAlongNearNode,NodeUtility,~,~] = UtilitySegment_Backward([extendedNode.x extendedNode.y],nearestNodeID,AllTrees,BackwardTreeID,AllParam,dist);
        
        nearest = [extendedNode.x extendedNode.y];
        nearestNodeID = extendedNode.ID;
        
    else
        fail = true;
        PathUtilityAlongNearNode = 0;
        NodeUtility = 0;
        break;
    end
    
end


end


