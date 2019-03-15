% This file is created by U. Cem Kaya - Spring 2019
% This function tries to extend Forward tree branches multiple steps one
% at a time considering maximum step size. Currently, used only during the
% rewiring .
%% MULTIPLE EXTEND FUNCTION FORWARD
function [AllTrees,PathUtilityAlongNearNode,NodeUtility,fail]...
    = ExtendMultiple_Fwd(nearest, target,nearestNodeID,AllTrees,AllParam,dist)   

fail = false;
stepSize = AllParam.Constraints.stepSize;
steps = ceil(dist/stepSize);
ExtTreeNo = 1; % Forward Tree is extended

for k = 1:steps
    
    [AllTrees,extended] = Extend_1(nearest,nearestNodeID,target,AllTrees,ExtTreeNo,AllParam) ;  
   
    if (extended ~= 0)
        
        extendedNode = AllTrees.ForwardTree(end);
        extendedNode.ID = length(AllTrees.ForwardTree);
        
        dist = Dist(nearest,extended);
        
        [PathUtilityAlongNearNode,NodeUtility,~,~] = UtilitySegment_Forward([extendedNode.x extendedNode.y],nearestNodeID,AllTrees,AllParam,dist);
        
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


