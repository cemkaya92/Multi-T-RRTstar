% This file is created by U. Cem Kaya - Spring 2019
% This function checks collision between extended tree branch and
% poliygonal obstacles. Polygonal obstacles have encapsulating outer circles
% around them. First sanity check uses this circle, then, if it is inside
% or intersecting, inpolygon check is applied.
%% COLLISION CHECK FUNCTION
function [UtilityAlongPath,NodeUtility,collision,AllTrees]...
    = collCheck(newPosition,nearestNodeID,pathTree,AllTrees,AllParam,TotalLength,dist)

ResMapSize = AllParam.GridMapParam.ResMapSize;

LengthThreshold = AllParam.UtilityThreshold.TotalLength;
% CumulativeRiskThreshold = AllParam.UtilityThreshold.Cumulative;
% ExposureRateThreshold = AllParam.UtilityThreshold.ExposureRate;

p1 = [pathTree(nearestNodeID).x; pathTree(nearestNodeID).y]; 
p2 = [newPosition(1); newPosition(2)];
% r = sqrt(sum((p2-p1).^2))/2; % radius of the circle enclosing the straight line
r = dist; % dist comes from a previous calculation
center_of_circle = (p1+p2)/2;
x_center = center_of_circle(1);
y_center = center_of_circle(2);

PolygonCheck = false; % initialize

if(~isempty(AllParam.obstacles))

    OuterCircleCheck = ((sum(([x_center;y_center]-[AllParam.obstacles.center]).^2))<([AllParam.obstacles.outerRadius]+r/2).^2);
    
    [~,ObsID] = find(OuterCircleCheck == true);
%%% NOTE: you can use logical vectors instead of using find to make it faster 
%%% however, simulations does not have obstacles yet
    if (~isempty(ObsID))
        
        PolygonCheck = true;
        [~,InsideObstCircleID] = find((([AllParam.obstacles.outerRadius]-r).^2>(sum(([x_center;y_center]-[AllParam.obstacles.center]).^2))) == true);
        IntersectingCirclesID = setdiff(ObsID,InsideObstCircleID);
        
        for j = 1:length(InsideObstCircleID)
            PolygonCheck = inpolygon(newPosition(1),newPosition(2),AllParam.obstacles(InsideObstCircleID(j)).pos(1,:)',AllParam.obstacles(InsideObstCircleID(j)).pos(2,:)');
            if PolygonCheck == true
                break;
            end
        end
        if PolygonCheck == false
            for j = 1:length(IntersectingCirclesID)
                
                PolygonCheck = inpolygon(newPosition(1),newPosition(2),AllParam.obstacles(IntersectingCirclesID(j)).pos(1,:)',AllParam.obstacles(IntersectingCirclesID(j)).pos(2,:)');
                if PolygonCheck == true
                    break;
                end
                
            end
        end
    end
 
end

if ((TotalLength>LengthThreshold)||PolygonCheck||(newPosition(1)<ResMapSize(1,1))||(newPosition(1)>ResMapSize(2,1))...
        ||(newPosition(2)<ResMapSize(1,2))||(newPosition(2)>ResMapSize(2,2)))

    collision = true;
    UtilityAlongPath = NaN;
    NodeUtility = NaN;
else
    
    if pathTree(nearestNodeID).Type == 1
        [UtilityAlongPath,NodeUtility,tr_test,AllTrees] = UtilitySegment_Forward(newPosition,...
        nearestNodeID,AllTrees,AllParam,dist);

    elseif pathTree(nearestNodeID).Type == 2
        BackwardTreeID = pathTree(nearestNodeID).TreeNo;
        [UtilityAlongPath,NodeUtility,tr_test,AllTrees] = UtilitySegment_Backward(newPosition,...
        nearestNodeID,AllTrees,BackwardTreeID,AllParam,dist);
    
    end
    
    % Check the Transition Test result as well, it is not a collision but
    % can be used to reject tree extensions.
    if (tr_test == 0)
        collision = true;
    else
        collision = false;   
    end
   
end

end