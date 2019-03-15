% This file is created by U. Cem Kaya - Spring 2019
% this function takes the nearest node on a tree and a target location to
% extend a tree branch toward for using just 1 step. It also considers the
% vehicle kinematic constraints while extending. If the extension is
% possible, a new node is added to pathTree.
%% EXTEND FUNCTION FOR TREE EXTENSION FOR JUST 1 STEP
function [AllTrees,extended] = Extend_1(nearest,nearestNodeID,target,AllTrees,ExtTreeNo,AllParam)   
   
% Find which tree to be extend
if ExtTreeNo ==1
    pathTree = AllTrees.ForwardTree;
    Tree = AllTrees.ForwardDiGraph;
else
    pathTree = AllTrees.BackwardTrees{ExtTreeNo-1}; % -1 is the forward Tree
    Tree = AllTrees.BackwardDiGraphs{ExtTreeNo-1};
end

Velocity = AllParam.RRTParam.Vdes;
stepSize = AllParam.RRTParam.stepSize;

% Find diffX and diffY
diffX = abs(nearest(1) - target(1));
diffY = abs(nearest(2) - target(2));

% Find distance between them
dist = Dist(nearest, target);

psiNearest =  pathTree(nearestNodeID).psi; % between
psiTarget = atan2((target(2)-nearest(2)),(target(1)-nearest(1)));

angDiff = psiTarget-psiNearest;

if angDiff < -pi
    angDiff = angDiff + 2*pi;
elseif angDiff > pi
    angDiff = angDiff - 2*pi;
end

delta_t = (dist/Velocity)*AllParam.Constraints.delta_t;
psiDot_Constraint = AllParam.Constraints.psi_dot;
%% 
if ((abs(angDiff)/delta_t)<=psiDot_Constraint)&&(dist <= stepSize)
    
    newPos = target;
    
    TotalLength = pathTree(nearestNodeID).totalDistance + dist;    
    [UtilityAlongPath,NodeUtility,collision,AllTrees] = collCheck(newPos,nearestNodeID,pathTree,AllTrees,AllParam,TotalLength,dist);
     
%%    
elseif ((abs(angDiff)/delta_t)>psiDot_Constraint)&&(dist <= stepSize)
    
    max_turn_angle = delta_t*psiDot_Constraint;
    
    if  (angDiff>0) % add turn angle to close the gap      
        newPos(1) = (nearest(1) + (dist * cos(psiNearest + max_turn_angle)));
        newPos(2) = (nearest(2) + (dist * sin(psiNearest + max_turn_angle)));             
    else % remove turn angle to close the gap   
        newPos(1) = (nearest(1) + (dist * cos(psiNearest - max_turn_angle)));
        newPos(2) = (nearest(2) + (dist * sin(psiNearest - max_turn_angle)));   
    end 
    
    TotalLength = pathTree(nearestNodeID).totalDistance + dist; 
    [UtilityAlongPath,NodeUtility,collision,AllTrees] = collCheck(newPos,nearestNodeID,pathTree,AllTrees,AllParam,TotalLength,dist);
    
%%   
elseif ((abs(angDiff)/delta_t)<=psiDot_Constraint)&&(dist > stepSize)
    % Find sin and cos
    cosA = diffX/dist;
    sinA = diffY/dist;
    
    % Find new position after the step towards target, round them to
    % nearest integer
    if ( ((nearest(1) - target(1)) < 0) && ((nearest(2) - target(2)) < 0) )         
        newPos(1) = (nearest(1) + (stepSize * cosA));
        newPos(2) = (nearest(2) + (stepSize * sinA));       
    elseif ( ((nearest(1) - target(1)) > 0) && ((nearest(2) - target(2)) < 0) )     
        newPos(1) = (nearest(1) - (stepSize * cosA));
        newPos(2) = (nearest(2) + (stepSize * sinA));       
    elseif ( ((nearest(1) - target(1)) < 0) && ((nearest(2) - target(2)) > 0) )      
        newPos(1) = (nearest(1) + (stepSize * cosA));
        newPos(2) = (nearest(2) - (stepSize * sinA));
    else      
        newPos(1) = (nearest(1) - (stepSize * cosA));
        newPos(2) = (nearest(2) - (stepSize * sinA));
    end
    dist = Dist(nearest,newPos);
    TotalLength = pathTree(nearestNodeID).totalDistance + dist;
    [UtilityAlongPath,NodeUtility,collision,AllTrees] = collCheck(newPos,nearestNodeID,pathTree,AllTrees,AllParam,TotalLength,dist);
    
%% 
else
    
    max_turn_angle = delta_t*psiDot_Constraint;
    
    if  (angDiff>0) % add turn angle to close the gap     
        newPos(1) = (nearest(1) + (stepSize * cos(psiNearest + max_turn_angle)));
        newPos(2) = (nearest(2) + (stepSize * sin(psiNearest + max_turn_angle)));   
    else % remove turn angle to close the gap    
        newPos(1) = (nearest(1) + (stepSize * cos(psiNearest - max_turn_angle)));
        newPos(2) = (nearest(2) + (stepSize * sin(psiNearest - max_turn_angle)));     
    end
    
    dist = Dist(nearest,newPos);
    TotalLength = pathTree(nearestNodeID).totalDistance + dist;
    [UtilityAlongPath,NodeUtility,collision,AllTrees] = collCheck(newPos,nearestNodeID,pathTree,AllTrees,AllParam,TotalLength,dist);
    
end

%% If there is no Collision, then add the node into tree and Update Trees
if (collision==true)
    extended = 0;
else   
    extended = newPos;   
    [pathTree,Tree] = AddNode(pathTree,Tree, newPos, nearestNodeID,UtilityAlongPath,NodeUtility,dist,Velocity);
        % Update Trees
    if ExtTreeNo ==1
        AllTrees.ForwardTree = pathTree;
        AllTrees.ForwardDiGraph = Tree;
    else
        AllTrees.BackwardTrees{ExtTreeNo-1} = pathTree; % -1 is the forward Tree
        AllTrees.BackwardDiGraphs{ExtTreeNo-1} = Tree;
    end  
end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ADD NODE FUNCTION
function [pathTree,Tree] = AddNode(pathTree,Tree, extended, parentNodeID,PathUtility,NodeUtility,delta_dist,Velocity)    
    % Get the length of the tree
    tlength = length(pathTree);
    % Add new element to the tree
    Tree = addnode(Tree,1);
        
    pathTree(tlength+1).x = extended(1);
    pathTree(tlength+1).y = extended(2);
    pathTree(tlength+1).parent = parentNodeID;
    pathTree(parentNodeID).children = [pathTree(parentNodeID).children tlength+1];
    pathTree(tlength+1).PathUtility = PathUtility;
    pathTree(tlength+1).NodeUtility = NodeUtility;
    pathTree(tlength+1).ID = tlength+1;
    Tree = addedge(Tree,parentNodeID,tlength+1);
    pathTree(tlength+1).totalDistance = pathTree(parentNodeID).totalDistance + delta_dist;
    
    pathTree(tlength+1).psi = heading(pathTree(parentNodeID),pathTree(tlength+1));
    pathTree(tlength+1).vel = Velocity;
    pathTree(tlength+1).time = pathTree(parentNodeID).time + delta_dist/Velocity;
    
    pathTree(tlength+1).TreeNo = pathTree(tlength).TreeNo;
    pathTree(tlength+1).Type = pathTree(tlength).Type;
end

%% FUNCTION TO CALCULATE HEADING
function theta = heading(position1,position2)
theta = atan2((position2.y-position1.y),(position2.x-position1.x));
end 