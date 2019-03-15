% This file is created by U. Cem Kaya - Spring 2019
% This function calculates the utility of the nodes and the cumulative
% utility of extending a branch using Vehicle Impact parameters and PREM
% risk mapping. First, constructs the elliptical regions for the impact zones
% and assign Truncated Gaussian Distributions within these zones. Then,
% locates the corresponding PREM values on a grid map to multiply and
% integrate the Utility Function. At the end, applies transition test. 
%% FUNCTION TO CALCULATE THE UTILITY OF EXTENDING A BACKWARD TREE BRANCH
function [UtilityAlongPath,NodeUtility1,tr_test,AllTrees] = UtilitySegment_Backward(extended,...
    PathNodeID,AllTrees,BackwardTreeID,AllParam,dist)
% Grid Resolution parameters
dX = AllParam.GridMapParam.dX; dY = AllParam.GridMapParam.dY;
X = AllParam.GridMapParam.X_mesh; Y = AllParam.GridMapParam.Y_mesh; % mesh matrix of gridmap
% Extended Location
x1 = extended(1);  y1 = extended(2);
% Node on the Tree
x2 = AllTrees.BackwardTrees{BackwardTreeID}(PathNodeID).x;  
y2 = AllTrees.BackwardTrees{BackwardTreeID}(PathNodeID).y;
length_segment = dist;

NodeUtility0 = AllTrees.BackwardTrees{BackwardTreeID}(PathNodeID).NodeUtility;
% Vehicle Failure Parameters
Angles = AllParam.PURM.theta;
ReachableRadius = AllParam.PURM.reachableRadius;
failureRate = AllParam.PURM.failureRate;
lambda = AllParam.PURM.lambda;
dArea = dX*dY;

% find the heading from tree node to extended location
phi = atan2((y1-y2),(x1-x2))  ;
% Elliptical Region Dimensions for each failure
a=ReachableRadius(:,1);
b=ReachableRadius(:,2);

%% create small mesh to construct impact distribution enclosing all the failure zones
x_small = x2-max(a):dX:x2+max(a);
y_small = y2-max(a):dY:y2+max(a);
[X_small,Y_small] = meshgrid(x_small,y_small);
%%
sizeSmall = size(X_small);
ImpactDist_small = zeros(sizeSmall(1),sizeSmall(2));

 for j = 1:length(a)
     % Rotate the ellipse by its offset angle
     Theta = phi+Angles(j);
     %% Calculate Gaussian Parameters
     sigma_x = a(j)/3; sigma_y = b(j)/3;
     
     a2 = ((cos(-Theta)^2)/(2*sigma_x^2))+((sin(-Theta)^2)/(2*sigma_y^2));
     b2 = (-(sin(2*(-Theta)))/(4*sigma_x^2))+((sin(2*(-Theta)))/(4*sigma_y^2));
     c2 = ((sin(-Theta)^2)/(2*sigma_x^2))+((cos(-Theta)^2)/(2*sigma_y^2));
     
     %Define a rotation matrix
     R1 = [ cos(Theta) -sin(Theta); sin(Theta) cos(Theta) ]; % angle is negative because prem and purm is flipped upside down.
     % upper side of the map corresponds to bottom of the matrix
     
     mu = [x2;y2]+R1*[0.4*a(j);0]; % I have shifted the center of elliptical regions by 40 percent of the a-axis such that quadcopter is not on the origin, but in the perigee point
     
     [~,x_mu] = min((X(1,:)-mu(1)).^2);
     [~,y_mu] = min((Y(:,1)-mu(2)).^2);
     
     A = 1; % below F formula is an alternative way to construct multivariate normal distribution
     F = A*exp(-(a2*(X_small - mu(1)).^2 + 2*b2*(X_small - mu(1)).*(Y_small - mu(2)) + c2*(Y_small - mu(2)).^2));
     
     edge = A*exp(-(a2*(a(j)*cos(Theta)).^2)); % define the edge of the Gaussian like 3sigma deviation
     idx =  F > edge; % Truncated Gaussian Distribution is obtained
     
     % F = F(idx)/sum(sum(F(idx))); normalized to make the sum 1
     ImpactDist_small(idx) = ImpactDist_small(idx) + (failureRate(j)/lambda)*F(idx)/sum(sum(F(idx)));
     
 end

ImpactDist_small = sparse(ImpactDist_small);
 
[length_Y,length_X] = size(X);
ImpactDist = sparse(length_Y,length_X);
 
[rowDim,colmDim] = size(ImpactDist_small);
half_x = round(colmDim/2);
half_y = round(rowDim/2);
%% Locate the Center of Impact Matrix on a Large PREM Matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if (x_mu > half_x ) && (x_mu + half_x <= length_X) && (y_mu > half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x = upperLeft_x + colmDim-1;
    bottomRight_y = upperLeft_y + rowDim-1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small;

elseif (x_mu <= half_x ) && (x_mu + half_x <= length_X) && (y_mu > half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = 1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x =  upperLeft_x + colmDim + x_mu - half_x-1;
    bottomRight_y =  upperLeft_y + rowDim -1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small(:,(half_x-x_mu+1):end);

elseif (x_mu > half_x ) && (x_mu + half_x > length_X) && (y_mu > half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x = length_X;
    bottomRight_y = upperLeft_y + rowDim-1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small(:,1:(colmDim-((colmDim-half_x)-(length_X - x_mu))));

elseif (x_mu > half_x ) && (x_mu + half_x <= length_X) && (y_mu <= half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = 1;
    bottomRight_x =  upperLeft_x + colmDim -1;
    bottomRight_y =  upperLeft_y + rowDim +y_mu - half_y -1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small((half_y-y_mu+1):end,:);

elseif (x_mu > half_x ) && (x_mu + half_x <= length_X) && (y_mu > half_y ) && (y_mu + half_y > length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x = upperLeft_x + colmDim-1;
    bottomRight_y = length_Y;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small(1:(rowDim-((rowDim-half_y)-(length_Y - y_mu))),:);
    
elseif (x_mu <= half_x ) && (x_mu + half_x <= length_X) && (y_mu <= half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = 1;
    upperLeft_y = 1;
    bottomRight_x =  upperLeft_x + colmDim + x_mu - half_x-1;
    bottomRight_y =  upperLeft_y + rowDim + y_mu - half_y -1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small((half_y-y_mu+1):end,(half_x-x_mu+1):end);


elseif (x_mu > half_x ) && (x_mu + half_x > length_X) && (y_mu > half_y ) && (y_mu + half_y > length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x = length_X;
    bottomRight_y = length_Y;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small(1:(rowDim-((rowDim-half_y)-(length_Y - y_mu))),1:(colmDim-((colmDim-half_x)-(length_X - x_mu))));

elseif (x_mu > half_x ) && (x_mu + half_x > length_X) && (y_mu <= half_y ) && (y_mu + half_y <= length_Y)
    upperLeft_x = x_mu - half_x+1;
    upperLeft_y = 1;
    bottomRight_x =  length_X;
    bottomRight_y =  upperLeft_y + rowDim +y_mu - half_y -1;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small((half_y-y_mu+1):end,1:(colmDim-((colmDim-half_x)-(length_X - x_mu))));

elseif (x_mu <= half_x ) && (x_mu + half_x <= length_X) && (y_mu > half_y ) && (y_mu + half_y > length_Y)
    upperLeft_x = 1;
    upperLeft_y = y_mu - half_y+1;
    bottomRight_x =  upperLeft_x + colmDim +x_mu - half_x -1;
    bottomRight_y =  length_Y;
    ImpactDist(upperLeft_y:bottomRight_y,upperLeft_x:bottomRight_x) = ImpactDist_small(1:(rowDim-((rowDim-half_y)-(length_Y - y_mu))),(half_x-x_mu+1):end);

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Perform the Path Utility Integration
idx2 = ImpactDist ~= 0;
NodeUtility1 =  -sum(ImpactDist(idx2).*AllParam.PREM.layer1(idx2)*dArea);
     
t0 = AllTrees.BackwardTrees{BackwardTreeID}(PathNodeID).time;
t1 = AllTrees.BackwardTrees{BackwardTreeID}(PathNodeID).time ...
    + (length_segment/AllTrees.BackwardTrees{BackwardTreeID}(PathNodeID).vel);
delta_t = (t1-t0);
%% Calculation of REL is different in backward tree
% calculation proceeds from children to parent and include the exponential delta_t term (instead of parent to children)
% Look at the Cumulative Risk Calculation for Backward Tree
%CumRisk(i+1) = 0.5*(R(i+1)+exp(-lambda*delta_t)*R(i))*delta_t + exp(-lambda*delta_t)*CumRisk(i);
REL = lambda*0.5*(NodeUtility1 + (exp(-lambda*delta_t))*NodeUtility0)*delta_t;

UtilityAlongPath = REL + exp(-lambda*delta_t)*AllTrees.BackwardTrees{BackwardTreeID}(PathNodeID).PathUtility;

%% Apply Transition Test (Rejection Sampling)
[tr_test,AllTrees.T_Backwards{BackwardTreeID}] = transitionTest(NodeUtility0, NodeUtility1, AllTrees.T_Backwards{BackwardTreeID}); % I can add the negative sign so that I can use Transition test as normal if it is defined in cost space not in utility space.

%% Update Transition Parameters
% update node min max temperatures
if NodeUtility1 < AllTrees.T_Backwards{BackwardTreeID}.min
    AllTrees.T_Backwards{BackwardTreeID}.min = NodeUtility1;
    AllTrees.T_Backwards{BackwardTreeID}.range ...
        = AllTrees.T_Backwards{BackwardTreeID}.max ...
        - AllTrees.T_Backwards{BackwardTreeID}.min;
    
elseif NodeUtility1 > AllTrees.T_Backwards{BackwardTreeID}.max
    AllTrees.T_Backwards{BackwardTreeID}.max = NodeUtility1;
    AllTrees.T_Backwards{BackwardTreeID}.range...
        = AllTrees.T_Backwards{BackwardTreeID}.max...
        - AllTrees.T_Backwards{BackwardTreeID}.min;
end

end
