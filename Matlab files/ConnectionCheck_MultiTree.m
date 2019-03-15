% This file is created by U. Cem Kaya - Spring 2019
% this function checks the possible connection scenarios between forward
% and backward trees. 
%% CONNECTION CHECK FUNCTION BETWEEN TREES FOR YAW CONSTRAINTS
function [success,AllTrees] = ConnectionCheck_MultiTree(ScenarioNo,...
        extended_forw,extended_backs,AllTrees,AllParam)

No_Trees = AllParam.RRTParam.No_Trees;
success = 0; 
distThresh = AllParam.RRTParam.distThresh;

Connection = 0; % Possible Connection Flag 

for k = 1:length(ScenarioNo)
    
    switch ScenarioNo(k)
        
        case 1
            
            %% First Scenario - Forward Tree Connects to Goal Locations
            if extended_forw ~= 0 % if extention of node is successful
                for jj = 2:No_Trees % first tree is forward,
                    if Dist(extended_forw,AllParam.RRTParam.finish(jj-1,:)) < distThresh
                        Connection = 1;
                        NodeID_Forw = length(AllTrees.ForwardTree); % last extended node can connect to goal in 1st scenario
                        NodeID_Back = 1; % goal locations are the roots of the backtrees
                        BackTreeID = jj-1;
                    end
                end
            end
            
        case 2
            
            %% Second Scenario - Backward Trees Connect to Start Location
            for jj = 2:No_Trees
                if extended_backs(jj-1,:) ~= 0 % if extention of node is successful
                    if Dist(extended_backs(jj-1,:),AllParam.RRTParam.start) < distThresh               
                        Connection = 1;
                        NodeID_Forw = 1; % start is the root of the Forward Tree
                        NodeID_Back = length(AllTrees.BackwardTrees{jj-1}); % last extended node can connect to start
                        BackTreeID = jj-1;
                    end
                end
            end
            
        case 3
            
            %% Third Scenario - Forward Tree Connects to one of the Backward Trees
            if extended_forw ~= 0 % if extention of node is successful
                for jj = 2:No_Trees
                    if extended_backs(jj-1,:) ~= 0 % if extention of node is successful
                        [nearestToExt_Bkwd, nearestToExtID_Bkwd] = NearestNode(AllTrees.BackwardTrees{jj-1}, extended_forw, AllTrees.BackKDtrees{jj-1});
                        if Dist(nearestToExt_Bkwd, extended_backs(jj-1,:)) < distThresh
                            Connection = 1;
                            NodeID_Forw = length(AllTrees.ForwardTree); % last extended node can connect to
                            NodeID_Back = nearestToExtID_Bkwd;
                            BackTreeID = jj-1;
                        end
                    end
                end
            end
            
        case 4
            
            %% Fourth Scenario - One of the Backward Trees connects to Forward Tree
            for jj = 2:No_Trees
                if extended_backs(jj-1,:) ~= 0 % if extention of node is successful
                    [nearestToExt_Fwd, nearestToExtID_Fwd] = NearestNode(AllTrees.ForwardTree, extended_backs(jj-1,:),AllTrees.ForwardKDtree);
                    if Dist(nearestToExt_Fwd, extended_backs(jj-1,:)) < distThresh
                        Connection = 1;
                        NodeID_Forw = nearestToExtID_Fwd; % nearest node on the Forward Tree
                        NodeID_Back = length(AllTrees.BackwardTrees{jj-1});
                        BackTreeID = jj-1;
                    end
                end
            end
            
    end
    
    %% After Checking Possible Connections, If there is a possible connection, Look at the Angle Constraints
    if Connection == 1
               
        angleTest = AngleCheck(NodeID_Forw,NodeID_Back,BackTreeID,AllTrees,AllParam);
        % If the condition passes the angleTest then, attempt to connect Forward Tree to Connection Node
        if angleTest == 1
            [AllTrees,success] = Extend_to_Connect(NodeID_Forw,NodeID_Back,BackTreeID,AllTrees,AllParam);
        end
        
    end
    
end

end

%% If both trees are connectable, extend a branch from the first tree to connect
function [AllTrees,success] = Extend_to_Connect(nearestToExtID_fwd,extendToID_bkwd,BackTreeID,AllTrees,AllParam)
     
    nearest = [AllTrees.ForwardTree(nearestToExtID_fwd).x   AllTrees.ForwardTree(nearestToExtID_fwd).y];
    target = [AllTrees.BackwardTrees{BackTreeID}(extendToID_bkwd).x   AllTrees.BackwardTrees{BackTreeID}(extendToID_bkwd).y];
    
    nearestNodeID = nearestToExtID_fwd;
    ExtTreeNo = 1;
    %% 
    [AllTrees,extended_forw] = Extend_1(nearest,nearestNodeID,target,AllTrees,ExtTreeNo,AllParam) ;
    if extended_forw == 0
        success = 0;        % connection failed
    else
        % Look at MainFunc for the notation of NodeIDsuccess
        AllTrees.NodeIDsuccesses(end+1,1:4) = [1,length(AllTrees.ForwardTree), AllTrees.BackwardTrees{BackTreeID}(extendToID_bkwd).TreeNo, extendToID_bkwd]; 
        % Notation: [ID of ConnectedTree on ForwardTree = 1, IDofNode on
        % FT, ID ConnectedTree on Backward Trees = j, Id of Node on BT, total elapsed time, total distance, total Path utility]
        success = 1;
    end
    
end

%% CHECK ANGLES FOR THE YAW RATE
function angleTest = AngleCheck(id1,id2,BackTreeID,AllTrees,AllParam)

ForwardNode = AllTrees.ForwardTree(id1);
BackwardNode = AllTrees.BackwardTrees{BackTreeID}(id2);

psi1 = ForwardNode.psi;
psi2 = BackwardNode.psi; 

dist = Dist([ForwardNode.x ForwardNode.y], [BackwardNode.x BackwardNode.y]);

delta_t_max = (dist/ForwardNode.vel)*AllParam.Constraints.delta_t;
psiDot_Constraint = AllParam.Constraints.psi_dot;

if psi2 < 0 % reverse the second tree connection angle
    psi2 = psi2 + pi;
else
    psi2 = psi2 - pi;
end

angDiff = psi2-psi1;
    
if angDiff < -pi
    angDiff = angDiff + 2*pi;
elseif angDiff > pi
    angDiff = angDiff - 2*pi;
end

if ((abs(angDiff)/delta_t_max)<=psiDot_Constraint)
    angleTest = true;
else
    angleTest = false;
end

end