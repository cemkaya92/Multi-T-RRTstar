% This file is created by U. Cem Kaya - Spring 2019
% This file requires the connection node IDs from the connected trees with
% the other parameters to backtrack both Tree to find a path from start
% location to goal location of the connected backward tree.
%% FUNCTION TO BIDIRECTIONAL BACKTRACK
function [Path, PathUtilityLevel,total_distance,MinNodeUtility]...
    = bidirectionalBacktrack(NodeID1,NodeID2,BackTreeID,AllTrees,AllParam)
                  
lambda = AllParam.PURM.lambda; % total failure rate assuming Poisson Process Failures
currNode_1 = AllTrees.ForwardTree(NodeID1); % connection node on the Forward tree

idx1 = 1;
%% Backtrack the Forward Tree until the Start Location
while (1)
    Path(idx1) = currNode_1; % Note: since the total path number is not known
                             % until you backtrack, not possible to
                             % allocate matrix before hand. Alternative:
                             % total number of steps to reach a current
                             % node from the root can be added to each node
    % find parent of current node
    parentID_1 = currNode_1.parent;
    % X Y location of the Current node
    coord_X_1 = currNode_1.x;
    coord_Y_1 = currNode_1.y;
    % Change current node to its parent
    currNode_1 = AllTrees.ForwardTree(parentID_1);
       
    % If startpoint was reached, break out of the loop
    if ( (coord_X_1 == AllParam.RRTParam.start(1)) && (coord_Y_1 == AllParam.RRTParam.start(2)) )
        break;
    end
    
    idx1 = idx1 + 1;   
end
% reverse the order to get correct ordering
Path = fliplr(Path); 

%% Now Backtrack the Backward Tree
currNode_2_first = AllTrees.BackwardTrees{BackTreeID}(NodeID2); % connected node on tree2
parentID_2 = currNode_2_first.parent; % since tree1 has already a node on top of this node, we will start counting from the parent
% Change current node to its parent
currNode_2 = AllTrees.BackwardTrees{BackTreeID}(parentID_2);

% If the connection occurs at the root
if (parentID_2 == 1 )
    t_forward = Path(end).time;
    Path(end+1) = AllTrees.BackwardTrees{BackTreeID}(1);
    Path(end).PathUtility = Path(end-1).PathUtility;
    Path(end).totalDistance = Path(end-1).totalDistance;
    Path(end).time = Path(end-1).time;
else

    idx2 = idx1+1;
    
    previousNode_2 = currNode_2_first;
    
    delta_t = previousNode_2.time - currNode_2.time; % previous node is the children of the current node in backward tree
    t_forward = Path(idx2-1).time; % forward time begining at which forward tree connects the backward tree
    % Backtrack the backward tree
    while (1)
        Path(idx2) = currNode_2;
        % find parent of current node
        parentID_2 = currNode_2.parent;        
        Path(idx2).psi = heading(previousNode_2,currNode_2);              
        % Add the cost of pathTree2 nodes
        %% Look at the calculation of the nodes on the path for actual costs, distances and time
        % Be careful with the exponential terms. Spesific to Poisson
        Path(idx2).PathUtility = Path(idx2-1).PathUtility + exp(-lambda*t_forward)*(previousNode_2.PathUtility - exp(-lambda*delta_t)*currNode_2.PathUtility);
        Path(idx2).totalDistance = Path(idx2-1).totalDistance + previousNode_2.totalDistance - currNode_2.totalDistance;
        Path(idx2).time = Path(idx2-1).time + delta_t;
        % location of the node
        coord_X_2 = currNode_2.x;
        coord_Y_2 = currNode_2.y;
        % Change previous node to its parent
        previousNode_2 = currNode_2;
        % Change current node to its parent
        currNode_2 = AllTrees.BackwardTrees{BackTreeID}(parentID_2);
        
        % Update delta_t and t_forward
        delta_t = previousNode_2.time - currNode_2.time;
        t_forward = Path(idx2).time; % forward time at the begining of next node in backward tree
        
        % If startpoint was reached, break out of the loop
        if ( (coord_X_2 == AllParam.RRTParam.finish(BackTreeID,1)) && (coord_Y_2 == AllParam.RRTParam.finish(BackTreeID,2)) )
            break;
        end
        
        idx2 = idx2 + 1;       
    end
    
end
% Add backward tree initial utility
Path(end).PathUtility = Path(end).PathUtility + exp(-lambda*t_forward)*currNode_2.PathUtility;

PathUtilityLevel = Path(end).PathUtility;
total_distance = Path(end).totalDistance;
MinNodeUtility = min([Path(:).NodeUtility]);

end

%% FUNCTION TO CALCULATE HEADING
function theta = heading(position1,position2)
theta = atan2((position2.y-position1.y),(position2.x-position1.x));
end 