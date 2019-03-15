% This file is created by U. Cem Kaya - Spring 2019
%% Function to Check whether the previously found paths have changed the shape or the utility due to rewiring
function [AllTrees,AllParam] = CheckPrevPaths(AllTrees,AllParam)

lambda = AllParam.PURM.lambda; % total failure rate
No_Paths = length(AllTrees.NodeIDsuccesses(:,1)); % number of potential paths

for jj = 2:No_Paths % remove the Do-Not-Fly path (very first path)
    
    % look for the notation of NodeIDsuccess creation
    NodeID_Forw = AllTrees.NodeIDsuccesses(jj,2);  % Node ID on FT connecting to BTree
    NodeID_Back = AllTrees.NodeIDsuccesses(jj,4);  % Node ID on BT connecting to FTree
    BackTreeID = AllTrees.NodeIDsuccesses(jj,3); 
    % calculate the path utility from the connection point
    totalPathUtility = (AllTrees.ForwardTree(NodeID_Forw).PathUtility...
        + exp(-lambda*AllTrees.ForwardTree(NodeID_Forw).time)*AllTrees.BackwardTrees{BackTreeID}(NodeID_Back).PathUtility);
    
    totalDistance = AllTrees.ForwardTree(NodeID_Forw).totalDistance + AllTrees.BackwardTrees{BackTreeID}(NodeID_Back).totalDistance;
    totalTime = AllTrees.ForwardTree(NodeID_Forw).time + AllTrees.BackwardTrees{BackTreeID}(NodeID_Back).time; 
    
    % if rewiring changes trees then update their values
    if (totalPathUtility~=AllTrees.NodeIDsuccesses(jj,7))...
            ||(totalDistance~=AllTrees.NodeIDsuccesses(jj,6))...
            ||(totalTime~=AllTrees.NodeIDsuccesses(jj,5))
        
        AllTrees.NodeIDsuccesses(jj,5) = totalTime;
        AllTrees.NodeIDsuccesses(jj,6) = totalDistance;
        AllTrees.NodeIDsuccesses(jj,7) = totalPathUtility;
   % ?f it is better than the best one so far         
        if totalPathUtility > 1.0*AllParam.UtilityThreshold.Cumulative % min cumulative utility
            
            [Path, PathUtilityLevel,~,MinNodeUtility] = bidirectionalBacktrack(NodeID_Forw,NodeID_Back,...
                BackTreeID,AllTrees,AllParam);
            
            if AllParam.RRTParam.RecordVideo            
%                 figure(AllParam.figHandle.map);
                set(AllParam.figHandle.FinalPath(end),'visible','off')
                AllParam.figHandle.FinalPath(end+1) =  plot([Path(:).x]',[Path(:).y]','g-','LineWidth', 2);
                drawnow;
            end
            
            % Update MultiplePaths
            AllTrees.NodeIDsuccesses(jj,5:7) = [Path(end).time, Path(end).totalDistance, Path(end).PathUtility];
            
            AllParam.UtilityThreshold.Cumulative = 1.01*PathUtilityLevel;
            AllParam.UtilityThreshold.ExposureRate = 1.01*MinNodeUtility;            
        end
    end
    
    
end


end