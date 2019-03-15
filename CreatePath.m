% This file is created by U. Cem Kaya - Spring 2019
% If the connection between trees is successful, check whether this
% connection results in a better path (higher utility). If it has higher
% utility then previously found paths, then use backtracking to find and
% save the path as the new one. Also update the parameters.
%% FUNCTION TO CREATE PATH
function [MultiplePaths,AllTrees,AllParam] = CreatePath(AllTrees,AllParam,MultiplePaths,iter_N)

    lambda = AllParam.PURM.lambda;
    % look for the notation of NodeIDsuccess creation
    NodeID_Forw = AllTrees.NodeIDsuccesses(end,2);  % Node ID on FT connecting to BTree
    NodeID_Back = AllTrees.NodeIDsuccesses(end,4);  % Node ID on BT connecting to FTree
    BackTreeID = AllTrees.NodeIDsuccesses(end,3);    
    
    % Assign total Elapsed time, total Distance Traveled and Total Path Utility over the Path
    AllTrees.NodeIDsuccesses(end,5) = AllTrees.ForwardTree(NodeID_Forw).time...
                                    + AllTrees.BackwardTrees{BackTreeID}(NodeID_Back).time;
                                
    AllTrees.NodeIDsuccesses(end,6) = AllTrees.ForwardTree(NodeID_Forw).totalDistance...
                                    + AllTrees.BackwardTrees{BackTreeID}(NodeID_Back).totalDistance;
    
    totalPathUtility = (AllTrees.ForwardTree(NodeID_Forw).PathUtility ...
                     + exp(-lambda*AllTrees.ForwardTree(NodeID_Forw).time)*AllTrees.BackwardTrees{BackTreeID}(NodeID_Back).PathUtility);
    AllTrees.NodeIDsuccesses(end,7) = totalPathUtility;
    
    if totalPathUtility > 1.0*AllParam.UtilityThreshold.Cumulative % minimum cumulative utility limit
        
        [Path, PathUtilityLevel,~,MinNodeUtility] = bidirectionalBacktrack(NodeID_Forw,NodeID_Back,...
            BackTreeID,AllTrees,AllParam);
                 
            if AllParam.RRTParam.RecordVideo
%                 figure(AllParam.figHandle.map)
                if (length(AllParam.figHandle.FinalPath)>=1)
                    set(AllParam.figHandle.FinalPath(end),'visible','off') 
                end         
                title(['Iter = ' num2str(iter_N)])
                AllParam.figHandle.FinalPath(end+1) =  plot([Path(:).x]',[Path(:).y]','g-','LineWidth', 1.5);
                drawnow;
                
                frm = getframe;
                img = frame2im(frm);
                writeVideo(AllParam.RRTParam.writerObj, img);
                
            end
                    
%%            
            if  PathUtilityLevel > AllParam.UtilityThreshold.Cumulative
                
                MultiplePaths(end+1,:) = [AllTrees.NodeIDsuccesses(end,:)];
                % Update Utility Thresholds
                AllParam.UtilityThreshold.Cumulative = 1.01*PathUtilityLevel;         
                AllParam.UtilityThreshold.ExposureRate = 1.01*MinNodeUtility;
                
            end
    
    end
        
  
end