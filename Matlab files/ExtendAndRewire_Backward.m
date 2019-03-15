% This file is created by U. Cem Kaya - Spring 2019
% This function performs the extend and rewire operations for the each
% backward tree in a loop. First selects a target, Second tries to extend a
% branch towards to target, if successful, then, tries to perform rewiring
% among the near nodes.
%% EXTEND AND REWIRE OPERATIONS FOR THE BACKWARD TREES
function [extended_backs,AllTrees] = ExtendAndRewire_Backward(target,AllTrees,AllParam)

randNodeExtProb = AllParam.HeuristicParam.randNodeExtProb;
gama = AllParam.HeuristicParam.gama;
alpha = AllParam.HeuristicParam.alpha;

no_BackTrees = AllParam.RRTParam.No_Trees - 1;
extended_backs = zeros(no_BackTrees,2);

for j = 1:no_BackTrees
  
    ExtTreeNo = j + 1; % including Forward Tree, j is the current backTreeNo
    
    if rand < randNodeExtProb
        [randNode, randNodeID] = RandomNode(AllTrees.BackwardTrees{j});
        [AllTrees,extended] = Extend_1(randNode,randNodeID,target(j,:),AllTrees,ExtTreeNo,AllParam);
        
    else        
        % Call the function to find the closest node to target
        [nearestNode, nearestNodeID] = NearestNode(AllTrees.BackwardTrees{j}, target(j,:),AllTrees.BackKDtrees{j});
        % Call the function to extend the branch towards next target
        [AllTrees,extended] = Extend_1(nearestNode,nearestNodeID,target(j,:),AllTrees,ExtTreeNo,AllParam);       
    end
    
    NumOfNodes = length(AllTrees.BackwardTrees{j});
    if (extended ~= 0)
        %% Find Nodes in the neighborhood of the New Node
        r = gama*(log(NumOfNodes)/(NumOfNodes))^(1/alpha); % radius of near search
        nearNodes=Near(AllTrees.BackwardTrees{j},extended,r,AllTrees.BackKDtrees{j});
        
        extendedNode = AllTrees.BackwardTrees{j}(end);
        extendedNode.ID = NumOfNodes; %length(BackTree);
        
        %% Perform Rewiring Among the Near Nodes
        AllTrees = RewireNodes_Backward(AllTrees,AllParam,extendedNode,nearNodes,j); 
        
        if AllParam.RRTParam.plotBranches
            % Plot line
%             figure(AllParam.figHandle.map);
            line([AllTrees.BackwardTrees{j}(AllTrees.BackwardTrees{j}(end).parent).x  AllTrees.BackwardTrees{j}(end).x],...
                 [AllTrees.BackwardTrees{j}(AllTrees.BackwardTrees{j}(end).parent).y  AllTrees.BackwardTrees{j}(end).y] ,...
                 'Color','r','LineWidth', 1); hold on
        end
        
        extended_backs(j,:) = extended;
    end
    %% Create New KD tree with added node
    % Note That: creating from scratch is slowing down. find a way to
    % append the existing KDtree, e.g. octaTrees
    AllTrees.BackKDtrees{j} = KDTreeSearcher([AllTrees.BackwardTrees{j}.x ; AllTrees.BackwardTrees{j}.y]');
end
        
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% RANDOM NODE FUNCTION
function [randNode, randNodeID] = RandomNode(tree)
    randNodeID = randi([1,length(tree)],1);
    
    randNode(1) = tree(randNodeID).x;
    randNode(2) = tree(randNodeID).y;
end