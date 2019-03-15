% This file is created by U. Cem Kaya - Spring 2019
% This function performs the extend and rewire operations for the forward
% tree. First selects a target, Second tries to extend a branch towards to target
% if successful, then, tries to perform rewiring among the near nodes. 
%% EXTEND AND REWIRE OPERATIONS FOR THE BACKWARD TREES
function [extended,AllTrees] = ExtendAndRewire_Forward(target,AllTrees,AllParam)

randNodeExtProb = AllParam.HeuristicParam.randNodeExtProb;
gama = AllParam.HeuristicParam.gama;
alpha = AllParam.HeuristicParam.alpha;

ExtTreeNo = 1; % Forward Tree is extended
%%
if rand < randNodeExtProb
    [randNode, randNodeID] = RandomNode(AllTrees.ForwardTree);
    [AllTrees,extended] = Extend_1(randNode,randNodeID,target,AllTrees,ExtTreeNo,AllParam); 
else 
    % Call the function to find the closest node to target
    [nearestNode, nearestNodeID] = NearestNode(AllTrees.ForwardTree, target,AllTrees.ForwardKDtree);
    % Call the function to extend the branch towards next target
    [AllTrees,extended] = Extend_1(nearestNode,nearestNodeID,target,AllTrees,ExtTreeNo,AllParam);  
end
%%
NumOfNodes = length(AllTrees.ForwardTree);
if (extended ~= 0)
%% Find Nodes in the neighborhood of the New Node
    r = gama*(log(NumOfNodes)/(NumOfNodes))^(1/alpha); % radius of near search
    nearNodes=Near(AllTrees.ForwardTree,extended,r,AllTrees.ForwardKDtree);
    
    extendedNode = AllTrees.ForwardTree(end);
    extendedNode.ID = NumOfNodes; %length(ForwardTree);
    
    %% Perform Rewiring Among the Near Nodes
    AllTrees = RewireNodes_Forward(AllTrees,AllParam,extendedNode,nearNodes);
    %% Create New KD tree with added node
    % Note That: creating from scratch is slowing down. find a way to
    % append the existing KDtree, e.g. octaTrees
    AllTrees.ForwardKDtree = KDTreeSearcher([AllTrees.ForwardTree.x ; AllTrees.ForwardTree.y]');
         
    if AllParam.RRTParam.plotBranches
        % Plot line
%         figure(AllParam.figHandle.map);
        line([AllTrees.ForwardTree(AllTrees.ForwardTree(end).parent).x AllTrees.ForwardTree(end).x] , ...
             [AllTrees.ForwardTree(AllTrees.ForwardTree(end).parent).y AllTrees.ForwardTree(end).y] ,...
             'Color','b','LineWidth', 1); hold on
        drawnow;
    end   
    
end
        
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% RANDOM NODE FUNCTION
function [randNode, randNodeID] = RandomNode(tree)
    randNodeID = randi([1,length(tree)],1); 
    randNode(1) = tree(randNodeID).x;
    randNode(2) = tree(randNodeID).y;
end