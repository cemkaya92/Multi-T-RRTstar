% Last Updated: 15th March 2019
% 
% *Author:* Uluhan Cem Kaya
%  
% *Description:* This function implements the Multi Tree variant of
% Transition based optimal Rapidly-Exploring Random Trees (RRT*). 
% Path planning is defined on a real 2D map scenario where the building
% footprints are used to construct the risk map of the area for the risk of 
% exposure to the UAS impact (Cost) with Gaussian distributions. A Utility 
% based cost function is developed to measure the path quality and used to 
% generate Forward and Backward tree branches originating from specified 
% start and goal locations. Algorithm runs until the maximum iteration 
% number is reached, then, outputs all the successful connections of trees 
% selecting the Final Path with highest utility. 
%  
%  *Features:*
%      - Multiple Trees  (1 Forward and multiple Backward Trees)
%      - Heruistics: Goal Bias and Transition Test (rejection sampling)
%      - Obstacle Avoidance (polygonal obstacles with circular avoid radii)
%      - Utility Maximization. 
%        (min UAS impact risk/max UAS operation utility)
% 
% *To Do List:*
%      - Faster Near and Nearest search are needed.
%        (current implementation of KDTrees are slow - the bottleneck)
%      - Adaptive Sampling Heuristics instead of uniform sampling
%      - Transition Test requires a fine initial tuning. There should be 
%        a way of making it more generic.
%      - Object Oriented Programming can be more efficient and cleaner
%        (passing all the parameters into many funtions are unnecessary and
%        inefficient even though they are under a main struct)
%      - Potential places that requires optimization of the method and code:
%             \Utility Calculation: Large matrix operations PREM and PURM
%             \Rewiring: unsorted rewire trials may exp. increase
% 
% *Usage:* Use TESTRun.m script to load and pass the required parameters 
%          that are defined in ParameterFile.m.
%          Check ParameterFile.m for details of individual parameters.

%% MAIN FUNCTION to loop Multi-T-RRT* iterations
function [MultiplePaths,AllTrees,AllParam] = Multi_T_RRTstar(AllParam)

%% Initialize Forward and Backward Trees
AllTrees.T_Forward = AllParam.HeuristicParam.Temp; % Transition-based RRT Temperature Parameter
No_Trees = AllParam.RRTParam.No_Trees;
% Initialize the Forward Tree from the start location
[AllTrees.ForwardTree, AllTrees.ForwardDiGraph] = Initialize_Tree(AllParam.RRTParam.start,1,1,AllParam.RRTParam.Vdes,AllParam.RRTParam.InitNodeUtility(1)); % [matlab structure, and graph]
% Initialize the Backward Trees from the finish locations
AllTrees.BackwardTrees = cell(No_Trees-1,1);
AllTrees.BackwardDiGraphs = cell(No_Trees-1,1);
AllTrees.T_Backwards = cell(No_Trees-1,1);
for j = 1:No_Trees-1 % type 2 tree
    [AllTrees.BackwardTrees{j}, AllTrees.BackwardDiGraphs{j}] = Initialize_Tree(AllParam.RRTParam.finish(j,:),j,2,AllParam.RRTParam.Vdes,AllParam.RRTParam.InitNodeUtility(j+1));
    AllTrees.T_Backwards{j} = AllTrees.T_Forward; % initialize all the temperatures the same
end

%% Initialize the Successful Connection Nodes' Array
% Notation: [TreeID, NodeID, connected TreeID, connected NodeID, totalTime, totalDistance, PathUtility]
% Initialize the First Path as the Do-Not-Fly Path, which means the first node of the Forward Tree 
% [First Success ID is 1st tree, 1st Node, connecting to 1st tree, 1st Node,  total time = 0, total distance = 0, path utility, 
AllTrees.NodeIDsuccesses = [1 1 0 1 0 0 AllParam.RRTParam.DoNotFlyUtility];
MultiplePaths(1,:) = AllTrees.NodeIDsuccesses(1,:);
% Initialize the Path plot handles 
AllParam.figHandle.FinalPath = [];%handles will be saved as a struct

%% Start Searching a Path
%%%%%%%%%%%%%%%%%%%%%%%%%% Multi-T-RRTstar %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Loop until nearest is close enough to goal or until iterMax is reached
fprintf('Searching for the optimum path... \n');
for iter_No =1:AllParam.RRTParam.iterMax      
    
    No_Trees = AllParam.RRTParam.No_Trees; % number of trees created
    
    % Create KDtrees to use KD search functions of Matlab
    AllTrees.ForwardKDtree = KDTreeSearcher([AllTrees.ForwardTree.x ; AllTrees.ForwardTree.y]');
    AllTrees.BackKDtrees = cell(No_Trees-1,1); % initialize Backward KD trees as a cell array
    for j = 1:No_Trees-1
        AllTrees.BackKDtrees{j} = KDTreeSearcher([AllTrees.BackwardTrees{j}.x ; AllTrees.BackwardTrees{j}.y]');
    end
    
    extended_forw = 0; % just to reset before calling in loop
    extended_backs = zeros(No_Trees-1,2);
    
    for jj = 1:AllParam.HeuristicParam.Forw_Back_ExtRate % Ratio of iterations of Forward Tree and Backward Trees
        % Call the function to select the next target for the Forward Tree
        target = ChooseTarget_Multi(AllParam.RRTParam.finish,AllParam,iter_No,1); % 1 is TreeType of Forward
        % Try to Extend Forward Tree to selected target location, then, if
        % possible, check near nodes for rewiring operation 
        [extended_forw,AllTrees] = ExtendAndRewire_Forward(target,AllTrees,AllParam);
        % Check the extension has a successful connection with other trees
        [success,AllTrees] = ConnectionCheck_MultiTree([1 3],... %% Scenario 1 and 3 are for Forward Tree extension
            extended_forw,extended_backs,AllTrees,AllParam);
        % If connection is successful, create the path
        if success
            [MultiplePaths,AllTrees,AllParam] = CreatePath(AllTrees,AllParam,MultiplePaths,iter_No);
        end
    end
    % Select Targets for Backward Trees to Extend 
    MultiTarget = ChooseTarget_Multi(extended_forw,AllParam,iter_No,2); % 2 is TreeType of Backward
    % Try to Extend Backward Trees to selected target locations, then, if
    % possible, check near nodes for rewiring operation for each tree
    [extended_backs,AllTrees] = ExtendAndRewire_Backward(MultiTarget,AllTrees,AllParam);
    % Check each extension for successful connection with forward tree or goals
    [success,AllTrees] = ConnectionCheck_MultiTree([2 4],...
        extended_forw,extended_backs,AllTrees,AllParam);
    % If connection is successful, create the path
    if success
        [MultiplePaths,AllTrees,AllParam] = CreatePath(AllTrees,AllParam,MultiplePaths,iter_No);
    end
    
    % Check the previous paths in every 100 iterations whether rewiring
    % made them better or not.
    if mod(iter_No,100) == 0
        [AllTrees,AllParam] = CheckPrevPaths(AllTrees,AllParam);
    end
                                    
end     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END of RRT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Close the Writer Object to save the Recorded Video
if AllParam.RRTParam.RecordVideo
    close(AllParam.RRTParam.writerObj);
end

end
