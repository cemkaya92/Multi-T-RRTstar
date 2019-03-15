% This file is created by U. Cem Kaya - Spring 2019
% This function uses probabilistic bias to determine whether to choose
% specified finish location or any random configuration to be target. Given
% the Tree Type, Forward and Backward Trees select different finish
% locations. Forward tree selects either any of the backward trees root or
% random location. Backward trees can select the random configuration, last
% extended forward tree branch or the start location. User can also select
% random number generator type among uniform and quasi rng.
%% CHOOSE TARGETS FOR TREES TO BE EXTENDED TO
function target = ChooseTarget_Multi(finish,AllParam,iter_No,TreeType)

goalProb = AllParam.HeuristicParam.goalProb;
ResMapSize = AllParam.GridMapParam.ResMapSize;
% to decide whether choosing multi target or single target
if TreeType == 1 % Forward Type  
    TargetNo = 1; % target for forward tree
else
    % targets for backward trees
    TargetNo = AllParam.RRTParam.No_Trees - 1; % -1 is Forward Tree
end
% Create a random value between 0 and 1
p = rand;
    
    switch AllParam.HeuristicParam.samplingMethod
        
        case 2
            %% QUASI-RANDOM SAMPLING
            succeed = ([finish(:,1) ~= 0]);
            target = zeros(TargetNo,2);
            GoalNO = length(finish(:,1));
            
            if (GoalNO > 1) && (TreeType == 1)
                if ( (p > 0) && (p < goalProb) )
                    goal = finish(1,:);
                else
                    goal = finish(randi([2 GoalNO],1),:);
                end          
            else            
                goal = finish;
            end
            
            for j = 1:TargetNo
                
                if ( (p > 0) && (p < goalProb) ) && succeed(1)
                    target(j,:) = goal;
                elseif ( (p > goalProb) && (p < 1) )
                    %r = a + (b-a).*rand(100,1); range of [a, b]
                    target(j,1) = (ResMapSize(1,1) + (ResMapSize(2,1)-ResMapSize(1,1))*AllParam.HeuristicParam.quasiNumbers(iter_No,2*(j-1)+1));
                    target(j,2) = (ResMapSize(1,2) + (ResMapSize(2,2)-ResMapSize(1,2))*AllParam.HeuristicParam.quasiNumbers(iter_No,2*(j-1)+2));
                end
                
            end

        case 1           
            %% UNIFORM RAND SAMPLING
            succeed = ([finish(:,1) ~= 0]);
            target = zeros(TargetNo,2);
            GoalNO = length(finish(:,1));
            
            if (GoalNO > 1) && (TreeType == 1)
                if ( (p > 0) && (p < goalProb) )
                    goal = finish(1,:);
                else
                    goal = finish(randi([2 GoalNO],1),:);
                end          
            else            
                goal = finish;
            end
            
            for j = 1:TargetNo
                
                if ( (p > 0) && (p < goalProb) ) && succeed(1)
                    target(j,:) = goal;
                else
                    target(j,1) = (ResMapSize(1,1) + (ResMapSize(2,1)-ResMapSize(1,1))*rand);
                    target(j,2) = (ResMapSize(1,2) + (ResMapSize(2,2)-ResMapSize(1,2))*rand);%r = a + (b-a).*rand(100,1); range of [a, b]
                end
                
            end

    end
end