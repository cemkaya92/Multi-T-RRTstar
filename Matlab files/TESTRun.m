% Last Updated: 15th March 2019
% 
% *Author:* Uluhan Cem Kaya
% 
% *Usage:* Load the Parameters for the Path Planning to pass the RRT
% algorithm. These Parameters are defined in ParameterFile.m file as
% different groups under a main struct. Dot (.) notation of the struct can
% be used here to modify the desired parameters. Note that this sample run
% is using a specific scenario map with building footprints. To generate
% different scenario maps, new footprint.shp file are required from GIS database
% and buildingFootprints_GIS.m file needs to be updated accordingly.

clearvars, clc, close all
rng(1) % random number generator seed to repeat the same scenarios
%% Load the Parameter File 
% For detailed explanation, look at ReadMe
AllParam = ParameterFile; % All the parameters are put in a main struct AllParam

%% Modify the Parameters
AllParam.RRTParam.plotBranches = 0; %0/1 plot the branches
AllParam.RRTParam.iterMax = 5000; % maximum number of iterations
% and the other parameters can be modified as above

%% Run Main Function 
tic % start timer to time how long it takes to finish iterations

[MultiplePaths,AllTrees,AllParam] = Multi_T_RRTstar(AllParam);

toc % stop timer to print elapsed time

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% save path solutions into .mat file for later use
save('SavedScenario','MultiplePaths','AllTrees','AllParam');

% Adjust path handles to visualize final and alterative paths 
NodeIDsuccesses = AllTrees.NodeIDsuccesses;
if (isempty(NodeIDsuccesses))
    fprintf('Goal was not reached! Try changing parameters \n');
else
    set(AllParam.figHandle.FinalPath(end),'visible','on','Color','r','LineStyle','-','LineWidth',2.5)
    set(AllParam.figHandle.FinalPath(1:end-1),'visible','on','Color','b','LineStyle','--','LineWidth',0.5)
    title('\bf RRT Generated Path - Top View')  
end

