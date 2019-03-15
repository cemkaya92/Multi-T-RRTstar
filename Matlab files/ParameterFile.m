% Last Updated: 15th March 2019
% 
% *Author:* Uluhan Cem Kaya
%  
% *Description:* This file defines the Parameters for the Path Planning
% problem. These parameters are divided into 9 categories. List of
% categories and the short description is below. At the end of the file, 
% all the parameters are put under a main struct (AllParam) for the 
% accessability. For more detail, check the comments on the individual parameters.
% 
% \Categories:
%       - RRTParam:
%                   (RRT specific and planning parameters are grouped 
%                   together. i.e. Step Size, maximum iteration, display 
%                   tree branches, start/goal configurations etc.) 
% 
%       - GridMapParam:
%                   (Resolution of the Cost Mapping, grid representation 
%                   parameters, planning map sizes, and geo-references 
%                   are under this category)
% 
%       - HeuristicParam:
%                   (Heuristics for randomized sampling and planning
%                   parameters such as Goal Biases, Transition Test
%                   temperatures, RRT* rewiring parameters, and sample
%                   distribution methods)
% 
%       - PREM:
%                   (Composed of Probabilistic Risk Exposure Map layers,
%                   and their relative weight factors. Map is represented
%                   as a 2 dimensional Matrix created by the meshgrid.)
% 
%       - PURM:
%                   (Originally defined as Probabilistic UAS Reachability
%                   Map, and it is intended to group the parameters related
%                   to UAS Failure and Reachability parameters. It includes
%                   Failure Rates and impact zone parameters where the
%                   impact zones are represented as elliptical regions.)
% 
%       - UtilityThreshold:
%                   (Utility thresholds are defined for the Planning to
%                   eliminate some of the paths according to specified
%                   thresholds. These thresholds can be selected as maximum
%                   length of a path, maximum risk exposure of a node,
%                   or minimum cumulative utility along the path.)
% 
%       - obstacles:
%                   (obstacle parameters are saved in this struct. Here,
%                   the obstacles are assumed to be polygonal, and given
%                   the corners of a polygon, centroid of the polygon and
%                   the circle enclosing all the corners are computed and
%                   saved in the struct.)
% 
%       - Constraints:
%                   (Vehicle specific constraints such as kinematic/dynamic
%                   constraints, i.e. maximum yaw rate, acceleration, speed,
%                   and also the path planning solution constraints such as 
%                   maximum delta_t or the step size of the solution.
% 
%       - figHandle:
%                   (Figure and Solution of the Path figures are saved as
%                   figure handles during the iterations. It is used to
%                   visualize the solutions during the process.)
% 
% 
%  NOTE: As mentioned before, all these categories are put under a main
%        struct to pass around easily. Therefore, using a dot (.) notation,
%        they could be easily accessed and modified.
% 
%        Exm: AllParam.RRTParam.iterMax = 10000;
% 

function AllParam = ParameterFile
%%%%%%%%%%%%%%%%%%%%%%%%%%%% DEFINE PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% User Defined Parameters
scale = 1.0; % scaling of the velocity without changing grid size
RRTParam.Vdes = scale*40/3600; %km/sec Desired velocity of the UAS - constant
solver_delta_t = 1; % if you'd like use steer function for the extension, you can use with ODE solvers
RRTParam.stepSize = RRTParam.Vdes*solver_delta_t; % maximum extension step distance
RRTParam.distThresh = 0.3*RRTParam.stepSize; % distance threshold to connect nodes on separate trees
RRTParam.iterMax = 5000; %maximum number of iterations to run algorithm
% Visualization Parameters
RRTParam.plotBranches = 1;% true/false, to plot the each branch extension in every iteration (very slow);
RRTParam.RecordVideo = 1; % if you would like to plot the paths in figure and save in video file
if RRTParam.RecordVideo %% if you are recording, create a folder to save figures, and video
    mkdir figs
    dir = pwd; RRTParam.pathFigs = [dir,'\figs\'];
    % Visualization Object Creation
    RRTParam.writerObj = VideoWriter('PathsFound.avi');
    RRTParam.writerObj.FrameRate = 0.5;
    open(RRTParam.writerObj);
end

%% Construct Risk Exposure Map from the GIS Building Footprints
GridMapParam.gridSize = 0.1*RRTParam.stepSize/scale; % Grid size of the constructed PREM map (from square grids), finer the resolution slower the algorithm
[Building_PREM,GridMapParam,~,~,utmstruct] = buildingFootprints_GIS(GridMapParam);
GridMapParam.UTM = utmstruct; % UTM geo parameters

PREM.layer1 = Building_PREM;
PREM.weights = [1 ;0.0]; % if multiple layers are used, weigh them
% Plot PREM layer over the map
figHandle.map = PREMplot(PREM,GridMapParam);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Start - Finish Locations
% if there are pre-specified locations load them
if isfile('inputLocations.mat')
    load('inputLocations.mat');
else % else choose from the screen
    disp('Select Start and Goal Locations from the Map')
    [x_in,y_in] = ginput;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

RRTParam.start = [x_in(1) , y_in(1)]; % Root of Forward Tree
RRTParam.finish = [x_in(2:end) , y_in(2:end)]; % Roots of each Backward Tree
RRTParam.No_Trees = length(x_in); % Number of total trees
% node utilities can be defined below
RRTParam.InitNodeUtility = [0 5 4 3]*1e-9; % If there is any utility value on the root nodes of the trees
RRTParam.DoNotFlyUtility = 0;

%% RRT Heuristics Parameters
% Transition Parameters for T-RRT heuristic
T.rate = 0.02; % transition rate [0 - 1], higher rates make exploration to look steeper hills
T.min = 0; T.max = 5e-7;
T.range = T.max - T.min; % initial cost range between max and min NodeUtility
T.temp = 5e-7; % Initial Transition Temperature
HeuristicParam.samplingMethod = 2; % 1 - Uniformly Random, 2 - Quasi Random
if HeuristicParam.samplingMethod == 2
    HeuristicParam.quasiNumbers = haltonset(2*(length(RRTParam.finish(:,1))+1),'Skip',1e3,'Leap',1e2); % quasi random numbers
end
HeuristicParam.goalProb = 0.01; % Goal bias heuristic to bias forward tree to the goals
HeuristicParam.connectProb = 0.02; % Connection bias for backward trees to connect forward tree
HeuristicParam.randNodeExtProb = 0.001; % instead of selecting the nearest node to extend all the time, sometimes you can randomly select a node to extend
HeuristicParam.Temp = T; % T-RRT Temperatures are put under Heuristics Struct
% parameters for RRTstar near nodes calculation
HeuristicParam.gama = 7*RRTParam.stepSize; %radius of rewiring with logarithmic decrease.
HeuristicParam.alpha = 3.5; % (1/alpha), dimensions of planning
HeuristicParam.Forw_Back_ExtRate = 1; % Ratio between Forward Tree Extension iteration and Backward Tree Extension in a sequential way i.e. 5 to 1

%% Kinematic/Dynamic Constraints of UAS
Constraints.psi_dot = (60)*pi/180; % to limit yaw motion of the vehicle during extension
Constraints.delta_t = solver_delta_t; % dt_max
Constraints.stepSize = RRTParam.stepSize;

%% Utility Constraints
UtilityThreshold.Cumulative = -inf; % can be used to eliminate branches with high cumulative risk exceeding this number
UtilityThreshold.ExposureRate = -inf; % can be used to eliminate the extension branches to high instantenous risk locations
UtilityThreshold.TotalLength = inf; % can be used to eliminate the extension of branches exceeding this distance

%% Ground Impact Parameters
PURM.theta = (pi/180)*[0 -30 0 30]'; % relative orientation of the elliptical impact domains wtr body frame
PURM.reachableRadius = RRTParam.stepSize*[(6) (4); (4.5) 2.5; (4) 2; (4.5) 2.5]/(4*scale); % a and b dimension of the ellipse
PURM.failureRate = [1e-3 5e-2 1e-2 5e-2]'/3600;
PURM.FailModes = [1 1 1 1]; % we can define the degree of catastrophy of the Fail like 1 is the highest
PURM.lambda = sum(PURM.failureRate); % total failure rate assuming Poisson Process Failures
% to prevent reachable radius to go beyond the mapSize, define ResizedMap
d = min(min(PURM.reachableRadius)); % keep away from walls
mapSize = GridMapParam.mapSize;
ResMapSize = [mapSize(1,1)+d mapSize(1,2)+d;
    mapSize(2,1)-d mapSize(2,2)-d];
GridMapParam.ResMapSize = ResMapSize;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%% END OF PARAMETER DEFINITIONS %%%%%%%%%%%%%%%%%%%%%%%%


%% Read Obstacle File to Add polygonal obstacles
if isfile('obstacles.mat')
    load('obstacles.mat'); % corners of an obstacle in struct polyObst
    obstacles = polygonalObstacles(figHandle,polyObst);
else
    obstacles = [];
end

%% Plot the Start and Goal Location Markers
plot(RRTParam.start(1) , RRTParam.start(2),'g^',...
     RRTParam.finish(1,1), RRTParam.finish(1,2),'rs',...
     RRTParam.finish(2:end,1), RRTParam.finish(2:end,2),'ys',...
     'MarkerSize',10,'LineWidth',1.8);hold all; drawnow

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Put all the Parameter in a Main Struct
AllParam.RRTParam = RRTParam;
AllParam.GridMapParam = GridMapParam;
AllParam.HeuristicParam = HeuristicParam;
AllParam.PREM = PREM;
AllParam.PURM = PURM;
AllParam.UtilityThreshold = UtilityThreshold;
AllParam.obstacles = obstacles;
AllParam.Constraints = Constraints;
AllParam.figHandle = figHandle;

end


