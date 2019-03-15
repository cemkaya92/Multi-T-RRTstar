% This file is created by U. Cem Kaya - Spring 2019
%% Initialize Trees with specified parameters
function [PathTree, directedGraph] = Initialize_Tree(start,TreeNo,Type,Velocity,varargin)

% Set the first element of a start tree
PathTree(1).x = start(1);
PathTree(1).y = start(2);
PathTree(1).parent = 1;
PathTree(1).children = [];
PathTree(1).NodeUtility = 0;
if ~isempty(varargin)
    PathTree(1).PathUtility = varargin{1};
else
    PathTree(1).PathUtility = 0;
end
PathTree(1).totalDistance = 0;
PathTree(1).psi = -pi/2;
PathTree(1).vel = Velocity;
PathTree(1).time = 0;
PathTree(1).ID = 1;
PathTree(1).TreeNo = TreeNo;
PathTree(1).Type = Type;

% Create Directed Graph object of Matlab
% This allows easier dfsearch to find children or parents
directedGraph = digraph;
directedGraph = addnode(directedGraph,1);

end