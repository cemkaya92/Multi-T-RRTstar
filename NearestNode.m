% This file is created by U. Cem Kaya - 2018
%% FIND THE NEAREST NODE TO THE TARGET FROM A TREE
function [nearestNode, nearestNodeID] = NearestNode(tree, target,KDtree)

    nearestNodeID = knnsearch(KDtree,target,'K',1);
    
    nearestNode(1) = tree(nearestNodeID).x;
    nearestNode(2) = tree(nearestNodeID).y;

end