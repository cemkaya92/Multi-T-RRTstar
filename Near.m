% This file is created by U. Cem Kaya - 2018
%% FUNCTION TO FIND NEAR NODES IN THE PROXIMITY OF EXTENDED NODE
function nearNodes=Near(pathTree,extended,r,KDtree)

% find node IDs of near nodes
NodeID = rangesearch(KDtree,extended,r);
 
lengthNodeID = length(NodeID{:});

bucketSize = 10; % this bucket size is used to prevent excessive computational 
                 % cost of Rewiring operations for the larger number of
                 % near nodes. However, it might also be a limiting factor
                 % for good solutions.

if lengthNodeID > 1 && lengthNodeID <= bucketSize
    
    nearNodes = cell(lengthNodeID,1);
    
    for id = 1:lengthNodeID
        nearNodes{id,1} = pathTree(NodeID{1}(id));
    end
    
elseif lengthNodeID > bucketSize
    
    nearNodes = cell(bucketSize,1);
    
    for id = 1:bucketSize
        nearNodes{id,1} = pathTree(NodeID{1}(id));
    end
    
else 
    
    nearNodes = {};
end
        

end 