% This file is created by U. Cem Kaya - 2018
% This function performs the Rewiring operation of RRT* among the nearNodes
% First checks whether the extended node has any other better parent among 
% near nodes to give higher utility, then, checks the near nodes whether
% the extended node could be a better parent resulting higher utility.
%% REWIRING FUNCTION FOR BACKWARD TREES
function AllTrees = RewireNodes_Backward(AllTrees,AllParam,extendedNode,nearNodes,BackwardTreeID)

pathTree = AllTrees.BackwardTrees{BackwardTreeID};
Tree = AllTrees.BackwardDiGraphs{BackwardTreeID};

stepSize = AllParam.Constraints.stepSize;

if length(nearNodes) > 1
      
    psiDot_Constraint = AllParam.Constraints.psi_dot;
        
    for jj = 1:length(nearNodes)
        
        NearNode = pathTree(nearNodes{jj}.ID);
        psiNear = NearNode.psi;
        psi_Ext = atan2((extendedNode.y-NearNode.y),(extendedNode.x-NearNode.x));
        
        angDiff = psi_Ext - psiNear;
        
        if angDiff < -pi
            angDiff = angDiff + 2*pi;
        elseif angDiff > pi
            angDiff = angDiff - 2*pi;
        end
        
        dist = Dist([extendedNode.x extendedNode.y], [NearNode.x NearNode.y]);
        Velocity = NearNode.vel;
        delta_t_max = (dist/Velocity)*AllParam.Constraints.delta_t;
        
        if (nearNodes{jj}.ID ~= pathTree(extendedNode.ID).parent)&&((abs(angDiff)/delta_t_max)<=psiDot_Constraint)           
            %%
            % Here, there has to be an extend loop to more accurately
            % calculate the utility of extending a branch for rewiring that
            % has a distance arbitrary larger than the stepsize. same is
            % true for backward and the below process
            %%
            nearest = [NearNode.x NearNode.y];
            target = [extendedNode.x extendedNode.y];
            nearestNodeID = NearNode.ID;
%%          
            if dist > stepSize
                
                [~,PathUtilityAlongNearNode,NodeUtility,fail] = ExtendMultiple_Bwd(nearest, target,nearestNodeID,BackwardTreeID,AllTrees,AllParam,dist);
               
            else
                
                [PathUtilityAlongNearNode,NodeUtility,~,~] = UtilitySegment_Backward([extendedNode.x extendedNode.y],nearNodes{jj}.ID,AllTrees,BackwardTreeID,AllParam,dist);
                
                fail = false;
            end
                        
            
            if (PathUtilityAlongNearNode > extendedNode.PathUtility)&&(fail==false)
                
                extendedNode.parent = nearNodes{jj}.ID;
                extendedNode.PathUtility = PathUtilityAlongNearNode;
                extendedNode.NodeUtility = NodeUtility;
                extendedNode.totalDistance = pathTree(extendedNode.parent).totalDistance + Dist([extendedNode.x extendedNode.y],[nearNodes{jj}.x nearNodes{jj}.y]);
                extendedNode.time = pathTree(extendedNode.parent).time + Dist([extendedNode.x extendedNode.y],[nearNodes{jj}.x nearNodes{jj}.y])/pathTree(extendedNode.parent).vel;
                extendedNode.psi = psi_Ext;
            end
        end
    end

    Tree = rmedge(Tree,pathTree(extendedNode.ID).parent,extendedNode.ID);
    Tree = addedge(Tree,extendedNode.parent,extendedNode.ID);
                
    pathTree(extendedNode.ID).parent = extendedNode.parent;
    pathTree(extendedNode.ID).PathUtility = extendedNode.PathUtility;
    pathTree(extendedNode.ID).NodeUtility = extendedNode.NodeUtility;
    pathTree(extendedNode.ID).totalDistance = extendedNode.totalDistance;
    pathTree(extendedNode.ID).time = extendedNode.time;
    pathTree(extendedNode.ID).psi = extendedNode.psi;
    
    for jj = 1:length(nearNodes)
         
        NearNode = pathTree(nearNodes{jj}.ID);       
        psiNear = atan2((NearNode.y-extendedNode.y),(NearNode.x-extendedNode.x));
        psi_Ext = extendedNode.psi;
        
        angDiff =  psiNear - psi_Ext;
        if angDiff < -pi
            angDiff = angDiff + 2*pi;
        elseif angDiff > pi
            angDiff = angDiff - 2*pi;
        end
        
        dist = Dist([extendedNode.x extendedNode.y], [NearNode.x NearNode.y]);
        Velocity = extendedNode.vel;
        delta_t_max = (dist/Velocity)*AllParam.Constraints.delta_t;
        
        if (nearNodes{jj}.ID ~= pathTree(extendedNode.ID).parent)&&((abs(angDiff)/delta_t_max)<=psiDot_Constraint)
        
            %%
            % Here, there has to be an extend loop to more accurately
            % calculate the utility of extending a branch for rewiring that
            % has a distance arbitrary larger than the stepsize. same is
            % true for backward and the below process
            %%
            target = [NearNode.x NearNode.y];
            nearest = [extendedNode.x extendedNode.y];
            nearestNodeID = extendedNode.ID;
            
            if dist > stepSize
                
                [~,PathUtilityAlongExtendedNode,NodeUtility,fail] = ExtendMultiple_Bwd(nearest, target,nearestNodeID,BackwardTreeID,AllTrees,AllParam,dist);
         
            else
                
                [PathUtilityAlongExtendedNode,NodeUtility,~,~] = UtilitySegment_Backward([nearNodes{jj}.x nearNodes{jj}.y],extendedNode.ID,AllTrees,BackwardTreeID,AllParam,dist);
                
                fail = false;
            end
            
            
            if (PathUtilityAlongExtendedNode > NearNode.PathUtility)&&(fail==false)%&&(nearNodes(jj).parent ~= extendedNode.parent)
                          
                nearNodes{jj}.parent = extendedNode.ID;
                
                costChange = PathUtilityAlongExtendedNode - NearNode.PathUtility;
                
                totalDistance = pathTree(nearNodes{jj}.parent).totalDistance + Dist([extendedNode.x extendedNode.y],[nearNodes{jj}.x nearNodes{jj}.y]);
                distanceChange = totalDistance - NearNode.totalDistance;
                
                TotalTime = pathTree(nearNodes{jj}.parent).time + Dist([extendedNode.x extendedNode.y],[nearNodes{jj}.x nearNodes{jj}.y])/pathTree(nearNodes{jj}.parent).vel;
                timeChange = TotalTime - NearNode.time;               
                
                Tree = rmedge(Tree,pathTree(nearNodes{jj}.ID).parent,nearNodes{jj}.ID);
                Tree = addedge(Tree,extendedNode.ID,nearNodes{jj}.ID);
                
                if isdag(Tree)
                    
                    pathTree(nearNodes{jj}.ID).parent = nearNodes{jj}.parent;
                    pathTree(nearNodes{jj}.ID).PathUtility = PathUtilityAlongExtendedNode;
                    pathTree(nearNodes{jj}.ID).NodeUtility = NodeUtility;
                    pathTree(nearNodes{jj}.ID).totalDistance = totalDistance;
                    pathTree(nearNodes{jj}.ID).time = TotalTime;
                    
                    pathTree(nearNodes{jj}.ID).psi = psiNear;
                    
                    % update all the children nodes starting from near node
                    pathTree = childrenNodesUpdate(pathTree,Tree,nearNodes{jj}.ID,costChange,distanceChange,timeChange);
                                     
                else
                    
                     Tree = rmedge(Tree,extendedNode.ID,nearNodes{jj}.ID);
                     Tree = addedge(Tree,pathTree(nearNodes{jj}.ID).parent,nearNodes{jj}.ID);               

                end
                
            end
        end
    end

    %% Update the pathTree and DiGraph
    AllTrees.BackwardTrees{BackwardTreeID} = pathTree;
    AllTrees.BackwardDiGraphs{BackwardTreeID} = Tree;
end


end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Function to Update Child Nodes
function PathTree = childrenNodesUpdate(PathTree,Tree,startingNodeID,costChange,distanceChange,timeChange)

childrenNodeIDs = dfsearch(Tree,startingNodeID);

for j = 2:length(childrenNodeIDs)
    
    id = childrenNodeIDs(j); 
    
    PathTree(id).PathUtility = PathTree(id).PathUtility + costChange;
    PathTree(id).time = PathTree(id).time + timeChange;
    PathTree(id).totalDistance = PathTree(id).totalDistance + distanceChange;
    
end

end