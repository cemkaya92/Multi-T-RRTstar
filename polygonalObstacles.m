% This file is created by U. Cem Kaya - 2018
% find the centroids of the obstacles and also the enclosing outer circle for the obstacles
%% CALCULATE PARAMETERS OF THE POLYGONAL OBSTACLES 
function obstacles = polygonalObstacles(figHandle,polyObst)

No_Obst = length(polyObst);
for i = 1:No_Obst
    
    r = [[polyObst(i).X]' ; [polyObst(i).Y]']; % in km
    
    [ geom, ~, ~ ] = polygeom( polyObst(i).X, polyObst(i).Y );
    % polygon area and centroid
    % I used a code for this. check the code for details
    x0 = geom(2); y0 = geom(3);
   
    outerRadius = max(sqrt((r(1,1:end-1)'-x0).^2+(r(2,1:end-1)'-y0).^2));
    
    obstacles(i).pos = r;
    obstacles(i).center = [x0;y0];
    obstacles(i).outerRadius = outerRadius;
    % plot polygon
    figure(figHandle.map)
    patch(r(1,:)',r(2,:)','red','DisplayName','No-Fly-Over Zones');
    hold on

end

end