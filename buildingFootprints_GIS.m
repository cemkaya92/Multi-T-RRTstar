% This file is created by U. Cem Kaya - 2018
% This file reads Footprints.shp file dowloaded from GIS database having
% the footprints of a specific area, and then, compute the map parameters
% and the parameters required for PREM construction using Gaussian
% distributions. 
%% CONSTRUCTION OF PREM FROM BUILDING FOOTPRINTS 
function [Building_PREM,GridMapParam,BuildingCenters,buildingData,utmstruct]...
        = buildingFootprints_GIS(GridMapParam) 

grid_size = GridMapParam.gridSize;
Populations = [];

buildingData=shaperead('Footprints.shp','UseGeoCoords', true);
N=length(buildingData);
Populations(end+1) = N;

mapSize_GPS = [32.72114 -97.12790 ;
               32.72412 -97.12313 ];% GPS
% find the UTM zone using the center of the map
UTMzone = utmzone(0.5*(mapSize_GPS(1,1)+mapSize_GPS(2,1)),0.5*(mapSize_GPS(1,2)+mapSize_GPS(2,2)));

% assign UTM zone as a coordinate frame
utmstruct = defaultm('utm'); 
utmstruct.zone = UTMzone;  
utmstruct.geoid = wgs84Ellipsoid;
utmstruct = defaultm(utmstruct);

% mapSize in meters
[mapSize(:,1),mapSize(:,2)] = mfwdtran(utmstruct,mapSize_GPS(:,1),mapSize_GPS(:,2));

mapSize = mapSize/1000; %in km

% % add an empty frame around the map
mapSize(1,1) = mapSize(1,1)-0.1*grid_size; mapSize(2,1) = mapSize(2,1)+0.1*grid_size;
mapSize(1,2) = mapSize(1,2)-0.1*grid_size; mapSize(2,2) = mapSize(2,2)+0.1*grid_size;

lat_min=mapSize(1,1);
lon_min=mapSize(1,2);

lat_grid=lat_min:grid_size:mapSize(2,1); % in Km
lon_grid=lon_min:grid_size:mapSize(2,2);

lat_max=lat_grid(end);
lon_max=lon_grid(end);

%% calculate area limits and generate grid
% memory allocation for speed
mu = zeros(N,2);
buildingArea = zeros(N,1);
corners(N) = struct('X',[],'Y',[]);
totalArea = 0;
sigma = zeros(2,2,N); % covariance matrix

for k=1:N   % find the centroid of each building
                   
    nonNanElementID = ~isnan(buildingData(k).Lat(:));
    corners_Lat = buildingData(k).Lat(nonNanElementID);
    corners_Lon = buildingData(k).Lon(nonNanElementID);
    
    [x_in_m,y_in_m] = mfwdtran(utmstruct,corners_Lat',corners_Lon'); %in meters
    corners(k).X = (x_in_m)/1000; % in km
    corners(k).Y = (y_in_m)/1000; % in km

    [ geom, ~, ~ ] = polygeom( [corners(k).X]', [corners(k).Y]' );
 
    % polygon area and centroid
    % I used a code for this. check the code for details
    buildingArea(k) = geom(1);
    mu(k,:) = [geom(2) , geom(3)]; % in km
     
    sigma(:,:,k) = cov(corners(k).X,corners(k).Y);
    
    totalArea = totalArea + buildingArea(k); % I will use to normalize building area to use them as weights
    
end

BuildingWeights = buildingArea/totalArea;

BuildingCenters = mu;

%% Construct PREM from the calculated Gaussian Parameters
No_Grid_x = (length(lat_grid)-1); No_Grid_y = (length(lon_grid)-1); % I have -1 here since length of lon/lat_grid are 1 more than number of grid between range 
[Building_PREM,dX,dY,X_mesh,Y_mesh] = construct_PREM(BuildingCenters,sigma,BuildingWeights,[lat_min lat_max],[lon_min lon_max],No_Grid_x,No_Grid_y);

%% GridMap Parameters as a struct
% [X,Y] mesh
GridMapParam.X_mesh = X_mesh; 
GridMapParam.Y_mesh = Y_mesh;
% dX, dY increments
GridMapParam.dX = dX;
GridMapParam.dY = dY;
% mapSize in km
GridMapParam.mapSize = mapSize;
GridMapParam.mapSize_GPS = mapSize_GPS;
GridMapParam.Populations = Populations;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Construct PREM Matrix from the multivariate Gaussians 
function [PREM_Matrix,dX,dY,X,Y] = construct_PREM(mu,sigma,weights,xrange,yrange,No_Grid_x,No_Grid_y)

dX = (xrange(2)-xrange(1))/No_Grid_x;
dY = (yrange(2)-yrange(1))/No_Grid_y;

x1 = (xrange(1)+dX/2:dX:xrange(2)-dX/2);
y1 = (yrange(1)+dY/2:dY:yrange(2)-dY/2);
% Create the X and Y list for contour plot
[X,Y] = meshgrid(x1,y1); % These X and Y's are the center of each small grid

 % Initialize the PREM evaluation
PREM_Vector = zeros((No_Grid_x)*(No_Grid_y),1); % vector operations may be faster
for ii = 1:length(mu)    % For each PDF in the PREM
    
    F = mvnpdf([X(:) Y(:)],mu(ii,:),sigma(:,:,ii));        
    % Evaluate and superimpose each PDF
    PREM_Vector = PREM_Vector + weights(ii)*F;
end

% This is for efficiency
% I ignored the very low risk values and made the matrix sparse
epsilon = 1e-6;

PREM_Vector(PREM_Vector < epsilon) = 0;
PREM_Matrix = reshape(PREM_Vector,length(y1),length(x1));
% Sparsifying the matrix would be more efficient in a large matrix
PREM_Matrix = sparse(PREM_Matrix);

end
