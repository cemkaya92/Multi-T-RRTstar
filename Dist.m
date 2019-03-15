% This file is created by U. Cem Kaya - 2018
%% EUCLIDEAN DISTANCE BETWEED TWO 2D POINTS
function distance = Dist(x, y)
    % Calculate the euclidean distance between x and y
    distance  = sqrt( (x(1) - y(1))^2 + (x(2) - y(2))^2 );
end