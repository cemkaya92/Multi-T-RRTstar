% This File is created by U. Cem Kaya - Spring 2019
% This function performs the Transition Test to reject the sampling
% according to the current Temperature parameters of the tree. This is an
% adaptive process. If the rejection occurs, temperature increases to allow
% higher cost (lower utility) nodes to be sampled. Vice versa, if the test
% passes it also decreases the temperature to search lower cost (higher
% utility) regions.
%% TRANSITION TEST FOR MOVING TO HIGHER COST REGION
function [test,T] = transitionTest(utility_parent, utility_child, T)
    T_rate = T.rate; % rate of temperature increase
    costRange = T.range; % utility difference between min and max from nodes
    Temp = T.temp; % temperature
    if utility_child < utility_parent % moving uphill
        
        if exp(-(utility_parent - utility_child)/Temp) > 0.01 % depending on this probability, choose to move uphill
            test = 1;
            Temp = Temp/(2^((utility_parent - utility_child)/costRange));
        else
            test = 0;
            Temp = Temp*2^(T_rate);
        end
        T.temp = Temp;
        
    else
        test = 1;
    end
    
%     test = 1;%% comment out to use T-test

end