function [cost, Dcost] = costfunc(Z, N)
    tf = Z(end);
    
    % Compute the cost value
    cost = tf;
    
    % Compute the derivative of the cost value (set to zero for this example)
    Dcost = zeros(size(Z));
    
    % You can add more computations or modify the cost value and its derivative as needed
end
