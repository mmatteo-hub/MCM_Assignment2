%% Error function
% Inputs:
% - bTe:  transformation matrix that describes the current position of the
%         end-eff w.r.t. the base;
% - bTg:  goal transformation matrix w.r.t. the base;
%
% Output:
% - err:  a Vect3 representing the angular and linear error committed, in 
%         particular err(1:3) represents the angular error while err(4:6)
%         the linear one;

function [err]= ComputeError(bTe, bTg)

    % obtain the rotational matrices
    bRe= bTe(1:3, 1:3);
    bRg= bTg(1:3, 1:3);
    
    % obtain the position vectors
    r_be= bTe(1:3, 4);
    r_bg= bTg(1:3, 4);
    
    % compute the misalignment vector (angular error)
    rho= VersorLemma(bRe, bRg);
    
    % compute the second objective vector (linear error)
    s= r_bg - r_be;
    
    % return the error
    err= [rho; s];
    
end

