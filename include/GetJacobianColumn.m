%% Inverse Kinematic Function
% Function computing the end effector jacobian column for the input
% parameters.
%
% Inputs
% - bTi:       transformation matrix from the manipulator base to the joint <i>;
%              from link <i> to link <i-1> for the current q;
% - bTe:       tranformation matrix from the base to the end effector;
% - jointType: 0 if the joint is revolute, 1 if the joint is prismatic (is
%              referred to the joint corresponding to bTe).
%
% Output
% - h:         Jacobian column h(1:3) angular part, h(4:6) linear part;

function [h] = GetJacobianColumn(bTi, bTe, jointType)
    
    % Initialize the i-th column of the Jacobian
    h= zeros(6, 1);
    
    % Unit vector that indicates the direction of the i-th joint's axis,
    % since all the frames have been assigned s.t. the z axis points in the 
    % direction of the joints' axes it holds 
    k_i= bTi(1:3, 3);
    
    % Compute the distance vector between the end-effector and joint <i>
    r_e0= bTe(1:3, 4);      % position of the end-eff w.r.t. the base
    r_i0= bTi(1:3, 4);      % position of <i> w.r.t. the base
    r_ei= r_e0 - r_i0;      % distance between end-eff and <i>
    
    if(jointType == 0) 
        % revolute joint
        h= [k_i ; cross(k_i, r_ei)]; 
    
    elseif(jointType == 1)       
        % prismatic joint
        h= [0 ; 0 ; 0 ; k_i];
        
    else                    
        % error
        error('Unrecognized link type.')
        
    end
end
