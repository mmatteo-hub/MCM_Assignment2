%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by biTei.
%
% Inputs:
% - biTei:     vector of matrices containing the transformation matrices from 
%              link <i> to link <i-1> for the current q;
% - bTe:       current transformation matrix from base to the end effector;
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
%              prismatic;
%
% Output:
% - J:         end-effector jacobian matrix;

function J = GetJacobian(biTei, bTe, jointType)
    % Get the number of joints in the manipulator's structure
    n= size(jointType, 1);
    
    % Initialize the Jacobian
    J= zeros(6, n);
    
    for i= 1:n
        % Get the transformation matrix that describes the pose of the joint 
        % <i> w.r.t. the base of the manipulator
        bTi= GetTransformationWrtBase(biTei, i);
        
        % Fill the Jacobian's columns
        J(:, i)= GetJacobianColumn(bTi, bTe, jointType(i));
    end


end
