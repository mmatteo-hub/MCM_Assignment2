%% GetFrameWrtFrame function 
% inputs: 
% - linkNumber_i : number of ith link 
% - linkNumber_j: number of jth link 
% - biTei: vector of matrices containing the transformation matrices from 
%          link i to link i+1 for the current q.
% output:
% iTj : transformationMatrix from link i and link j for the configuration
%       described in biTei.

function [iTj]= GetFrameWrtFrame(linkNumber_i, linkNumber_j, biTei)
    % Initialize iTj as the identity matrix
    iTj= eye(4);
    
    % Post-multiply all the matrices that are encountered from link i to
    % link j
    if linkNumber_i <= linkNumber_j
        for k= linkNumber_i:linkNumber_j
            iTj= iTj * biTei(:, :, k);
        end
    else
        for k= linkNumber_j:linkNumber_i
            iTj= biTei(:, :, k) * iTj;
        end
    end
    
end