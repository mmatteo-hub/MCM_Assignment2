%% VersorLemma function
% Inputs:
% - R1:   the first rotation matrix;
% - R2:   the second rotation matrix;
%
% Output:
% - rho:  the Vect3 representing the axis around which R1 should rotate to 
%         reach R2, where its modulus is the angle;

function rho = VersorLemma(R1, R2)
    % small threshold to take into account numerical errors in the
    % evaluation of cos_th
    err_threshold= 0.0001;
    
    % inverse-equivalent angle-axis algorithm to compute the misalignment
    % vector
    cos_th= 1/2*(sum(sum(R1.*R2)) - 1);
    
    if (cos_th >= (1 - err_threshold))
        % Case when cos(th) is almost 1 (no rotation, th= 0)
        rho= [0 ; 0 ; 0];
        
    elseif (abs(cos_th) < (1 - err_threshold))
        % Case when abs(cos(th)) smaller then 1
        v_times_sin_th= 1/2*(sum(cross(R1, R2, 1)));
        sin_th= norm(v_times_sin_th);
        
        th= atan2(sin_th, cos_th);
        v= v_times_sin_th/sin_th;
        
        rho= (v*th)';
        
    else
        % Case when cos(th) is almost -1 (flip around a single axis th= pi)
        R= R1 + R2;
        th= pi;
        v= zeros(3, 1);
        for i= 1:3
            col_i= R(:, i);
            if( abs(col_i(1)) > err_threshold || ...
                abs(col_i(2)) > err_threshold || ...
                abs(col_i(3)) > err_threshold  )  
           
               % choose as axis of rotation the non-null column
               v= col_i;
               % normalize the vector
               v= v/norm(v);
            end
        end
       
        rho= v*th;
    end
end

