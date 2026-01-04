function tau = rep(q, myrobot, obs)
    % Initialization
    F_rep = zeros(3, 6);
    J = zeros(3,6);
    tau = zeros(6,1);
    %q2(4) = q2(4) + 2*pi;


    for i = 1:6
        % Get honogeneous transformation and position
        %H_init = myrobot.A(1:i, q');
        H_init = forward_kuka(q, myrobot, i);
        O_i = H_init(1:3, 4);

        %tolerance = 1;
        %diff_vec = O_i - O_f;
        %normalized = norm(diff_vec);

        % calclate based on different obstacle types
        % for cylinders
        if (obs.type == 'cyl')
            h = obs.h;
            R = obs.R;
            c = obs.c;
            % if directly above the cylinder (within radius of the cylinder
            if ((norm(O_i(1:2) - c) <= R) && (O_i(3) > h))
                diff_vec = [0; 0; O_i(3) - h];
                rho = norm(diff_vec);
            
            % above the cylinder but outside of the radius area
            elseif (O_i(3) > obs.h)
                theta = atan2((O_i(2) - c(2)), (O_i(1) - c(1)));
                diff_vec = [O_i(1) - c(1) - R * cos(theta); O_i(2) - c(2) - R*sin(theta); O_i(3) - h];
                rho = norm(diff_vec);

            % point is next to cylinder body (outside of R but under h)
            else
                b = (R * (O_i(1:2) - c) / norm(O_i(1:2) - c)) + c;
                diff_vec = O_i - [b; O_i(3)];
                rho = norm(diff_vec);
            end

        % for spheres
        elseif (obs.type == 'sph')
            R = obs.R;
            c = obs.c;

            b = (R* (O_i - c) / norm(O_i - c)) + c;
            diff_vec = O_i - b;
            rho = norm(diff_vec);
            
        % planar obstacles
        elseif (obs.type == 'pla')
            h = obs.h; 

            % O_i - b vertical vector from surface to point 
            % (only z component)
            diff_vec = [0; 0; O_i(3) - h];
            rho = norm(diff_vec);
        end
       
        % calculate F_rep according to formula 7.5
        if (rho <= obs.rho0)
            n = 1;
            F_rep(1:3, i) = n * ((1/rho) - (1/obs.rho0)) * (1/rho^2) * (diff_vec / rho);
        else
            F_rep(1:3, i) = zeros(3, 1);
        end

        % calculate jacobian
        for j = 1:i
          %H_j_1 = myrobot.A(1:j-1, q);
          H_j_1 = forward_kuka(q, myrobot, j-1);
          o_j_1 = H_j_1(1:3, 4);
          z_j_1 = H_j_1(1:3, 3);
          J(1:3, j) = cross(z_j_1, O_i - o_j_1);
        end

        % calculate tau
        tau =  tau + J.'*F_rep(1:3, i);

    end

    % normalize tau only if tau has value
    if (norm(tau) ~= 0)
        tau = tau/norm(tau);
    end
    tau = tau.';

end