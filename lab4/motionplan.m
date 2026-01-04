function qref = motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    %qref is piecewise cubic polynomial,
    % q0 is column vec of initial joint angles
    % q2 is column vec of final joint angles
    % t1 is the start time of PWCP
    % t2 is the finish time
    % obs is an obstacle structure, tol is the tolerance

    % Initialization
    q = q0;
    alpha_att = 0.01;
    alpha_rep = 0.01;

    % Terminate algorithem until the difference between q and desired q is
    % within tolerance
    while (norm(wrapTo2Pi(q(end, 1:5)) - wrapTo2Pi(q2(1:5))) >= tol)
        tau_att_i = att(q(end, 1:6), q2, myrobot);
        tau_rep_i = zeros(1, 6); % Initialize tau_rep_i if no obstacles

        % accumulate repulsive forces for all obstacles
        for i = 1:length(obs)
            rep(q(end, 1:6), myrobot, obs{i});
            tau_rep_i = tau_rep_i + rep(q(end, 1:6), myrobot, obs{i});
        end

        q_i = q(end, 1:6) + alpha_att * tau_att_i + alpha_rep * tau_rep_i;
        q = [q; q_i];

        q(end,:);
        norm(q(end, 1:5) - q2(1:5));
    end

    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q'); % defines a spline object with interpolation
    % times in t and interpolation values the columns of q
end