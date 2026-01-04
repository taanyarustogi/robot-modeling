function tau = att(q,q2,myrobot)
    % Initialization
    F_att = zeros(3, 6);
    J = zeros(3,6);
    tau = zeros(6,1);
    %q2(4) = q2(4) + 2*pi;

    for i = 1:6
        % Get homogenous transformation and position 
        %H_init = myrobot.A(1:i, q);
        %H_final = myrobot.A(1:i, q2);
        H_init = forward_kuka(q, myrobot, i);
        H_final = forward_kuka(q2, myrobot, i);

        O_i = H_init(1:3, 4);
        O_f = H_final(1:3, 4);

        % calculate vector difference and F_att from textbook 7.4

        %tolerance = 1;
        zeta = 1;
        diff_vec = O_i - O_f;
        %normalized = norm(diff_vec);
        F_att(1:3,i) = - zeta * diff_vec;
        %if (normalized > tolerance)
          %  F_att(:,i) = - tolerance * zeta * diff_vec / normalized
      %  else 
           % F_att(:,i) = - zeta * diff_vec
      %  end

      % calculate jacobian
      for j = 1:i
          %H_j_1 = myrobot.A(1:j-1, q);
          H_j_1 = forward_kuka(q, myrobot, j-1);
          o_j_1 = H_j_1(1:3, 4);
          z_j_1 = H_j_1(1:3, 3);
          J(1:3, j) = cross(z_j_1, O_i - o_j_1);
      end

      % calculate tau
      tau =  tau + J.'*F_att(1:3, i);
      %F_att(1:3, i)
      %J.'
    end

    % normalize tau only if tau has value (prevent undefined)
    if (norm(tau) ~= 0)
        tau = tau/norm(tau);
    end
    tau = tau.';

end