function q = deltajoint(delta)
    % TODO 1/2: Add proper documentation for this function.

    kuka = mykuka_search(delta);

    %-------------------------- Calibration ----------------------------%
    % TODO 2/2: Fill in values Xi and Qi for i = {1, 2, 3}. Xi are 3 by 1
    % column vectors, while Qi are 1 by 6 row vectors.
    
    X1 = [784.05, 0.18, 20.93];
    X2 = [612.46, -129.61, 23.46];
    X3 = [603.47, 159.54, 23.01];
    Q1 = [0, 0.4112, 0.0609, 0, 1.5708, 0];
    Q2 = [-0.2087, 0.7252, -0.4451, 0, 1.5708, 0];
    Q3 = [0.2581, 0.7271, -0.4494, 0, 1.5708, 0];
    %-------------------------------------------------------------------%

    H1=forward_kuka(Q1, kuka);
    H2=forward_kuka(Q2, kuka);
    H3=forward_kuka(Q3, kuka);
    
    q=norm(H1(1:3,4)-X1)+norm(H2(1:3,4)-X2)+norm(H3(1:3,4)-X3);
end