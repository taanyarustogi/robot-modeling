function q = inverse_kuka(H, myrobot)

    % Extracting Rd and O from H
    Rd = H(1:3,1:3);
    od = H(1:3,4);

    % Extracting end-effector positions from our matrix
    translation = [myrobot.a(6);0;myrobot.d(6)];
    wrist_position = od - Rd*translation;
    x_c = wrist_position(1);
    y_c = wrist_position(2);
    z_c = wrist_position(3);

    % Extracting information from DH table
    d_1 = myrobot.d(1);
    d_2 = myrobot.d(2);
    d_4 = myrobot.d(4);

    a_1 = myrobot.a(1);
    a_2 = myrobot.a(2);
    a_3 = myrobot.a(3);

    r = sqrt(x_c^2+y_c^2) - a_1;
    s = z_c - d_1;
    l = sqrt(a_3^2 + d_4^2);
    alpha = atan2(a_3, d_4);
    D = (r^2 + s^2 - a_2^2 -l^2) / (2 * a_2 * l);
    phi = atan2(real(sqrt(1 - D^2)), D);

    % Calculate the inverse kinematics for the robot's joints (q1, q2, q3)
    q(1) = atan2(y_c, x_c);
    q(3) = pi/2 - phi - alpha;
    q(2) = atan2(s, r) + atan2(l * sin(phi), a_2 + l * cos(phi));
    
    % Calculate the inverse kinematics for the robot's joints (q4, q5, q6)
    H3_0 = eye(4);

    % Compute H3_0 (multiplying H3_2, H2_1, H1_0)
    for i = 1:3
        alpha = myrobot.alpha(i);
        a = myrobot.a(i);
        theta = q(i);
        d = myrobot.d(i);
        temp = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                 sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                 0, sin(alpha), cos(alpha), d;
                 0, 0, 0, 1];
        H3_0 = H3_0 * temp;
    end

    % Isolating H3_6 (multiplying H6_0, H3_0)
    r = transpose(H3_0(1:3, 1:3)) * H(1:3, 1:3);

    % Extracting joint information (q4, q5, q6)
    q(4) = atan2(r(2,3), r(1,3));    
    q(5) = atan2(sqrt(1-(r(3,3))^2), r(3,3));
    q(6) = atan2(r(3,2), -r(3,1));
end