function H = forward_kuka(joint, myrobot, last)

    H = eye(4); % Initialize H as a 4x4 identity matrix

    % Loop through each joint to compute the transformation matrix
    for i = 1:last
        alpha = myrobot.alpha(i);
        a = myrobot.a(i);
        theta = joint(i);
        d = myrobot.d(i);
        temp = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
                 sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                 0, sin(alpha), cos(alpha), d;
                 0, 0, 0, 1];
        H = H * temp;
    end
end