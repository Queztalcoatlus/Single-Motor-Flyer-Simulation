% Convert derivatives of roll, pitch, yaw to omega.
function thetadot = omega2thetadot(omega, angles)
    phi = angles(1);
    theta = angles(2);
    %psi = angles(3);
    W = [
        1, 0, -sin(theta)
        0, cos(phi), cos(theta)*sin(phi)
        0, -sin(phi), cos(theta)*cos(phi)
    ];
    thetadot = inverse(W) * omega;
end