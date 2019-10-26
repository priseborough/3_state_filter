function quat = AlignTilt( ...
    initAccel)  % initial accelerometer vector
    tiltMagnitude = atan2(sqrt(dot([initAccel(1);initAccel(2)],[initAccel(1);initAccel(2)])),-initAccel(3));
    % take the unit cross product of the accel vector and the -Z vector to
    % give the tilt unit vector
    if (tiltMagnitude > 1e-3)
        tiltUnitVec = cross([initAccel(1);initAccel(2);initAccel(3)],[0;0;-1]);
        tiltUnitVec = tiltUnitVec/sqrt(dot(tiltUnitVec,tiltUnitVec));
        tiltVec = tiltMagnitude*tiltUnitVec;
        quat = [cos(0.5*tiltMagnitude); tiltVec/tiltMagnitude*sin(0.5*tiltMagnitude)];
    else
        quat = [1;0;0;0];
    end
end