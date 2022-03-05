function [theta,axisH] = r2angvec_POLITO(matR)
    theta=acos((trace(matR)-1)/2);           % angle of rotation (0-pi)
    if (sin(theta) == 0) %%%% TODO
        axisH = sqrt((diag(matR)+1)*0.5) ;
        axisH(2) = sign(matR(2,1)*axisH(1)) * axisH(2) ;
        axisH(3) = sign(matR(3,1)*axisH(1)) * axisH(3) ;
    else
        axisH=(vex(matR-matR'))/2/sin(theta);   % unit vector of rotation axis
    end
end