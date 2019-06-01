function [x, P] = mu_m(x, P, mag, Rm, m0)

    % calculate yacc = h(x)
    hx  = Qq(x)' * m0;
    
    % calculate jacobian(h(x),x)
    [Q0, Q1, Q2, Q3] = dQqdq(x);
    dhx = [Q0'*m0 Q1'*m0 Q2'*m0 Q3'*m0];
    
    % calculate inovation covariance and kalman gain
    S = dhx * P * dhx' + Rm;
    K = P * dhx' / S;

    % update
    x = x + K * ( mag - hx );
    P = P - K * S * K'; 
end