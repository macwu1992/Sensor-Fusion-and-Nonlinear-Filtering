function [x, P] = mu_m(x, P, mag, Rm, m0)

    q   = x(1:4);
    b_w = x(5:7);
    b_a = x(8:10);

    % calculate yacc = h(x)
    hx  = Qq(q)' * m0;
    
    % calculate jacobian(h(x),x)
    [Q0, Q1, Q2, Q3] = dQqdq(q);
    dhx = [Q0'*m0 Q1'*m0 Q2'*m0 Q3'*m0 zeros(3) zeros(3)];
    
    % calculate inovation covariance and kalman gain
    S = dhx * P * dhx' + Rm;
    K = P * dhx' / S;

    % update
    x = x + K * ( mag - hx );
    P = P - K * S * K'; 
end