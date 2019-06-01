function [x, P] = mu_g(x, P, yacc, Ra, g0)

    % calculate yacc = h(x)
    hx  = Qq(x)' * g0;
    
    % calculate jacobian(h(x),x)
    [Q0, Q1, Q2, Q3] = dQqdq(x);
    dhx = [Q0'*g0 Q1'*g0 Q2'*g0 Q3'*g0];
    
    % calculate inovation covariance and kalman gain
    S = dhx * P * dhx' + Ra;
    K = P * dhx' / S;

    % update
    x = x + K * ( yacc - hx );
    P = P - K * S * K';
end