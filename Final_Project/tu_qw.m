function [x, P] = tu_qw(x, P, omega, T, Rw)

    q   = x(1:4);
    b_w = x(5:7);
    b_a = x(8:10);

    F = eye(4) + 0.5 * Somega(omega) * T;
    G = 0.5 * Sq(q) * T;
    
    A = [ F,                G,  zeros(4,3)  ;
          zeros(3,4),   eye(3), zeros(3)    ;
          zeros(3,4), zeros(3), eye(3)      ];
    GAMMA = [G; 
             zeros(3); 
             zeros(3)];

    % predict using motion model:
    % x_{k+1} = A(u)*x_{k} + GAMMA*v_{k},  where v_{k}~N(0,Rw)
    x = A*x;
    P = A*P*A' + GAMMA*Rw*GAMMA';
end

