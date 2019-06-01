function [x, P] = tu_qw(x, P, omega, T, Rw)

    F = eye(4) + 0.5 * Somega(omega) * T;
    G = 0.5 * Sq(x) * T;

    % predict using motion model:
    x = F*x;
    P = F*P*F' + G*Rw*G';
end
