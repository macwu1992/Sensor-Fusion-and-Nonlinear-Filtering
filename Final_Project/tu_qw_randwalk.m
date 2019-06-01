function [x, P] = tu_qw_randwalk(x, P, Rq)
    
% predict using motion model:
    x = x;
    P = P + Rq;
end
