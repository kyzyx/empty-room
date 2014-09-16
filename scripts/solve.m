function [p,l,r] = solvechannel(A, B, C, weights)
    w = diag(1-weights);
    p = fminbnd(@(x) optimization(x,A,B,C,weights),0,2);
    [l r] = lsqnonneg(w*A,w*B/p - C);
    r = sqrt(r*p/length(B));
end
function y = optimization(p, A, B, C, weights)
    w = diag(1-weights);
    [x y] = lsqnonneg(w*A, w*B/p - C);
    y = y*p;
end

filename = 'eq.m';
run(filename);
[p0,l0,r0] = solvechannel(A,B0,C0,weights);
[p1,l1,r1] = solvechannel(A,B1,C1,weights);
[p2,l2,r2] = solvechannel(A,B2,C2,weights);

fprintf('MRSE: %.3f %.3f %.3f\n', r0,r1,r2);
fprintf('Reflectance: %.3f %.3f %.3f\n', p0, p1, p2);
for i=1:length(l0),
    fprintf('Light %d emittance: %.3f %.3f %.3f\n', i, l0(i)/pi, l1(i)/pi, l2(i)/pi);
end
