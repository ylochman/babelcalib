% Ported from C++ implementation
% https://github.com/erich666/GraphicsGems/blob/master/gems/Roots3And4.c

function [s, num] = solve_cubic(c)
    % Solves c(1) + c(2)*x + c(3)*x^2 + c(4)*x^3 = 0

    % normal form: x^3 + Ax^2 + Bx + C = 0
    A = c(:,3) ./ (c(:,4) + eps);
    B = c(:,2) ./ (c(:,4) + eps);
    C = c(:,1) ./ (c(:,4) + eps);

    % substitute x = y - A/3 to eliminate quadratic term:
	% x^3 + px + q = 0
    sq_A = A .* A;
    p = 1.0 / 3 * (- 1.0 / 3 * sq_A + B);
    q = 1.0 / 2 * (2.0 / 27 * A .* sq_A - 1.0 / 3 * A .* B + C);

    % use Cardano's formula
    cb_p = p .* p .* p;
    D = q .* q + cb_p;

    s = ones(size(D,1),3) .* NaN;
    num = zeros(size(D,1),1);

    is_D_zero = (abs(D) < 1e-8);

    ind = find(is_D_zero & (abs(q) < 1e-8)); % one triple solution
    s(ind,1:3) = 0;
    num(ind) = 1;

    ind = find(is_D_zero & (abs(q) >= 1e-8)); % one single and one double solution
    u = cbrt(-q(ind));
    s(ind,1) = 2 * u;
    s(ind,2:3) = - u;
    num(ind) = 2;

    ind = find(D < -1e-8); % Casus irreducibilis: three real solutions
    phi = 1.0 / 3 * acos(-q(ind) ./ (sqrt(-cb_p(ind)) + eps));
    t = 2 * sqrt(-p(ind));
    s(ind,1) =   t .* cos(phi);
    s(ind,2) = - t .* cos(phi + pi / 3);
    s(ind,3) = - t .* cos(phi - pi / 3);
    num(ind) = 3;

    ind = find(D > 1e-8); % one real solution
    sqrt_D = sqrt(D(ind));
    u = cbrt(sqrt_D - q(ind));
    v = - cbrt(sqrt_D + q(ind));
    s(ind,1) = u + v;
    s(ind,2) = u + v;
    s(ind,3) = u + v;
    num(ind) = 1;

    % resubstitute
    s = s - 1.0 / 3 .* A;
end

function res = cbrt(x)
    res = zeros(size(x,1),1);

    ind = find(x > 1e-8);
    res(ind) = power(double(x(ind)), 1.0/3.0);

    ind = find(x < -1e-8);
    res(ind) = -power(-double(x(ind)), 1.0/3.0);
end