%% A MATLAB code of Nakano's P3P solver [1].
% Copyright (c) 2020 NEC Corporation
% This software is released under the NEC Corporation License. See License.txt.
% For commercial use, please contact Gaku Nakano <g-nakano@nec.com>.
% 
% USAGE: 
%   [R, t] = p3p_nakano_bmvc2019(m, X, polishing)
%   solves P3P problem given by m_i ~ R*X_i + t (i={1,2,3}). 
%
% INPUTS:
%   m - 3x3 matrix of 2D points represented by homogeneous coordinates.
%       Each column m(:,i) corresponds to the 3D point X(:,i),
%       [u1, u2, u3
%        v1, v2, v3
%        w1, w2, w3],
%       where each column is normalized by sqrt(u^2+v^2+w^2)=1.
%
%   X - 3x3 matrix of 3D points.
%       Each column X(:,i) corresponds to the 2D point m(:,i),
%       [x1, x2, x3
%        y1, y2, y3
%        z1, z2, z3].
%
%   polishing - (optional) an integer to set the number of iterations of
%               root polishing. If <= 0, the root polishing is not performed.
%               (default: 1)
%
% OUTPUS:
%   R - 3x3xM rotation matrix (1<= M <= 4). 
%       R(:,:,i) corresponds to t(:,i).
%   t - 3xM translation vector.
%
% REFERENCE:
%   [1] G. Nakano, "A Simple Direct Solution to the Perspective-three-point
%       Problem," BMVC2019.

function [R, t] = p3p_nakano_bmvc2019(m, X, polishing)
    
    if nargin < 3
        polishing = 1;
    end
    
 
    %% permute m and X so that the longest distance is between X(:,1) and X(:,2)
    d = sqrt(sum([X(:,2)-X(:,1), X(:,3)-X(:,2), X(:,1)-X(:,3)].^2));
    [~, idx] = max(d);
    switch idx
        case 2
            X = X(:,[2,3,1]);
            m = m(:,[2,3,1]);
        case 3
            X = X(:,[1,3,2]);
            m = m(:,[1,3,2]);
    end
    


    %% rigid transformation so that all points are on a plane z=0.
    % Xg = [ 0 a b
    %        0 0 c
    %        0 0 0];
    % a~=b, b>0, c>0
    X21 = X(:,2) - X(:,1);
    X31 = X(:,3) - X(:,1);
    nx = X21; 
    nx = nx / norm(nx);
    nz = cross(nx, X31);
    nz = nz / norm(nz);
    ny = cross(nz, nx);
    N  = [nx, ny, nz];
    
    %% calcurate coefficients of the polinomial for solving projective depths
    a = N(:,1)'*X21;
    b = N(:,1)'*X31;
    c = N(:,2)'*X31;
    
    M12 = m(:,1)'*m(:,2);
    M13 = m(:,1)'*m(:,3);
    M23 = m(:,2)'*m(:,3);
    p = b/a;
    q = (b^2+c^2)/a^2;

    f = [p, -M23,  0, -M12*(2*p-1),   M13, p-1];
    g = [q,    0, -1,     -2*M12*q, 2*M13, q-1];
    
    h = [ - f(1)^2 + g(1)*f(2)^2
        f(2)^2*g(4) - 2*f(1)*f(4) - 2*f(1)*f(2)*f(5) + 2*f(2)*f(5)*g(1)
        f(5)^2*g(1) - 2*f(1)*f(5)^2 - 2*f(1)*f(6) + f(2)^2*g(6) - f(4)^2 - 2*f(2)*f(4)*f(5) + 2*f(2)*f(5)*g(4)
        f(5)^2*g(4) - 2*f(4)*f(5)^2 - 2*f(4)*f(6) - 2*f(2)*f(5)*f(6) + 2*f(2)*f(5)*g(6)
        - 2*f(5)^2*f(6) + g(6)*f(5)^2 - f(6)^2];
    x = solveQuartic(h);  % h(1)*x^4 + h(2)*x^3 + h(3)*x^2 + h(4)*x + h(5) = 0
    x = real(x(real(x)>0 & abs(imag(x))<1e-8));
    y = - (( f(1)*x+f(4) ).*x + f(6)) ./ (f(5) + f(2)*x);
    if polishing > 0
        [x, y] = rootpolishing(f, g, x, y, polishing);
    end


    %% recover motion
    nsols = length(x);
    A = m .* [-1, 1, 0];
    B = m .* [-1, 0, 1];
    C = B - p*A;

    R = zeros(3,3,nsols);
    t = zeros(3,nsols);
    for i = 1:nsols

        lambda = [1, x(i), y(i)]';
        s = norm(A*lambda) / a;
        d = lambda / s;

        r1 = (A*d) / a;                        
        r2 = (C*d) / c;
        r3 = cross(r1,r2);
        Rc = [r1, r2, r3];
        tc = d(1)*m(:,1);

        R(:,:,i) = Rc*N';
        t(:,i)   = tc - Rc*N'*X(:,1);
    end

    
end

%% Root polishing by Gauss-Newton method
function [x, y] = rootpolishing(f, g, x, y, maxitr)

    if nargin < 5
        maxitr = 1;
    end

    for i = 1:maxitr
        x2 = x.^2;
        xy = x.*y;
        y2 = y.^2;

        fv = f(1)*x2 + f(2)*xy      + f(4)*x + f(5)*y + f(6);
        gv = g(1)*x2           - y2 + g(4)*x + g(5)*y + g(6);
        
        ii = abs(fv)<1e-15 & abs(gv)<1e-15;

        dfdx = 2*f(1)*x + f(2)*y + f(4);
        dfdy =   f(2)*x          + f(5);
        dgdx = 2*g(1)*x          + g(4);
        dgdy =              -2*y + g(5);

        inv_detJ  = 1./(dfdx.*dgdy - dfdy.*dgdx);

        dx = ( dgdy.*fv - dfdy.*gv) .* inv_detJ;
        dy = (-dgdx.*fv + dfdx.*gv) .* inv_detJ;

        dx(ii) = 0;
        dy(ii) = 0;
        
        x = x - dx;
        y = y - dy;
    end

end