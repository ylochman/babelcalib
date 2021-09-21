% Copyright (c) 2011, Laurent Kneip, ETH Zurich
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%     * Redistributions of source code must retain the above copyright
%       notice, this list of conditions and the following disclaimer.
%     * Redistributions in binary form must reproduce the above copyright
%       notice, this list of conditions and the following disclaimer in the
%       documentation and/or other materials provided with the distribution.
%     * Neither the name of ETH Zurich nor the
%       names of its contributors may be used to endorse or promote products
%       derived from this software without specific prior written permission.
% 
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
% ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
% WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
% DISCLAIMED. IN NO EVENT SHALL ETH ZURICH BE LIABLE FOR ANY
% DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
% (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
% ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
% (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function sols = solveQuartic( factors )

    A = factors(1);
    B = factors(2);
    C = factors(3);
    D = factors(4);
    E = factors(5);
    
    A_pw2 = A*A;
    B_pw2 = B*B;
    A_pw3 = A_pw2*A;
    B_pw3 = B_pw2*B;
    A_pw4 = A_pw3*A;
    B_pw4 = B_pw3*B;
    
    alpha = -3*B_pw2/(8*A_pw2)+C/A;
    beta = B_pw3/(8*A_pw3)-B*C/(2*A_pw2)+D/A;
    gamma = -3*B_pw4/(256*A_pw4)+B_pw2*C/(16*A_pw3)-B*D/(4*A_pw2)+E/A;
    
    alpha_pw2 = alpha*alpha;
    alpha_pw3 = alpha_pw2*alpha;
    
    P = -alpha_pw2/12-gamma;
    Q = -alpha_pw3/108+alpha*gamma/3-beta^2/8;
    R = -Q/2+sqrt(Q^2/4+P^3/27);
    U = R^(1/3);
    
    if U == 0
        y = -5*alpha/6-Q^(1/3);
    else
        y = -5*alpha/6-P/(3*U)+U;
    end
    
    w = sqrt(alpha+2*y);
    
    sols = zeros(4,1);
    sols(1) = -B/(4*A) + 0.5*( w+sqrt(-(3*alpha+2*y+2*beta/w)));
    sols(2) = -B/(4*A) + 0.5*( w-sqrt(-(3*alpha+2*y+2*beta/w)));
    sols(3) = -B/(4*A) + 0.5*(-w+sqrt(-(3*alpha+2*y-2*beta/w)));
    sols(4) = -B/(4*A) + 0.5*(-w-sqrt(-(3*alpha+2*y-2*beta/w)));
    
end