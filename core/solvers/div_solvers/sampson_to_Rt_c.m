function [F, c, H, R, t] = sampson_to_Rt_c(F0, x, X)
    opt = optimoptions('lsqnonlin','Display','off',...
                       'MaxIter',100);
    [U,s,V] = svd(F0');
    c0(1,1) = V(1,end)./V(3,end);
    c0(2,1) = V(2,end)./V(3,end);
    f0 = reshape(F0',[],1);
    h0(1:3) = f0(4:6);
    h0(4:6) = -f0(1:3);
    h0 = h0./h0(4);
    params0 = [c0' h0([1:3 5:6])];
    [params, ~, errs] = lsqnonlin(@sampson_err_cH, params0, [],[], opt, X, x);
    c = params(1:2)';
    h = params(3:end);
    h = [h(1:3) 1 h(4:5)];
    ex = sksym([c;1]);
    F = ex * [reshape(h,3,2)'; 0 0 0];
    h = h';

    H = reshape(h,3,2)';
    if nargout >= 4
        r11 = h(1);
        r12 = h(2);
        r21 = h(4);
        r22 = h(5);
        A = r11.*r21 + r12.*r22;
        B = r11.^2 - r21.^2 + r12.^2 - r22.^2;
        r23 = [sqrt((B - sqrt(B.^2+4*A.^2))/2)...
            -sqrt((B - sqrt(B.^2+4*A.^2))/2)...
            sqrt((B + sqrt(B.^2+4*A.^2))/2)...
            -sqrt((B + sqrt(B.^2+4*A.^2))/2)];
        r13 = - repmat(A,1,4) ./ r23;

        r11 = repmat(r11,1,4);
        r12 = repmat(r12,1,4);
        r21 = repmat(r21,1,4);
        r22 = repmat(r22,1,4);
        c = repmat(c,1,4);
        h = repmat(h,1,4);

        realidx = abs(imag(r23))<1e-7;
        r11 = r11(realidx);
        r12 = r12(realidx);
        r21 = r21(realidx);
        r22 = r22(realidx);
        r13 = real(r13(realidx));
        r23 = real(r23(realidx));
        c = c(:,realidx);
        h = h(:,realidx);

        t = [h(3,:); h(6,:)];

        s1 = 1./sqrt(r11.^2 + r12.^2 + r13.^2);
        s2 = -1./sqrt(r11.^2 + r12.^2 + r13.^2);

        r11 = [r11.*s1 r11.*s2];
        r12 = [r12.*s1 r12.*s2];
        r13 = [r13.*s1 r13.*s2];
        r21 = [r21.*s1 r21.*s2];
        r22 = [r22.*s1 r22.*s2];
        r23 = [r23.*s1 r23.*s2];
        t = [t.*s1 t.*s2];
        c = repmat(c,1,2);

        r1 = [r11' r12' r13'];
        r2 = [r21' r22' r23'];
        r3 = cross(r1',r2')';

        R = [reshape(r1',1,3,[]);...
            reshape(r2',1,3,[]);...
            reshape(r3',1,3,[])];
    end
end