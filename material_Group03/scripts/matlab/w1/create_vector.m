function [u, v] = create_vector(path, x, y)
    d = find_distances(path, x, y);
    d(length(d)+1) = 10e7;
    [M,I] = min_custom(d);

    x2 = path(I+1,:);
    x1 = path(I,:);
    vline = x2-x1;
    p = x1 + vline*(([x y]-x1)*vline'/(vline*vline'));
    s = ([x y]-x1)*vline'/(vline*vline');
    vnorm = p - [x y];
        
    ktg = 1;
    kn = (d(I)/0.10)^2*0.3;

    if norm(vnorm)~=0
        headingvec = vline/norm(vline)*ktg+vnorm/norm(vnorm)*kn;
    else
        headingvec = vline/norm(vline)*ktg;
    end
    
    dp1 = norm(vline)*(1-s); 
    dp2 = 0;
    headingvec2 = [0 0];
    % If close to end, blend with next segment
    if ((norm(vline)*(1-s)) < 0.5) && (I < (length(path(:,1))-1))
        x2 = path(I+2,:);
        x1 = path(I+1,:);
        vline = x2-x1;
        p = x1 + vline*(([x y]-x1)*vline'/(vline*vline'));
        s = ([x y]-x1)*vline'/(vline*vline');
        vnorm = p - [x y];
        if norm(vnorm)~=0
            headingvec2 = vline/norm(vline)*ktg+vnorm/norm(vnorm)*kn;
        else
            headingvec2 = vline/norm(vline)*ktg;
        end
        dp2 = norm(vline)*s; 
    end
    dbias = 0.01;
    w1 = dp1; w2 = 0.5-dp1;
    %w1 = (1-s)*norm(vline); w2 = 1; 
    %sumw = w1+w2;
    %w1 = w1/sumw; w2 = w2/sumw;

    u = headingvec(1)*w1 + headingvec2(1)*w2;
    v = headingvec(2)*w1 + headingvec2(2)*w2;

    normuv = norm([u v]);
    u = u/normuv;
    v = v/normuv;
end