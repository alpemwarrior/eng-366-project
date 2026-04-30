function [M, I] = min_custom(A)
    M = A(1);
    I = 1;
    for i=1:length(A)
        v = A(i);
        if v<=M
            M = v;
            I = i;
        end
    end
end