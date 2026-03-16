function is_inside = interval_is_subset(A, B)
% Checks if interval A is completely inside interval B
% A and B are interval objects with .inf and .sup fields
    is_inside = all(A.inf >= B.inf) && all(A.sup <= B.sup);
end