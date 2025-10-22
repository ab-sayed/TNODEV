function varargout = ndims_idx(varargin)
    % Generate all combinations of indices for n-dimensional grid
    % Input: varargin, cell array of index vectors for each dimension
    % Output: varargout, arrays of indices for each dimension
    args = varargin;
    n = length(args);
    [f{1:n}] = ndimsdeal(args{:});
    varargout = cellfun(@(x) x(:), f, 'UniformOutput', false);
end

function varargout = ndimsdeal(varargin)
    % Helper function to deal indices into multiple outputs
    args = varargin;
    n = length(args);
    sz = cellfun(@numel, args);
    [g{1:n}] = ndims_grid(sz);
    for i = 1:n
        varargout{i} = args{i}(g{i});
    end
end

function varargout = ndims_grid(sz)
    % Generate grid indices for n-dimensional array
    n = length(sz);
    idx = cell(1, n);
    [idx{1:n}] = ind2sub(sz, 1:prod(sz));
    for i = 1:n
        varargout{i} = idx{i};
    end
end