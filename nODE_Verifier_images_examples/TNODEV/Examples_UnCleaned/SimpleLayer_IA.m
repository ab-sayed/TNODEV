%% V 1.0

% classdef SimpleLayer_IA
%     % SimpleLayer_IA: A library of Interval Arithmetic layers for Neural Networks.
%     % Used to replace NNV dependencies.
%     % Logic: Computes Output Center and Radius given Input Center and Radius.
% 
%     methods(Static)
% 
%         function [y_c, y_r] = Conv2D(x_c, x_r, Weights, Bias, stride, padding)
%             % Interval Convolution: 
%             % Center = Conv(x_c, W) + b
%             % Radius = Conv(x_r, |W|)
% 
%             % 1. Prepare Bias (reshape to 1x1xChannels)
%             num_filters = size(Weights, 4);
%             Bias = reshape(Bias, 1, 1, num_filters);
% 
%             % 2. Compute Center (Standard Convolution)
%             % MATLAB's convn is usually for 3D, we need specific 4D mapping or loop
%             % Since this is a tiny network, a loop over filters is safe and clear.
% 
%             [H, W, C_in] = size(x_c);
%             [kH, kW, ~, ~] = size(Weights);
% 
%             % Calculate output size (assuming valid/same logic or fixed for MNIST)
%             % For 'stride' 1 and 'valid' padding (no padding):
%             H_out = (H - kH + 2*padding) / stride + 1;
%             W_out = (W - kW + 2*padding) / stride + 1;
% 
%             y_c = zeros(H_out, W_out, num_filters);
%             y_r = zeros(H_out, W_out, num_filters);
% 
%             % Absolute Weights for Radius calculation
%             AbsWeights = abs(Weights);
% 
%             for f = 1:num_filters
%                 % Get filter kernel and bias
%                 kernel = Weights(:, :, :, f);
%                 abs_kernel = AbsWeights(:, :, :, f);
% 
%                 % Compute Center: convn(x, kernel, 'valid')
%                 % Note: We must rotate kernel for true convolution vs correlation, 
%                 % but standard DL libs usually do correlation. We stick to DL standard.
%                 % For simplicity in MATLAB without toolboxes, we iterate channels.
% 
%                 for c = 1:C_in
%                     % Extract 2D slices
%                     xc_slice = x_c(:,:,c);
%                     xr_slice = x_r(:,:,c);
%                     w_slice = kernel(:,:,c);
%                     aw_slice = abs_kernel(:,:,c);
% 
%                     % 2D Convolution (Correlation)
%                     % Using 'conv2' with 'rot90(w,2)' makes it a convolution.
%                     % But standard NN weights are usually stored for correlation.
%                     % We assume correlation (filter2).
% 
%                     term_c = filter2(rot90(w_slice,2), xc_slice, 'valid'); 
%                     term_r = filter2(rot90(aw_slice,2), xr_slice, 'valid');
% 
%                     % Accumulate across input channels
%                     y_c(:,:,f) = y_c(:,:,f) + term_c;
%                     y_r(:,:,f) = y_r(:,:,f) + term_r;
%                 end
% 
%                 % Add Bias to Center
%                 y_c(:,:,f) = y_c(:,:,f) + Bias(f);
%             end
%         end
% 
%         function [y_c, y_r] = BatchNorm(x_c, x_r, Offset, Scale, Mean, Var, Epsilon)
%             % Interval BN:
%             % y = Scale * (x - Mean) / sqrt(Var + Eps) + Offset
%             % This is an element-wise linear transformation: y = a*x + b
% 
%             % Calculate effective Weight (a) and Bias (b)
%             std = sqrt(Var + Epsilon);
%             a = Scale ./ std;
%             b = Offset - (Scale .* Mean ./ std);
% 
%             % Reshape for broadcasting (1x1xChannels)
%             n_ch = length(Offset);
%             a = reshape(a, 1, 1, n_ch);
%             b = reshape(b, 1, 1, n_ch);
% 
%             % Interval Math
%             % Center: a * c + b
%             % Radius: |a| * r
%             y_c = x_c .* a + b;
%             y_r = x_r .* abs(a);
%         end
% 
%         function [y_c, y_r] = ReLU(x_c, x_r)
%             % Interval ReLU: [max(0, lb), max(0, ub)]
%             lb = x_c - x_r;
%             ub = x_c + x_r;
% 
%             y_lb = max(0, lb);
%             y_ub = max(0, ub);
% 
%             y_c = 0.5 * (y_lb + y_ub);
%             y_r = 0.5 * (y_ub - y_lb);
%         end
% 
%         function [y_c, y_r] = Flatten(x_c, x_r)
%             y_c = x_c(:);
%             y_r = x_r(:);
%         end
% 
%         function [y_c, y_r] = FullyConnected(x_c, x_r, Weights, Bias)
%             % Dense Layer: y = Wx + b
%             % Center = W * xc + b
%             % Radius = |W| * xr
% 
%             y_c = Weights * x_c + Bias;
%             y_r = abs(Weights) * x_r;
%         end
%     end
% end

%% V 1.1

classdef SimpleLayer_IA
    % SimpleLayer_IA: A library of Interval Arithmetic layers for Neural Networks.
    % V2: 
    % Fixed Stride handling for Conv2D.
    % Adds Padding support + Stride logic
    % Robust Dimension Handling
    
    methods(Static)
        
        function [y_c, y_r] = Conv2D(x_c, x_r, Weights, Bias, stride, padding)
                        
            % 1. Apply Padding (If needed)
            if padding > 0
                x_c = padarray(x_c, [padding, padding], 0);
                x_r = padarray(x_r, [padding, padding], 0);
            end

            % 2. Prepare Dimensions

            % num_filters = size(Weights, 4);
            % Bias = reshape(Bias, 1, 1, num_filters);
            
            [H, W, C_in] = size(x_c);   % Dimensions after Padding
            [kH, kW, Ch_in_W, num_filters] = size(Weights);

            % Sanity Check: Ensure weights are [H, W, In, Out]
            % If the 3rd dimension of Weights doesn't match Input Channels,
            % the weights might be permuted wrong.
            if Ch_in_W ~= C_in
                % Try to auto-fix common permutation issues [Out, In, H, W] -> [H, W, In, Out]
                if size(Weights, 2) == C_in
                    % warning('SimpleLayer_IA: Auto-permuting weights from [Out, In, H, W] to [H, W, In, Out]');
                    Weights = permute(Weights, [3 4 2 1]);
                    [kH, kW, ~, num_filters] = size(Weights);
                end
            end

            Bias = reshape(Bias, 1, 1, num_filters);
            AbsWeights = abs(Weights);
            
            % % Calculate output size
            % % floor is used here as a standard conv. behavior
            % H_out = floor((H - kH + 2*padding) / stride) + 1;
            % W_out = floor((W - kW + 2*padding) / stride) + 1;
            % 
            % y_c = zeros(H_out, W_out, num_filters);
            % y_r = zeros(H_out, W_out, num_filters);
            
            % AbsWeights = abs(Weights);

            % Prepare Output (Now we don't allocate till we know the stride result size)
            y_c = []; y_r = [];
            
            for f = 1:num_filters
                kernel = Weights(:, :, :, f);
                abs_kernel = AbsWeights(:, :, :, f);
                
                % Accumulators (Full convolution size)
                % valid conv. output size is  `H-kH+1` 
                term_c_accum = zeros(H - kH + 1, W - kW + 1);
                term_r_accum = zeros(H - kH + 1, W - kW + 1);
                
                for c = 1:C_in
                    xc_slice = x_c(:,:,c);
                    xr_slice = x_r(:,:,c);
                    w_slice = kernel(:,:,c);
                    aw_slice = abs_kernel(:,:,c);
                    
                    % Compute convolution 
                    % term_c_accum = term_c_accum + filter2(rot90(w_slice,2), xc_slice, 'valid'); 
                    % term_r_accum = term_r_accum + filter2(rot90(aw_slice,2), xr_slice, 'valid');
                    
                    % Use Correlation (Standard for CNNs)
                    term_c_accum = term_c_accum + filter2(w_slice, xc_slice, 'valid'); 
                    term_r_accum = term_r_accum + filter2(aw_slice, xr_slice, 'valid');
                end
                
                % 3. Apply Stride (Downsample) 
                term_c_strided = term_c_accum(1:stride:end, 1:stride:end);
                term_r_strided = term_r_accum(1:stride:end, 1:stride:end);
                
                % 4. Initialize Output on the 1st iteration (to match the actual strided size)
                if isempty(y_c)
                    [actual_H, actual_W] = size(term_c_strided);
                    y_c = zeros(actual_H, actual_W, num_filters);
                    y_r = zeros(actual_H, actual_W, num_filters);
                end
                 
                % % Trim to expected size (this can handle potential size mismatches)
                % term_c_strided = term_c_strided(1:H_out, 1:W_out);
                % term_r_strided = term_r_strided(1:H_out, 1:W_out);
                
                % Add Bias
                y_c(:,:,f) = term_c_strided + Bias(f);
                y_r(:,:,f) = term_r_strided;
            end
        end
        
        function [y_c, y_r] = BatchNorm(x_c, x_r, Offset, Scale, Mean, Var, Epsilon)
            % Interval BN
            std = sqrt(Var + Epsilon);
            a = Scale ./ std;
            b = Offset - (Scale .* Mean ./ std);
            
            n_ch = length(Offset);
            a = reshape(a, 1, 1, n_ch);
            b = reshape(b, 1, 1, n_ch);
            
            y_c = x_c .* a + b;
            y_r = x_r .* abs(a);
        end
        
        function [y_c, y_r] = ReLU(x_c, x_r)
            % Interval ReLU
            lb = x_c - x_r;
            ub = x_c + x_r;
            
            y_lb = max(0, lb);
            y_ub = max(0, ub);
            
            y_c = 0.5 * (y_lb + y_ub);
            y_r = 0.5 * (y_ub - y_lb);
        end
        
        function [y_c, y_r] = Flatten(x_c, x_r)
            y_c = x_c(:);
            y_r = x_r(:);
        end
        
        function [y_c, y_r] = FullyConnected(x_c, x_r, Weights, Bias)
            y_c = Weights * x_c + Bias;
            y_r = abs(Weights) * x_r;
        end
    end
end
