% Tool Paper:
%   "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed, <abdelrahman.ibrahim -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Pierre-Jean Meyer, <pierre-jean.meyer -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%   Mohamed Ghazel, <mohamed.ghazel -AT- univ-eiffel.fr>, COSYS-ESTAS, Univ Gustave Eiffel
%
% Date: 10 March 2026
% Last update: 11 March 2026 
% Last revision: XX XX 2026

%------------- BEGIN CODE --------------
function dx = acc_node_cora(x, u)
% acc_node_cora.m 
% Continuous dynamics of the 8D ACC Neural ODE system formatted for CORA.

    global w1_acc b1_acc w2_acc b2_acc

    % Extract the specific states feeding into the Neural Network
    % We must use concatenation rather than indexing array x directly with 
    % an array like x([3,6,7,8]) because CORA's overloaded zonotope objects
    % sometimes fail with multi-index arrays.
    nn_in = [x(3); x(6); x(7); x(8)];
    
    % Forward pass through the 1-hidden-layer Neural Network
    z = w1_acc * nn_in + b1_acc;
    h = tanh(z);
    nn_out = w2_acc * h + b2_acc;
    
    % Assemble the 8D derivative vector
    dx = [
        x(2);         % dx1 = x2 (Lead velocity)
        x(3);         % dx2 = x3 (Lead acceleration)
        nn_out(1);    % dx3 = NN output 1
        x(5);         % dx4 = x5 (Ego velocity)
        x(6);         % dx5 = x6 (Ego acceleration)
        nn_out(2);    % dx6 = NN output 2
        nn_out(3);    % dx7 = NN output 3
        nn_out(4)     % dx8 = NN output 4
    ];
end
%------------- END OF CODE --------------