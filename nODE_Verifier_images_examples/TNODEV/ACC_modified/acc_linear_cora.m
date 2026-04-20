function dx = acc_linear_cora(x, u) %#ok<INUSD>
% Linear 3rd-order NODE ACC plant, formatted for CORA.
    global w1_acc_lin b1_acc_lin w2_acc_lin b2_acc_lin
    nn_in = [x(3); x(6); x(7); x(8)];
    y = w1_acc_lin * nn_in + b1_acc_lin;         % NO tanh
    nn_out = w2_acc_lin * y + b2_acc_lin;
    dx = [
        x(2);
        x(3);
        nn_out(1);
        x(5);
        x(6);
        nn_out(2);
        nn_out(3);
        nn_out(4)
    ];
end