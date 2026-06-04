% =========================================================================
% File: CartpoleCTRNN.m
%
% Purpose:
%   12-dimensional cartpole benchmark with a CT-RNN neural-network
%   controller, written as a continuous-time vector field for use with
%   the TNODEV reachability pipeline.
%
% Description:
%   State layout (12D):
%     x_00, x_01, x_02, x_03 : cart position, cart velocity, pole angle, pole rate
%     h_00 .. h_07           : 8 CT-RNN hidden states (each with its own
%                              tanh activation and recurrent weights)
%
%   The hidden states evolve as
%       h_i' = -h_i + tanh( recurrent + input + bias )
%   and are then mapped through a readout tanh into a single scalar
%   action that drives the cart through the standard cartpole physics
%   equations (gravity, masscart, masspole, length, force_mag).
%
%   The Python source from which this MATLAB function was ported is kept
%   below in commented form for reference / provenance.
%
% Inputs:
%   x : 12x1 state vector
%   u : input (unused; kept for solver-signature compatibility)
%
% Output:
%   dx : 12x1 state derivative
%
% Part of the TNODEV Toolbox
% Tool Paper: "Add Title here"
%
% Authors:
%   Abdelrahman Sayed Sayed   <abdelrahman.ibrahim -AT- univ-eiffel.fr>
%   Pierre-Jean Meyer         <pierre-jean.meyer    -AT- univ-eiffel.fr>
%   Mohamed Ghazel            <mohamed.ghazel       -AT- univ-eiffel.fr>
%   COSYS-ESTAS, Univ Gustave Eiffel
%
% License: CC BY-NC-SA 4.0
% =========================================================================

%------------- BEGIN CODE --------------
function dx = CartpoleCTRNN(x, u) %#ok<INUSD>

% ----- Original Python reference (kept for provenance) ---------------------
% # 12-dimensional cartpole with CT-RNN neural network controller
% class CartpoleCTRNN:
%     def __init__(self, radius):
%         # ============ adapt initial values ===========
%         self.cx = (0, 0, 0.001, 0, 0, 0, 0, 0, 0, 0, 0, 0)
%         if radius is not None:
%             self.rad = radius
%         else:
%             self.rad = 1e-4
%         # ===================================================
%         self.cx = np.array(self.cx, dtype=float)
%         self.dim = self.cx.size  # dimension of the system
%
%     def fdyn(self, t=0, x=None):
%         if x is None:
%             x = np.zeros(self.dim, dtype=object)
%
%         # ============ adapt input and system dynamics ===========
%         x_00, x_01, x_02, x_03, h_00, h_01, h_02, h_03, h_04, h_05, h_06, h_07 = x
%
%         h_00_prime = -h_00 + tanh(
%             h_00 * -1.49394
%             + h_01 * -0.61947
%             + h_02 * 0.37393
%             + h_03 * -0.63451
%             + h_04 * -1.08420
%             + h_05 * 2.57981
%             + h_06 * -1.53850
%             + h_07 * -1.64354
%             + x_00 * -0.07445
%             + x_01 * 0.08736
%             + x_02 * 0.47684
%             + x_03 * 0.56397
%             + 1.81867
%         )
%         h_01_prime = -h_01 + tanh(
%             h_00 * 1.38295
%             + h_01 * -1.45811
%             + h_02 * 1.01473
%             + h_03 * -0.04578
%             + h_04 * -0.13416
%             + h_05 * -0.21970
%             + h_06 * 0.41791
%             + h_07 * -0.10833
%             + x_00 * -1.70409
%             + x_01 * 0.51560
%             + x_02 * -0.71273
%             + x_03 * -0.61720
%             + 0.86009
%         )
%         h_02_prime = -h_02 + tanh(
%             h_00 * 0.57055
%             + h_01 * 0.05941
%             + h_02 * -0.16993
%             + h_03 * -0.69688
%             + h_04 * -0.30939
%             + h_05 * -1.31558
%             + h_06 * 0.03316
%             + h_07 * 2.35873
%             + x_00 * 0.21849
%             + x_01 * 1.42990
%             + x_02 * 1.60666
%             + x_03 * -0.66847
%             + -1.56112
%         )
%         h_03_prime = -h_03 + tanh(
%             h_00 * -1.53087
%             + h_01 * -0.42779
%             + h_02 * -0.02195
%             + h_03 * 0.18007
%             + h_04 * 1.54262
%             + h_05 * -0.19275
%             + h_06 * -0.64598
%             + h_07 * -0.85840
%             + x_00 * 0.10095
%             + x_01 * 0.55012
%             + x_02 * 1.51416
%             + x_03 * 1.95952
%             + 0.00020
%         )
%         h_04_prime = -h_04 + tanh(
%             h_00 * -0.54493
%             + h_01 * 0.41139
%             + h_02 * -0.18790
%             + h_03 * -0.14312
%             + h_04 * -0.10144
%             + h_05 * 2.38792
%             + h_06 * -0.44134
%             + h_07 * -2.06462
%             + x_00 * -0.01260
%             + x_01 * 0.84681
%             + x_02 * 0.51889
%             + x_03 * 0.35089
%             + -0.06552
%         )
%         h_05_prime = -h_05 + tanh(
%             h_00 * 1.08793
%             + h_01 * -0.24559
%             + h_02 * -0.97041
%             + h_03 * -0.16874
%             + h_04 * -0.27133
%             + h_05 * 0.73513
%             + h_06 * 0.73096
%             + h_07 * -2.26244
%             + x_00 * 0.80312
%             + x_01 * -1.35635
%             + x_02 * -0.95283
%             + x_03 * -0.68460
%             + 0.11646
%         )
%         h_06_prime = -h_06 + tanh(
%             h_00 * 0.08745
%             + h_01 * 0.10104
%             + h_02 * 0.82251
%             + h_03 * -0.59823
%             + h_04 * 1.16949
%             + h_05 * 1.73511
%             + h_06 * -0.89387
%             + h_07 * 0.77149
%             + x_00 * 0.22731
%             + x_01 * -0.73872
%             + x_02 * 2.83509
%             + x_03 * -2.49535
%             + 0.70343
%         )
%         h_07_prime = -h_07 + tanh(
%             h_00 * -0.89045
%             + h_01 * -1.30985
%             + h_02 * 0.48740
%             + h_03 * -0.45750
%             + h_04 * -0.70727
%             + h_05 * -0.43216
%             + h_06 * -0.29159
%             + h_07 * 3.70331
%             + x_00 * -0.13294
%             + x_01 * -0.23242
%             + x_02 * 0.33244
%             + x_03 * -1.45838
%             + -0.02392
%         )
%
%         action_00 = tanh(
%             h_00 * -0.32011
%             + h_01 * 0.32958
%             + h_02 * 0.07718
%             + h_03 * 2.39431
%             + h_04 * -1.37732
%             + h_05 * 0.92722
%             + h_06 * -1.07137
%             + h_07 * -1.47286
%         )
%
%         gravity = 9.8
%         masscart = 1
%         masspole = 0.1
%         total_mass = masscart + masspole
%         length = 0.5
%         polemass_length = masspole * length
%         force_mag = 10
%
%         force = force_mag * action_00
%         costheta = cos(x_02)
%         sintheta = sin(x_02)
%         x_dot = x_01
%         theta_dot = x_03
%
%         temp = (force + polemass_length * theta_dot * theta_dot * sintheta) / total_mass
%         thetaacc = (gravity * sintheta - costheta * temp) / (
%             length * (4.0 / 3.0 - masspole * costheta * costheta / total_mass)
%         )
%         xacc = temp - polemass_length * thetaacc * costheta / total_mass
%
%         x_00_prime = x_dot
%         x_01_prime = xacc
%         x_02_prime = theta_dot
%         x_03_prime = thetaacc
%
%         system_dynamics = [
%             x_00_prime,
%             x_01_prime,
%             x_02_prime,
%             x_03_prime,
%             h_00_prime,
%             h_01_prime,
%             h_02_prime,
%             h_03_prime,
%             h_04_prime,
%             h_05_prime,
%             h_06_prime,
%             h_07_prime,
%         ]  # has to be in the same order as the input variables
%         # ===================================================
%
%         return np.array(system_dynamics)  # return as numpy array
% ----- end of Python reference --------------------------------------------


    % --- Declare state variables -----------------------------------------
    x_00 = x(1);  x_01 = x(2);  x_02 = x(3);  x_03 = x(4);
    h_00 = x(5);  h_01 = x(6);  h_02 = x(7);  h_03 = x(8);
    h_04 = x(9);  h_05 = x(10); h_06 = x(11); h_07 = x(12);


    % --- CT-RNN hidden-state derivatives  h_i' = -h_i + tanh(.) ----------
    h_00_prime = -h_00 + tanh( ...
        h_00 * -1.49394 ...
        + h_01 * -0.61947 ...
        + h_02 *  0.37393 ...
        + h_03 * -0.63451 ...
        + h_04 * -1.08420 ...
        + h_05 *  2.57981 ...
        + h_06 * -1.53850 ...
        + h_07 * -1.64354 ...
        + x_00 * -0.07445 ...
        + x_01 *  0.08736 ...
        + x_02 *  0.47684 ...
        + x_03 *  0.56397 ...
        + 1.81867);

    h_01_prime = -h_01 + tanh( ...
        h_00 *  1.38295 ...
        + h_01 * -1.45811 ...
        + h_02 *  1.01473 ...
        + h_03 * -0.04578 ...
        + h_04 * -0.13416 ...
        + h_05 * -0.21970 ...
        + h_06 *  0.41791 ...
        + h_07 * -0.10833 ...
        + x_00 * -1.70409 ...
        + x_01 *  0.51560 ...
        + x_02 * -0.71273 ...
        + x_03 * -0.61720 ...
        + 0.86009);

    h_02_prime = -h_02 + tanh( ...
        h_00 *  0.57055 ...
        + h_01 *  0.05941 ...
        + h_02 * -0.16993 ...
        + h_03 * -0.69688 ...
        + h_04 * -0.30939 ...
        + h_05 * -1.31558 ...
        + h_06 *  0.03316 ...
        + h_07 *  2.35873 ...
        + x_00 *  0.21849 ...
        + x_01 *  1.42990 ...
        + x_02 *  1.60666 ...
        + x_03 * -0.66847 ...
        + -1.56112);

    h_03_prime = -h_03 + tanh( ...
        h_00 * -1.53087 ...
        + h_01 * -0.42779 ...
        + h_02 * -0.02195 ...
        + h_03 *  0.18007 ...
        + h_04 *  1.54262 ...
        + h_05 * -0.19275 ...
        + h_06 * -0.64598 ...
        + h_07 * -0.85840 ...
        + x_00 *  0.10095 ...
        + x_01 *  0.55012 ...
        + x_02 *  1.51416 ...
        + x_03 *  1.95952 ...
        + 0.00020);

    h_04_prime = -h_04 + tanh( ...
        h_00 * -0.54493 ...
        + h_01 *  0.41139 ...
        + h_02 * -0.18790 ...
        + h_03 * -0.14312 ...
        + h_04 * -0.10144 ...
        + h_05 *  2.38792 ...
        + h_06 * -0.44134 ...
        + h_07 * -2.06462 ...
        + x_00 * -0.01260 ...
        + x_01 *  0.84681 ...
        + x_02 *  0.51889 ...
        + x_03 *  0.35089 ...
        + -0.06552);

    h_05_prime = -h_05 + tanh( ...
        h_00 *  1.08793 ...
        + h_01 * -0.24559 ...
        + h_02 * -0.97041 ...
        + h_03 * -0.16874 ...
        + h_04 * -0.27133 ...
        + h_05 *  0.73513 ...
        + h_06 *  0.73096 ...
        + h_07 * -2.26244 ...
        + x_00 *  0.80312 ...
        + x_01 * -1.35635 ...
        + x_02 * -0.95283 ...
        + x_03 * -0.68460 ...
        + 0.11646);

    h_06_prime = -h_06 + tanh( ...
        h_00 *  0.08745 ...
        + h_01 *  0.10104 ...
        + h_02 *  0.82251 ...
        + h_03 * -0.59823 ...
        + h_04 *  1.16949 ...
        + h_05 *  1.73511 ...
        + h_06 * -0.89387 ...
        + h_07 *  0.77149 ...
        + x_00 *  0.22731 ...
        + x_01 * -0.73872 ...
        + x_02 *  2.83509 ...
        + x_03 * -2.49535 ...
        + 0.70343);

    h_07_prime = -h_07 + tanh( ...
        h_00 * -0.89045 ...
        + h_01 * -1.30985 ...
        + h_02 *  0.48740 ...
        + h_03 * -0.45750 ...
        + h_04 * -0.70727 ...
        + h_05 * -0.43216 ...
        + h_06 * -0.29159 ...
        + h_07 *  3.70331 ...
        + x_00 * -0.13294 ...
        + x_01 * -0.23242 ...
        + x_02 *  0.33244 ...
        + x_03 * -1.45838 ...
        + -0.02392);


    % --- Readout: scalar action from the 8 hidden states ----------------
    action_00 = tanh( ...
        h_00 * -0.32011 ...
        + h_01 *  0.32958 ...
        + h_02 *  0.07718 ...
        + h_03 *  2.39431 ...
        + h_04 * -1.37732 ...
        + h_05 *  0.92722 ...
        + h_06 * -1.07137 ...
        + h_07 * -1.47286);


    % --- Standard cartpole physics --------------------------------------
    gravity         = 9.8;
    masscart        = 1;
    masspole        = 0.1;
    total_mass      = masscart + masspole;
    length          = 0.5;
    polemass_length = masspole * length;
    force_mag       = 10;

    force     = force_mag * action_00;
    costheta  = cos(x_02);
    sintheta  = sin(x_02);
    x_dot     = x_01;
    theta_dot = x_03;

    temp     = (force + polemass_length * theta_dot * theta_dot * sintheta) / total_mass;
    thetaacc = (gravity * sintheta - costheta * temp) / ...
               (length * (4.0/3.0 - masspole * costheta * costheta / total_mass));
    xacc     = temp - polemass_length * thetaacc * costheta / total_mass;


    % --- Pack the 12D derivative vector ---------------------------------
    x_00_prime = x_dot;
    x_01_prime = xacc;
    x_02_prime = theta_dot;
    x_03_prime = thetaacc;

    dx = [x_00_prime;
          x_01_prime;
          x_02_prime;
          x_03_prime;
          h_00_prime;
          h_01_prime;
          h_02_prime;
          h_03_prime;
          h_04_prime;
          h_05_prime;
          h_06_prime;
          h_07_prime];

end
%------------- END OF CODE --------------
