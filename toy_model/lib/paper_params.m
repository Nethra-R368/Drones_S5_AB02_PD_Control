function P = paper_params()
%PAPER_PARAMS  Section IV of Golestani et al. (2022) + the choices the paper does not state.
P.J     = [20 1.2 0.9; 1.2 17 1.4; 0.9 1.4 15];   % kg m^2
P.kp    = 40;  P.kd = 20;                          % PD gains
P.alpha = 1.5; P.beta = 1.2;                       % barrier weights
P.eta1  = 0.1;                                     % adaptation gain, eta2 = 2 exp(-0.5 t)
P.umax  = 2.5;                                     % N m per axis
P.rq    = [0.3 0.008 2.5 15];                      % rho_q0, rho_qT, kappa_q, Ts
P.rw    = [0.4 0.005 2.5 15];                      % rho_w0, rho_wT, kappa_w, Ts
P.c     = 0.5;                                     % NOT stated in the paper (c < kd/lambda_max(J) = 0.964)
P.vs    = 0.02;                                    % boundary layer varsigma, NOT stated in the paper
end
