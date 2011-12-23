function pxux = prob_motion_model(x_t, u_t, x_tm1)
% x_t = [x prime, y prime, theta prime]
% u_t = [velocity, omega]
% x_tm = [x, y, theta]

a = (x_tm1(1) - x_t(1))*cos(x_tm1(3)) + (x_tm1(2) - x_t(2))*sin(x_tm1(3));
b = (x_tm1(2) - x_t(2))*cos(x_tm1(3)) + (x_tm1(1) - x_t(1))*sin(x_tm1(3));
if b == 0
    b = 1E-9;
end
mu = (1/2)*(a/b);

x_s = (1/2)*(x_tm1(1) + x_t(1)) + mu*(x_tm1(2) - x_t(2));
y_s = (1/2)*(x_tm1(2) + x_t(2)) + mu*(-x_tm1(2) + x_t(2));
r_s = sqrt((x_tm1(1) - x_s).^2 + (x_tm1(2) - y_s).^2 );

dth = atan2(x_t(2) - y_s, x_t(2) - x_s) - atan2(x_tm1(2) - y_s, x_tm1(2) - x_s);
dt = 1;

om_hat = dth/dt;
v_hat = om_hat*r_s;
gamma_hat = (x_t(3) - x_tm1(3))/dt;

alpha1 = 1; % Translational error
alpha2 = 1; % Translational error
alpha3 = 1; % Rotational error
alpha4 = 1; % Rotational error
alpha5 = 1; % Rotational error
alpha6 = 1; % Rotational error

pxux = normpdf(u_t(1), v_hat, alpha1*u_t(1).^2 + alpha2*u_t(2).^2)*...
    normpdf(u_t(2), om_hat, alpha3*u_t(1).^2 + alpha4*u_t(2).^2)*...
    normpdf(gamma_hat, om_hat, alpha5*u_t(1).^2 + alpha6*u_t(2).^2);


