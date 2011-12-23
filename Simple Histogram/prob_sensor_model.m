function pzx = prob_sensor_model(z_t, x_t, landmark)

   sigma = 1;
   MU = sqrt((x_t(1) - landmark(1)).^2 + (x_t(2) - landmark(2)).^2);
   pzx = normpdf(z_t, MU, sigma);