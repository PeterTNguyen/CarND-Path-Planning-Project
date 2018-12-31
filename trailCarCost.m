close all;
clear all;

car_v = 40;
car_trail_v = 0:1:60;

car_d_diff = 10;

t = 30/car_v;

est_d_diff = car_d_diff - t*(car_v - car_trail_v)
##plot(car_trail_v, est_d_diff);




est_d_range = -30:1:100;


buffer = 10;
cost = [];
for est_d = est_d_range
  if(est_d <= buffer)
    cost(end+1) = 1;
  else
    asdf = est_d - buffer;
    cost(end+1) = (exp(-asdf/5));
  endif
endfor

figure;
plot(est_d_range, cost);
title('Trailing Car Cost function');
xlabel('Estimated Trail/Lead Car distance');ylabel('Cost');grid on;
hold on;
plot([buffer buffer], [0 1],'--');
legend('Cost', 'Buffer zone');
