T = 5;
S = [0 50 0 20 0 1];

A = [[T^3 T^4 T^5],
[3*T^2 4*T^3 5*T^4],
[6*T 12*T^2 20*T^3]];

B = [S(2) - (S(1) + S(3)*T + 0.5*S(5)*T*T),
     S(4) - (S(3) + S(5)*T),
     S(6) - S(5)];
     
C = inverse(A)*B;

t = 0:0.02:T;
s = C(1)*t.^3 + C(2)*t.^4 + C(3)*t.^5;
s1 = 3*C(1)*t.^2 + 4*C(2)*t.^3 + 5*C(3)*t.^4;
s2 = 6*C(1)*t + 12*C(2)*t.^2 + 20*C(3)*t.^3;
s3 = 6*C(1)   + 24*C(2)*t    + 60*C(3);
plot(t, s, t, s1, t, s2, t, s3); grid on;
legend('Distance','Velocity','Acceleration','Jerk'); 
xlabel('Time [s]'); ylabel('Motion Variables');title('JMT');