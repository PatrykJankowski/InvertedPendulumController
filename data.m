% Pendulum
r = 0.0826;
m = 0.027;
g = 9.81;
B_eq = 0;
l_p = 0.191;
J_p = 1.20e-4;

% Engine
R_m = 3.3;
K_t = 0.028;
K_m = K_t;
J_eq = 1.23e-4;
K_w = 2.3;

% Model
a = J_eq + m*r^2;
b = m*l_p*r;
c = m*l_p^2 + J_p;
d = m*g*l_p;
e = K_m*K_w/R_m;
f = (K_m*K_t + B_eq*R_m)/R_m;

% Work point 1 ----------------------------------------
alfa_deg = 0.0001;
alfa_rad = pi*alfa_deg/180;

h = a*c-(b*cos(alfa_rad))^2;
i = b*d*sin(alfa_rad)*cos(alfa_rad)/(alfa_rad*h);
j = -c*f/h;
k = a*d*sin(alfa_rad)/(alfa_rad*h);
l = -b*f*cos(alfa_rad)/h;

n = c*e/h;
o = b*e*cos(alfa_rad)/h;

% Space state model
A1 = [0 0 1 0; 0 0 0 1; 0 i j 0; 0 k l 0];
B1 = [0; 0; n; o];
C = [0 1 0 0];
D = 0;
M1 = [B1 A1*B1 A1^2*B1 A1^3*B1];

% Work point 2 ----------------------------------------

alfa_deg2 = 45;
alfa_rad2 = pi*alfa_deg2/180;

h2 = a*c-(b*cos(alfa_rad2))^2;
i2 = b*d*sin(alfa_rad2)*cos(alfa_rad2)/(alfa_rad2*h2);
j2 = -c*f/h2;
k2 = a*d*sin(alfa_rad2)/(alfa_rad2*h2);
l2 = -b*f*cos(alfa_rad2)/h2;

n2 = c*e/h2;
o2 = b*e*cos(alfa_rad2)/h2;

% Space state model
A2 = [0 0 1 0; 0 0 0 1; 0 i2 j2 0; 0 k2 l2 0];
B2 = [0; 0; n2; o2];
M2 = [B2 A2*B2 A2^2*B2 A2^3*B2];

% Work point 3 ----------------------------------------
alfa_deg3 = 89;
alfa_rad3 = pi*alfa_deg3/180;

h3 = a*c-(b*cos(alfa_rad3))^2;
i3 = b*d*sin(alfa_rad3)*cos(alfa_rad3)/(alfa_rad3*h3);
j3 = -c*f/h3;
k3 = a*d*sin(alfa_rad3)/(alfa_rad3*h3);
l3 = -b*f*cos(alfa_rad3)/h3;

n3 = c*e/h3;
o3 = b*e*cos(alfa_rad3)/h3;

% Space state model
A3 = [0 0 1 0; 0 0 0 1; 0 i3 j3 0; 0 k3 l3 0];
B3 = [0; 0; n3; o3];
M3 = [B3 A3*B3 A3^2*B3 A3^3*B3];

% Work point 4 ----------------------------------------
alfa_deg4 = 135;
alfa_rad4 = pi*alfa_deg4/180;

h4 = a*c-(b*cos(alfa_rad4))^2;
i4 = b*d*sin(alfa_rad4)*cos(alfa_rad4)/(alfa_rad4*h4);
j4 = -c*f/h4;
k4 = a*d*sin(alfa_rad4)/(alfa_rad4*h4);
l4 = -b*f*cos(alfa_rad4)/h4;

n4 = c*e/h4;
o4 = b*e*cos(alfa_rad4)/h4;

% Work point 5 ----------------------------------------
A4 = [0 0 1 0; 0 0 0 1; 0 i4 j4 0; 0 k4 l4 0];
B4 = [0; 0; n4; o4];
M4 = [B4 A4*B4 A4^2*B4 A4^3*B4];


% Work point 6 ----------------------------------------
alfa_deg5 = 179;
alfa_rad5 = pi*alfa_deg5/180;

h5 = a*c-(b*cos(alfa_rad5))^2;
i5 = b*d*sin(alfa_rad5)*cos(alfa_rad5)/(alfa_rad5*h5);
j5 = -c*f/h5;
k5 = a*d*sin(alfa_rad5)/(alfa_rad5*h5);
l5 = -b*f*cos(alfa_rad5)/h5;

n5 = c*e/h5;
o5 = b*e*cos(alfa_rad5)/h5;

% Space state model
A5 = [0 0 1 0; 0 0 0 1; 0 i5 j5 0; 0 k5 l5 0];
B5 = [0; 0; n5; o5];
M5 = [B5 A5*B5 A5^2*B5 A5^3*B5];

% Work point 7 ----------------------------------------
alfa_deg6 = 225;
alfa_rad6 = pi*alfa_deg6/180;

h6 = a*c-(b*cos(alfa_rad6))^2;
i6 = b*d*sin(alfa_rad6)*cos(alfa_rad6)/(alfa_rad6*h6);
j6 = -c*f/h6;
k6 = a*d*sin(alfa_rad6)/(alfa_rad6*h6);
l6 = -b*f*cos(alfa_rad6)/h6;

n6 = c*e/h6;
o6 = b*e*cos(alfa_rad6)/h6;

% Space state model
A6 = [0 0 1 0; 0 0 0 1; 0 i6 j6 0; 0 k6 l6 0];
B6 = [0; 0; n6; o6];
M6 = [B6 A6*B6 A6^2*B6 A6^3*B6];

% Work point 7 ----------------------------------------
alfa_deg7 = 269;
alfa_rad7 = pi*alfa_deg7/180;

h7 = a*c-(b*cos(alfa_rad7))^2;
i7 = b*d*sin(alfa_rad7)*cos(alfa_rad7)/(alfa_rad7*h7);
j7 = -c*f/h7;
k7 = a*d*sin(alfa_rad7)/(alfa_rad7*h7);
l7 = -b*f*cos(alfa_rad7)/h7;

n7 = c*e/h7;
o7 = b*e*cos(alfa_rad7)/h7;

% Space state model
A7 = [0 0 1 0; 0 0 0 1; 0 i7 j7 0; 0 k7 l7 0];
B7 = [0; 0; n7; o7];
M7 = [B7 A7*B7 A7^2*B7 A7^3*B7];


% Work point 8 ----------------------------------------
alfa_deg8 = 315;
alfa_rad8 = pi*alfa_deg8/180;

h8 = a*c-(b*cos(alfa_rad8))^2;
i8 = b*d*sin(alfa_rad8)*cos(alfa_rad8)/(alfa_rad8*h8);
j8 = -c*f/h8;
k8 = a*d*sin(alfa_rad8)/(alfa_rad8*h8);
l8 = -b*f*cos(alfa_rad8)/h8;

n8 = c*e/h8;
o8 = b*e*cos(alfa_rad8)/h8;

% Space state model
A8 = [0 0 1 0; 0 0 0 1; 0 i8 j8 0; 0 k8 l8 0];
B8 = [0; 0; n8; o8];
M8 = [B8 A8*B8 A8^2*B8 A8^3*B8];


% Work point 9 ----------------------------------------
alfa_deg9 = 359;
alfa_rad9 = pi*alfa_deg9/180;

h9 = a*c-(b*cos(alfa_rad9))^2;
i9 = b*d*sin(alfa_rad9)*cos(alfa_rad9)/(alfa_rad9*h9);
j9 = -c*f/h9;
k9 = a*d*sin(alfa_rad9)/(alfa_rad9*h9);
l9 = -b*f*cos(alfa_rad9)/h9;

n9 = c*e/h9;
o9 = b*e*cos(alfa_rad9)/h9;

% Space state model
A9 = [0 0 1 0; 0 0 0 1; 0 i9 j9 0; 0 k9 l9 0];
B9 = [0; 0; n9; o9];
M9 = [B9 A9*B9 A9^2*B9 A9^3*B9];


% LQR -----------------------------------------------
Q = [0.00011 0 0 0; 0 0.0035 0 0; 0 0 0.0000222 0; 0 0 0 0.000335];
R = [0.000133];
N = 0;

F1 = lqr(A1,B1,Q,R,N); F2 = lqr(A2,B2,Q,R,N); F3 = lqr(A3,B3,Q,R,N);
F4 = lqr(A4,B4,Q,R,N); F5 = lqr(A5,B5,Q,R,N); F6 = lqr(A6,B6,Q,R,N);
F7 = lqr(A7,B7,Q,R,N); F8 = lqr(A8,B8,Q,R,N); F9 = lqr(A9,B9,Q,R,N);

% The system
sys = ss(A1,B1,C,D);
system = tf(sys)

% PID parameters
Kp = 100;
Ki = 1;
Kd = 1;

% The feedback with PID controller
V = pid(Kp,Ki,Kd);
T = feedback(system,V)

% The input signal 
t=0:0.01:2;
impulse(T,t)

%The response
title('Response of Pendulum Position to an Impulse Disturbance under PID Control: Kp = 100, Ki = 1, Kd = 1');

initial = pi/2