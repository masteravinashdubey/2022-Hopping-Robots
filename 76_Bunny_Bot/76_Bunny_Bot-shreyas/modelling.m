pkg load control;

mw = 0.04;
Iw = 0.0005;
l = 0.26;
theta = 0;
yaw = 0;
r = 0.04;
d = 0.1;
g = 9.8;

mb = 1.5;
Iy = 0.5*mb*l*l;
Iz = 0.0027;
m1 = 1.5;
m2 = 0.05;
m3 = 0.05;
l1 = 0.14;
l2 = 0.08;
l3 = 0.01;

deno = 2*Iw*(Iy+mb*l*l) + (2*l*l*mb*mw + Iy*(mb + 2*mw)*r*r);

a1 = -(g*l*l*mb*mb*r*r)/deno;
a2 = (g*l*mb*(2*Iw + (mb + 2*mw)*r*r))/deno;

b1 = (r*(Iy + l*mb*(l+r)))/deno;
b2 = -(2*Iw + r*(l*mb + (mb + 2*mw)*r))/deno;
b3 = (d*r)/(2*Iz*r*r + d*d*(Iw + mw*r*r));

A = [0 0 0 1 0 0 ; 
     0 0 0 0 1 0 ;
     0 0 0 0 0 1 ;
     0 a1 0 0 0 0 ;
     0 a2 0 0 0 0 ;
     0 0 0 0 0 0];
     
B = [0 0 ;
     0 0 ; 
     0 0 ;
     b1 b1 ;
     b2 b2 ;
     b3 -b3];
     
C = [ 1 0 0 0 0 0 ;  
      0 1 0 0 0 0 ;
      0 0 1 0 0 0 ;
      0 0 0 0 0 0 ; 
      0 0 0 0 0 0 ;
      0 0 0 0 0 0 ;];
      
D = [0];


Q = [100   0   0 0 0 0 ;
      0 1000  0 0 0 0 ;
      0  0   1  0 0 0 ; 
      0  0   0 1 0 0 ;
      0  0   0 0 10 0 ;
      0  0   0 0  0 1 ;];
      
R = [1 0;
     0 1];

states = {'x'  'theta'  'phi'  'x_dot'  'theta_dot'  'phi_dot'};
inputs = {'uL' ; 'uR'};
outputs = {'x'; 'theta' ; 'phi'};

sys_ss = ss(A,B,C,D);

K = lqr(A,B,Q,R);

Ac = A - B*K;
sys_cl = ss(Ac,B,C,D);

step(sys_cl)


%step(sys_ss,5)