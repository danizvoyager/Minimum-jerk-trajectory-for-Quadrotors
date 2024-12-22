clear;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Trajectory generation point wise
% Define points
points = [
    0, 0, 0;   % p0
    0, 0, 5;   % p1
   -1,-1, 5;   % p2
   -2, 0, 5;   % p3
   -1, 1, 5;   % p4
    0, 0, 5;   % p5
    1,-1, 5;   % p6
    2, 0, 5;   % p7
    1, 1, 5;   % p8
    0, 0, 5;   % p9
    0, 0, 0;   % p10  
];
% Extract x, y, and z coordinates
x = points(:, 1)';
y = points(:, 2)';
z = points(:, 3)';
%determine initial and final points for each segment
a_x=x(1,1:(length(x)-1));        
a_y=y(1,1:(length(y)-1));      
a_z=z(1,1:(length(z)-1));
b_x=x(1,2:length(x));          
b_y=y(1,2:length(y));        
b_z=z(1,2:length(z));
[a_theta,a_phi,a_si]=deal(zeros(1,length(a_x)));
[b_theta,b_phi,b_si]=deal(zeros(1,length(a_x)));
%determine required time for each segment and total required time
T_r=abs([b_x-a_x; b_y-a_y; b_z-a_z;b_phi-a_phi;b_theta-a_theta;b_si-a_si]);
T=10*max(T_r);
T_s=sum(T);
% Initialize T_r with the first element of T
T_r = zeros(1, length(T));
% Calculate cumulative sums for T_r (time segment)
for i = 1:length(T)
    T_r(i) = sum(T(1:i));
end
%linearized model 
Jr=6e-5;
omega_r=0.025;
s=Jr*omega_r;
g=9.81;
m=0.5;
k=0.06; % trust coefficient
l=0.6;
d=0.01; % drag factor
w=sqrt(m*g/(6*k));
[Jx,Jy,Jz]=deal(0.01);
kx=1.5;
ky=0.1;
kz=1.9;
A=[0  1     0  0     0  0     0  0     0  0     0  0; 
   0 -kx/m  0  0     0  0     0  0     g  0     0  0;
   0  0     0  1     0  0     0  0     0  0     0  0;
   0  0     0 -ky/m  0  0    -g  0     0  0     0  0;
   0  0     0  0     0  1     0  0     0  0     0  0;
   0  0     0  0     0 -kz/m  0  0     0  0     0  0;
   0  0     0  0     0  0     0  1     0  0     0  0;
   0  0     0  0     0  0     0  0     0 -s/Jx  0  0;
   0  0     0  0     0  0     0  0     0  1     0  0;
   0  0     0  0     0  0     0  s/Jy  0  0     0  0;
   0  0     0  0     0  0     0  0     0  0     0  1;
   0  0     0  0     0  0     0  0     0  0     0  0];
B=[  0             0           0                0           0            0;
     0             0           0                0           0            0;
     0             0           0                0           0            0;
     0             0           0                0           0            0;
     0             0           0                0           0            0;
     2*k*w/m     2*k*w/m     2*k*w/m          2*k*w/m      2*k*w/m     2*k*w/m ;
     0             0           0                 0           0            0;
    -k*l*w/Jx   -2*k*w*l/Jx  -k*l*w/Jx        k*l*w/Jx     2*k*w*l/Jx   k*l*w/Jx;
     0             0           0                 0            0            0;
sqrt(3)*k*l*w/Jy   0   -sqrt(3)*k*l*w/Jy  -sqrt(3)*k*l*w/Jy   0        sqrt(3)*k*l*w/Jy;
     0             0           0                  0           0            0;
 -2*d*w/Jz     2*d*w/Jz      -2*d*w/Jz          2*d*w/Jz    -2*d*w/Jz    2*d*w/Jz ];
 C=[1 0  0 0  0 0  0 0  0 0  0 0;
    0 0  1 0  0 0  0 0  0 0  0 0;
    0 0  0 0  1 0  0 0  0 0  0 0;
    0 0  0 0  0 0  1 0  0 0  0 0;
    0 0  0 0  0 0  0 0  1 0  0 0;
    0 0  0 0  0 0  0 0  0 0  1 0];
[rowb,colb]=size(B);
[rowc,colc]=size(C);
D=zeros(rowc,colb);
Co=rank(ctrb(A,B));
Ov=rank(obsv(A,C));
% Q=1000*eye(12);
Q=1000*C'*C;
R=1;
k_lqr=lqr(A,B,Q,R);
k_o=-pinv(C*pinv(A-B*k_lqr)*B);
sim('MRAC_PID_SMC_for_Hexa_rotor.slx')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Fancy Trajectory model
T_sf=50;
r=0.2;
f=0.5;
t=[0: 0.05: T_s];
xd = r.*sin(2*pi*t*f);
yd = r.*cos(2*pi*t*f);
zd = r.*t;
plot3(xd,yd,zd)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Number eight
t = [0:0.001:10];
% f = 10;
x =r*sin(2*pi*t*f);
y =r*sin(2*pi*t*2*f);
z=ones(size(t));
figure();
plot3(x,y,z);