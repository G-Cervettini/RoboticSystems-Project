%rrmc_ol_pre_post_01.m

% file pre/post-process di rrmc_ol_01.slx (Simulink file)

clear
close all

%% pre-processor

% functional and geometric data robot ABB IRB 120
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%NOTA: i parametri di DH sono denominati con alfai, ai, di, essendo i
%indice del link cui si riferiscono (e non asse lungo i quali sono misurati)
%unità di misura: rad, m

%Denavit-Hartenberg parameters link 1
alfa1 = 0 ;
a1 = 0 ;
d1 = 290e-3 ;
teta10 = 0 ;
qlim1=[-165 +165]*pi/180;   %range q1

%Denavit-Hartenberg parameters link 2
alfa2 = -pi/2 ;
a2 = 0 ;
d2 = 0 ;
teta20 = -pi/2 ;
qlim2=[-110 +110]*pi/180;   %range q2

%Denavit-Hartenberg parameters link 3
alfa3 = 0 ;
a3 = 270e-3 ;
d3 = 0 ;
teta30 = 0 ;
qlim3=[-110 +70]*pi/180;   %range q3

%Denavit-Hartenberg parameters link 4
alfa4 = -pi/2 ;
a4 = 70e-3 ;
d4 = 302e-3 ;
teta40 = 0 ;
qlim4=[-160 +160]*pi/180;   %range q4

%Denavit-Hartenberg parameters link 5
alfa5 = pi/2 ;
a5 = 0 ;
d5 = 0 ;
teta50 = 0 ;
qlim5=[-120 +120]*pi/180;   %range q5

%Denavit-Hartenberg parameters link 6
alfa6 = -pi/2 ;
a6 = 0 ;
d6 = 72e-3 ;
teta60 = 0 ;
qlim6=[-400 +400]*pi/180;   %range q6

%tool (TCP) wrt 6
alphat=60*pi/180;
f=210e-3;   %201e-3;
h=60e-3;    %46e-3;
ATCP6o=[cos(alphat) 0 -sin(alphat) -h;    %sr TCP wrt 6
        0           1  0            0;
        sin(alphat) 0  cos(alphat)  f;
        0           0  0            1];


% Robot build up in Robotics Toolbox
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% L = vector of Links 1-6
L(1)=Link('alpha',alfa1,'a',a1,'d',d1,'offset',teta10,'modified','qlim',qlim1);     % robot link 1
L(2)=Link('alpha',alfa2,'a',a2,'d',d2,'offset',teta20,'modified','qlim',qlim2);     % robot link 2
L(3)=Link('alpha',alfa3,'a',a3,'d',d3,'offset',teta30,'modified','qlim',qlim3);     % robot link 3
L(4)=Link('alpha',alfa4,'a',a4,'d',d4,'offset',teta40,'modified','qlim',qlim4);     % robot link 4
L(5)=Link('alpha',alfa5,'a',a5,'d',d5,'offset',teta50,'modified','qlim',qlim5);     % robot link 5
L(6)=Link('alpha',alfa6,'a',a6,'d',d6,'offset',teta60,'modified','qlim',qlim6);     % robot link 6

rob6R= SerialLink(L,'name','rob6R');     % to build up a SerialLink object
% add tool TCP reference frame
rob6R.tool = ATCP6o ;

% display rob6
rob6R

%% dati simulazione e run simulatore

qvet_ini=[0 0 0 0 30 0]'*pi/180; % configurazione iniziale robot spazio giunti

% vTCP=[-.5 0 0 0 0 0]'; % velocità generalizzata TCP desiderata
% vTCP=[-.5 .1 0 0 0 0]'; % velocità generalizzata TCP desiderata
vTCP=[-.5 0 0 1 0 0]'; % velocità generalizzata TCP desiderata

stop_time=1;        % durata simulazione [s]
step_time=0.05;     % passo integrazione modello simulazione [s]  20Hz
% step_time=0.005;     % passo integrazione modello simulazione [s]  200Hz

% >>>> OPEN rrmc_ol_01.slx
% >>>> RUN rrmc_ol_01.slx

%% post-processor

close all
 
T=rob6R.fkine(qvet);        % matrici di posa TCP
xyz=transl(T);              % posizione origine TCP

figure('name','rob6R')
rob6R.plot(qvet_ini','notiles','nobase','zoom',2)
pause
rob6R.plot(qvet,'notiles','nobase','trail','-b','zoom',2)
pause

figure('name','spazio giunti')
plot(tout,qvet(:,1),tout,qvet(:,2),tout,qvet(:,3),tout,qvet(:,4),tout,qvet(:,5),tout,qvet(:,6)),grid
legend('q1 [rad]','q2 [rad]','q3 [rad]','q4 [rad]','q5 [rad]','q6 [rad]','Location','best')

figure('name','spazio cartesiano TCP')
subplot(221)
plot(tout,xyz(:,1)),grid,xlabel('tempo [s]'),ylabel('x_T_C_P [m]')
subplot(222)
plot(tout,xyz(:,2)),grid,xlabel('tempo [s]'),ylabel('y_T_C_P [m]')
subplot(223)
plot(tout,xyz(:,3)),grid,xlabel('tempo [s]'),ylabel('z_T_C_P [m]')
subplot(224)
plot3(xyz(:,1),xyz(:,2),xyz(:,3),'b'),grid
view(-35,30)
xlabel('x'),ylabel('y'),zlabel('z'),title('percorso TCP')



