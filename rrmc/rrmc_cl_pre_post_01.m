%rrmc_cl_pre_post_01.m

% file pre/post-process di rrmc_cl_01.slx (Simulink file)

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

% traiettoria di pura traslazione del TCP
qvet_ini=[-0.0000    1.4526   -2.6861    0.0000    1.2335   -0.0000];   % configurazione iniziale, spazio giunti [rad]
TCP_ini=rob6R.fkine(qvet_ini);              % sr TCP wrt 0 configurazione iniziale
deltaTCP=[-0.08 0.2 0.03];                       % variazione di posizione TCP [m]
TCP_fin=transl(deltaTCP)*TCP_ini;           % sr TCP wrt 0 configurazione finale

stop_time=2;        % durata traiettoria e simulazione [s]
step_time=0.01;     % passo integrazione modello simulazione [s] 100Hz

Kp=100;             % guadagno proporzionale controllo velocità giunti (quanto è pronto il robot a correggere l'errore di traiettoria)

% >>>> OPEN rrmc_cl_01.slx
% >>>> RUN rrmc_cl_01.slx

%% post processor

xyz_ref=transl(Tref);      % posizione origine TCP desiderata
T=rob6R.fkine(qvet);       % matrici di posa TCP durate traiettoria eseguita
xyz=transl(T);             % posizione origine TCP durate traiettoria eseguita

%% plot traiettoria desiderata

figure('name','traiettoria TCP desiderata')
trplot(TCP_ini,'frame','ini','color','b','length',0.2)
hold on
trplot(TCP_fin,'frame','fin','color','r','length',0.2) 
axis([0 1 -.5 .5 .5 1.5])
pause
tranimate(Tref,'notext','color','k','length',0.15,'fps',100)
pause

figure('name','posizione TCP desiderata')
subplot(221)
plot(tout,xyz_ref(:,1)),grid,xlabel('tempo [s]'),ylabel('x_T_C_P [m]')
subplot(222)
plot(tout,xyz_ref(:,2)),grid,xlabel('tempo [s]'),ylabel('y_T_C_P [m]')
subplot(223)
plot(tout,xyz_ref(:,3)),grid,xlabel('tempo [s]'),ylabel('z_T_C_P [m]')
subplot(224)
plot3(xyz_ref(:,1),xyz_ref(:,2),xyz_ref(:,3)),grid
view(-35,30)
xlabel('x'),ylabel('y'),zlabel('z'),title('percorso TCP')

%% plot movimento robot eseguito

figure('name','rob6R')
rob6R.plot(qvet_ini,'notiles','nobase','zoom',2)
axis([-.5 1 -.5 1 -.7 1.5])
hold on
pause
rob6R.plot(qvet,'noname','notiles','nobase','trail','-b','zoom',2,'fps',100)

%% plot traiettoria eseguita

figure('name','spazio giunti')
plot(tout,qvet(:,1),tout,qvet(:,2),tout,qvet(:,3),tout,qvet(:,4),tout,qvet(:,5),tout,qvet(:,6)),grid
legend('q1 [rad]','q2 [rad]','q3 [rad]','q4 [rad]','q5 [rad]','q6 [rad]','Location','best')

figure('name','spazio cartesiano TCP')
subplot(221)
plot(tout,xyz(:,1),tout,xyz_ref(:,1)),grid
xlabel('tempo [s]'),ylabel('x_T_C_P [m]'),legend('eff','ref','location','best')
subplot(222)
plot(tout,xyz(:,2),tout,xyz_ref(:,2)),grid
xlabel('tempo [s]'),ylabel('y_T_C_P [m]'),legend('eff','ref','location','best')
subplot(223)
plot(tout,xyz(:,3),tout,xyz_ref(:,3)),grid
xlabel('tempo [s]'),ylabel('z_T_C_P [m]'),legend('eff','ref','location','best')
subplot(224)
plot3(xyz(:,1),xyz(:,2),xyz(:,3),xyz_ref(:,1),xyz_ref(:,2),xyz_ref(:,3)),grid
legend('eff','ref','location','best')
view(-35,30)
xlabel('x'),ylabel('y'),zlabel('z'),title('percorso TCP')
