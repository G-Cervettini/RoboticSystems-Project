% ANALISI SINGOLARITA'
 
clear all
close all
clc
 
startup_rvc
 
%% PARAMETRI GEOMETRICI DENAVIT-HARTENBERG
alfa1 = 0 ;a1 = 0 ; d1 = 290e-3 ;
alfa2 = -pi/2 ;a2 = 0 ;d2 = 0 ;
alfa3 = 0 ; a3 = 270e-3 ;d3 = 0 ;
alfa4 = -pi/2 ;a4 = 70e-3 ;d4 = 302e-3 ;
alfa5 = pi/2 ; a5 = 0 ; d5 = 0 ;
alfa6 = 0 ;a6 = 0 ;d6 =0 ;
 
alfaT=0; aT=0; dT=0; tetaT0=0;
alfaB=0;aB=0;dB=0;tetaB0=0;
 
%Parametri tool
incl=60*pi/180;
a=0.06;
b=0.21;
 
% Forza di reazione dovuta al flusso di fluido uscente dall'ugello
% (intensità costante)
forza=30; % [N]
 
%% LIMITI DEI GRADI DI LIBERTA' DEI GIUNTI
 
qlim1=[-165 165]*pi/180;
qlim2=[-110 110]*pi/180;
qlim3=[-110 70]*pi/180;
qlim4=[-160 160]*pi/180;
qlim5=[-120 120]*pi/180;
qlim6=[-400 400]*pi/180;
 
%% LINK DEI ROBOT
 
L(1)=Link('alpha',alfa1,'a',a1,'d',d1,'modified','qlim',qlim1);
L(2)=Link('alpha',alfa2,'a',a2,'d',d2,'offset',-pi/2,'modified','qlim',qlim2);
L(3)=Link('alpha',alfa3,'a',a3,'d',d3,'modified','qlim',qlim3);
L(4)=Link('alpha',alfa4,'a',a4,'d',d4,'modified','qlim',qlim4);
L(5)=Link('alpha',alfa5,'a',a5,'d',d5,'modified','qlim',qlim5);
L(6)=Link('alpha',alfa6,'a',a6,'d',d6,'modified','qlim',qlim6);
 
 
%% COSTRUZIONE DEL ROBOT
 
RobotIRB120=SerialLink(L,'name','RobotIRB120');
 
 
%% SINGOLARITA' BRACCIO CONFIGURAZIONE Q2 E Q3
 
u=0.07;
f=0.302;
w=atan(u/f);
qvec=[0 pi/3 -pi/2+w 0 pi/4 0];
 
%% SINGOLARITA' BRACCIO D3=0 Q1 ININFLUENTE
 
u=0.302;
f=0.270-0.070;
w=atan(70/302);
h=acos(270/sqrt(302^2+70^2))*sin(pi/3);
qvec=[0 -atan(302/(270+70)) 0 0 pi/2.5 0];
 
%% PLOT
format loose
 
[zeroAtcp A_all]=RobotIRB120.fkine(qvec);
zeroA1=A_all(:,:,1);
zeroA2=A_all(:,:,2);
zeroA3=A_all(:,:,3);
zeroA4=A_all(:,:,4);
zeroA5=A_all(:,:,5);
zeroA6=A_all(:,:,6);
zeroAtcp;
 
J=RobotIRB120.jacob0(qvec)
Determinante=det(J)
jsingu(J)
 
z=linspace(0.29,0.9,10);
x=zeros(10);
 
figure
RobotIRB120.plot([qvec],'workspace',[-0.2,0.8,-0.3,0.3,0,0.9],'floorlevel',0,'notiles','noname','nobase','noshadow','jointdiam',4.5,'jointcolor','y','zoom',1,'nowrist','linkcolor','c', 'lightpos',[20 -20 20])
hold on
trplot(RobotIRB120.base,'notext','length',0.2,'rgb','arrow')
trplot(zeroA1,'notext','length',0.2,'rgb','arrow')
% trplot(zeroA2,'notext','length',0.2,'rgb','arrow')
trplot(zeroA3,'notext','length',0.2,'rgb','arrow')
trplot(zeroA5,'notext','length',0.2,'rgb','arrow')
% trplot(zeroA6,'notext','length',0.2,'rgb','arrow')
% trplot(zeroAtcp,'notext','length',0.2,'rgb','arrow')
legend('x','y','z')
% [X Y] = meshgrid(x,y);
% plot3(X,Y,(0.0135*X-8.5e-6*Y+0.0068)/0.0234,'c'),grid on
 
[X Z] = meshgrid(x,z);
plot3(X,0*X,Z,'--m')
 
