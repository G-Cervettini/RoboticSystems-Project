% Robot3R_SphericWrist.m

% by Guglielmo Cervettini - Politecnico di Torino

% use <Run Section> in Editor Toolbar to run each section 
disp(' use <Run Section> in Editor Toolbar to run each section')

%% Robotics Toolbox by Peter Corke


% ROBOT_ABB_IRB_120
 
clear all
close all
clc
 
startup_rvc    %load robotics toolbox
 
%% Parametri geometrici (Denavit-Hartenberg)
 
% Link 1
alpha1=0;
a1=0;
d1=0.290;
teta10=0;
qlim1=[-165 165]*pi/180;    %limiti gdl giunti (da catalogo)
 
% Link 2
alpha2=-0.5*pi;
a2=0;
d2=0;
teta20=-0.5*pi;
qlim2=[-110 110]*pi/180;    %limiti gdl giunti (da catalogo)
 
% Link 3
alpha3=0;
a3=0.270;
d3=0;
teta30=0;
qlim3=[-110 70]*pi/180;     %limiti gdl giunti (da catalogo)
 
% Link 4
alpha4=-0.5*pi;
a4=0.070;
d4=0.302;
teta40=0;
qlim4=[-160 160]*pi/180;    %limiti gdl giunti (da catalogo)
 
% Link 5
alpha5=0.5*pi;
a5=0;
d5=0;
teta50=0;
qlim5=[-120 120]*pi/180;    %limiti gdl giunti (da catalogo)
 
% Link 6
alpha6=-0.5*pi;
a6=0;
d6=0.072;
teta60=0;
qlim6=[-400 400]*pi/180;    %limiti gdl giunti (da catalogo)
 
% descrivo due ulteriori link che userò per analisi dinamica
% (per poter definire le proprietà inerziali di base e tool)
 
% Link B
alphaB=0*pi;
aB=0;
dB=0;
tetaB0=0;
qlimB=[0 0]*pi/180;  % limiti gdl giunti (non voglio che questi ulteriori
                     % link abbiano un moto relativo rispetto ai precedenti)
 
% Link T
alphaT=0*pi;
aT=0;
dT=0;
tetaT0=0;
qlimT=[0 0]*pi/180;  % limiti gdl giunti (non voglio che questi ulteriori
                     % link abbiano un moto relativo rispetto ai precedenti)
 
%definizione parametri di D-H
L(1)=Link('alpha',alpha1,'a',a1,'d',d1,'offset',teta10,'modified','qlim',qlim1);  % robot link 1
L(2)=Link('alpha',alpha2,'a',a2,'d',d2,'offset',teta20,'modified','qlim',qlim2);  % robot link 2
L(3)=Link('alpha',alpha3,'a',a3,'d',d3,'offset',teta30,'modified','qlim',qlim3);  % robot link 3
L(4)=Link('alpha',alpha4,'a',a4,'d',d4,'offset',teta40,'modified','qlim',qlim4);  % robot link 4
L(5)=Link('alpha',alpha5,'a',a5,'d',d5,'offset',teta50,'modified','qlim',qlim5);  % robot link 5
L(6)=Link('alpha',alpha6,'a',a6,'d',d6,'offset',teta60,'modified','qlim',qlim6);  % robot link 6
 
%costruzione robot
ABBirb120=SerialLink(L,'name','ABBirb120');     % to build up a SerialLink object
%ABBirb120.display
 
% Assegnazione del vettore dei gradi di libertà
 
%qvec0=[0 0 0 0 0 0]*pi/180;  %vettore dei gradi di libertà dei giunti assegnato
%qvec0=[30 60 45 -50 -80 -20]*pi/180;  %vettore dei gradi di libertà dei giunti assegnato
%qvec0=[20 30 10 -30 30 -20]*pi/180;  %vettore dei gradi di libertà dei giunti assegnato
qvec0=[0 90 -90 0 0 0]*pi/180;  %vettore dei gradi di libertà dei giunti assegnato
 
zeroAee=ABBirb120.fkine(qvec0) % .fkine calcola la matrice omogenea dell'EE rispetto al sistema zero (0Aee)
 
% plot del robot prima di aver assegnato il tool
% figure(1)
% ABBirb120.plot(qvec0) % plot del robot con i gradi di libertà assegnati
% hold on
% trplot(ABBirb120.base,'frame','A0')
 
% Forza di reazione dovuta al flusso di fluido uscente dall'ugello
% (intensità costante)
forza=30; %[N]
 
% Accelerazione gravitazionale
%g=9.80665; %[m/s^2]
g=9.81;     %[m/s^2]
 
% Parametri tool ugello di verniciatura
angolo=60*pi/180;
ics=-0.060;
zeta=0.210;
 
% Matrice eeAtcp
eeAtcp=[cos(angolo)  0  -sin(angolo)  ics
        0            1  0             0
        sin(angolo)  0  cos(angolo)   zeta
        0            0  0             1   ];
      
ABBirb120.tool=eeAtcp;
ABBirb120.display
 
% plot del robot dopo di aver assegnato il tool
figure(1)
ABBirb120.plot(qvec0) % plot del robot con i gradi di libertà assegnati
hold on
trplot(ABBirb120.base,'frame','A0')

%% PLOT PER I SISTEMI DI RIFERIMENTO DI D-H
 
qvec=[0 0 0 0 0 0];
[zeroAtcp, A_all]=ABBirb120.fkine(qvec);
zeroA1=A_all(:,:,1);
zeroA2=A_all(:,:,2);
zeroA3=A_all(:,:,3);
zeroA4=A_all(:,:,4);
zeroA5=A_all(:,:,5);
zeroA6=A_all(:,:,6);
zeroAtcp;
 
% Plot con i sistemi di riferimento: base,1,2,5,6,tool
figure
ABBirb120.plot([qvec],'workspace',[-0.2,0.8,-0.3,0.3,0,0.9],'floorlevel',0,'notiles','noname','nobase','noshadow','jointdiam',4.5,'jointcolor','y','zoom',1,'nowrist','linkcolor','c')
hold on
trplot(ABBirb120.base,'notext','length',0.2,'rgb','arrow')
trplot(zeroA1,'notext','length',0.2,'rgb','arrow')
trplot(zeroA3,'notext','length',0.2,'rgb','arrow')
trplot(zeroA5,'notext','length',0.2,'rgb','arrow')
trplot(zeroA6,'notext','length',0.2,'rgb','arrow')
trplot(zeroAtcp,'notext','length',0.2,'rgb','arrow')
legend('x','y','z')
 
% Plot con i sistemi di riferimento: base,2,4,tool
figure
ABBirb120.plot([qvec],'workspace',[-0.2,0.8,-0.3,0.3,0,0.9],'floorlevel',0,'notiles','noname','nobase','noshadow','jointdiam',4.5,'jointcolor','y','zoom',1,'nowrist','linkcolor','c')
hold on
trplot(ABBirb120.base,'notext','length',0.2,'rgb','arrow')
trplot(zeroA2,'notext','length',0.2,'rgb','arrow')
trplot(zeroA4,'notext','length',0.2,'rgb','arrow')
trplot(zeroAtcp,'notext','length',0.2,'rgb','arrow')

%% ANALISI SINGOLARITA'
 
%% SINGOLARITA' BRACCIO CONFIGURAZIONE Q2 E Q3
 
u=0.07;
f=0.302;
w=atan(u/f);
qvec=[0 pi/3 -pi/2+w 0 pi/4 pi/4];
 
%% SINGOLARITA' BRACCIO D3=0 Q1 ININFLUENTE
 
u=0.302;
f=0.270-0.070;
w=atan(70/302);
h=acos(270/sqrt(302^2+70^2))*sin(pi/3);
qvec=[0 -atan(302/(270+70)) 0 0 pi/2+atan(302/340) 0];
 
%% SINGOLARTIA' POLSO
 
qvec=pi/180*[10 -10 15 0 0 0];
 
%% Plot delle singolarità
 
[zeroAtcp, A_all]=ABBirb120.fkine(qvec);
zeroA1=A_all(:,:,1);
zeroA2=A_all(:,:,2);
zeroA3=A_all(:,:,3);
zeroA4=A_all(:,:,4);
zeroA5=A_all(:,:,5);
zeroA6=A_all(:,:,6);
zeroAtcp;
J= ABBirb120.jacob0(qvec)
Determinante=det(J)
jsingu(J)
 
figure
ABBirb120.plot([qvec],'workspace',[-0.2,0.8,-0.3,0.3,0,0.9],'floorlevel',0,'notiles','noname','nobase','noshadow','jointdiam',4.5,'jointcolor','y','zoom',1,'nowrist','linkcolor','c')
hold on
trplot(ABBirb120.base,'notext','length',0.2,'rgb','arrow')
% trplot(zeroA1,'notext','length',0.2,'rgb','arrow')
% trplot(zeroA3,'notext','length',0.2,'rgb','arrow')
trplot(zeroA4,'notext','length',0.2,'rgb','arrow')
% trplot(zeroA5,'notext','length',0.2,'rgb','arrow')
trplot(zeroA6,'notext','length',0.2,'rgb','arrow')
% trplot(zeroAtcp,'notext','length',0.2,'rgb','arrow')
legend('x','y','z')

 

%% Matrici di tutti i corpi rispetto al sistema zero
[zeroAtcp, A_all]=ABBirb120.fkine(qvec0); 
 
zeroA0=eye(4);
 
zeroA1=A_all(:,:,1);
zeroA2=A_all(:,:,2);
zeroA3=A_all(:,:,3);
zeroA4=A_all(:,:,4);
zeroA5=A_all(:,:,5);
zeroA6=A_all(:,:,6);
zeroAtcp;
 
% CALCOLI PER PLOTTAGGIO CAD DEL ROBOT CON SIMULINK
 
% Matrici S.R. CAD wrt S.R. D-H
 
dh0Acad0=[1  0  0  0
          0  1  0  0
          0  0  1  0
          0  0  0  1];
 
dh1Acad1=[1  0  0  0
          0  1  0  0
          0  0  1  0
          0  0  0  1];
 
dh2Acad2=[0  0  1  a3
          1  0  0  0
          0  1  0  0
          0  0  0  1 ];
 
dh3Acad3=[0  0  1  0
          1  0  0  0
          0  1  0  0
          0  0  0  1];
 
dh4Acad4=[0  0  1  0
          0 -1  0  0
          1  0  0  0
          0  0  0  1];
 
dh5Acad5=[0  0  1  0
          1  0  0  0
          0  1  0  0
          0  0  0  1];
 
dh6Acad6=[0  0  1  0
          0 -1  0  0
          1  0  0  0
          0  0  0  1];
 
dhTAcadT=dh6Acad6; %la utilizzo più avanti per trasferire le info di baricentro
                   % e tensore di inerzia nel S.R. di D-H
 
tcpAcadt=inv([sin(angolo)  0  cos(angolo)  zeta
              0           -1  0            0
              cos(angolo)  0 -sin(angolo)  ics
              0            0  0            1   ]);
 
% MATRICI SR_CAD (SERVONO PER IL PLOTTAGGIO DEL ROBOT CON SIMULINK)
 
zeroAcad0=zeroA0*dh0Acad0;
zeroAcad1=zeroA1*dh1Acad1;
zeroAcad2=zeroA2*dh2Acad2;
zeroAcad3=zeroA3*dh3Acad3;
zeroAcad4=zeroA4*dh4Acad4;
zeroAcad5=zeroA5*dh5Acad5;
zeroAcad6=zeroA6*dh6Acad6;
zeroAcadt=zeroAtcp*tcpAcadt;
 
%% PLOT CAD DEL ROBOT CON MECHANICS EXPLORER
 
plot_IRB120(zeroAcad1,zeroAcad2,zeroAcad3,zeroAcad4,zeroAcad5,zeroAcad6)    %plot CAD senza tool
plot_IRB120tool(zeroAcad1,zeroAcad2,zeroAcad3,zeroAcad4,zeroAcad5,zeroAcad6,zeroAcadt)  %plot CAD con tool
delete *.asv % Cleaning
 
%% Proprietà inerziali
 
% Masse
m0=9.62116; %[kg]
m1=4.25439; %[kg]
m2=4.42193; %[kg]
m3=4.58341; %[kg]
m4=1.34194; %[kg]
m5=0.75825; %[kg]
m6=0.01892; %[kg]
mT=0.43649; %[kg]
 
% Posizioni origini S.R. Centri di massa (baricentri) wrt S.R. CAD
CM0=[-42.04,0.08,79.64]*1e-3; %[m]
CM1=[0.10,-0.12,-51.59]*1e-3; %[m]
CM2=[0.78,-2.12,-168.76]*1e-3; %[m]
CM3=[22.81,1.06,57.91]*1e-3; %[m]
CM4=[-77.30,0.15,0.41]*1e-3; %[m]
CM5=[-1.09,0.04,0.06]*1e-3; %[m]
CM6=[-7.06,-0.17,0.00]*1e-3; %[m]
CMT=[98.57,0.00,11.37]*1e-3; %[m]
 
CM0o=[CM0,1];
CM1o=[CM1,1];
CM2o=[CM2,1];
CM3o=[CM3,1];
CM4o=[CM4,1];
CM5o=[CM5,1];
CM6o=[CM6,1];
CMTo=[CMT,1];
 
 
% Matrici orientazione S.R. Centri di massa wrt S.R. CAD
cad0Acm0=[ 0.9983, 0.0581, 0.0000
           0.0000, 0.0000, 1.0000
           0.0581,-0.9983, 0.0000];
       
cad1Acm1=[-0.0062, 0.9977, 0.0678
           0.0049,-0.0678, 0.9977
           0.9999, 0.0065,-0.0045];
       
cad2Acm2=[ 0.0000, 0.0000, 1.0000
          -0.0323,-0.9994, 0.0000
           0.9995,-0.0322, 0.0000];
 
cad3Acm3=[ 0.9578, 0.2876, 0.0020
          -0.0147, 0.0421, 0.9990
           0.2872,-0.9568, 0.0446];
 
cad4Acm4=[ 0.9998, 0.0184, 0.0067
          -0.0183, 0.9998,-0.0105
          -0.0069, 0.0103, 0.9999];
 
cad5Acm5=[ 1.0000, 0.0000, 0.0000
           0.0000, 0.0000, 1.0000
           0.0000,-1.0000, 0.0000];
 
cad6Acm6=[ 0.0000, 0.0000, 1.0000
           0.0000,-1.0000, 0.0000
           1.0000, 0.0000, 0.0000];
 
cadTAcmT=[-0.9826, 0.1857, 0.0000
           0.0000, 0.0000,-1.0000
          -0.1857,-0.9826, 0.0000];
      
% Tensori centrali di inerzia (wrt S.R. CM che è un S.R. centrale (=principale+baricentrico))
IG0=[38161020.67, 0.000000000, 0.000000000
     0.000000000, 73243624.28, 0.000000000
     0.000000000, 0.000000000, 76053785.67]*1e-9; %[kg*m^2]
 
IG1=[14500004.44, 0.000000000, 0.000000000
     0.000000000, 19720761.52, 0.000000000
     0.000000000, 0.000000000, 19982083.09]*1e-9; %[kg*m^2]
 
IG2=[29344594.91, 0.000000000, 0.000000000
     0.000000000, 47046106.12, 0.000000000
     0.000000000, 0.000000000, 68231040.33]*1e-9; %[kg*m^2]
 
IG3=[12341191.27, 0.000000000, 0.000000000
     0.000000000, 20428062.83, 0.000000000
     0.000000000, 0.000000000, 26035241.49]*1e-9; %[kg*m^2]
 
IG4=[2876276.090, 0.000000000, 0.000000000
     0.000000000, 4056258.710, 0.000000000
     0.000000000, 0.000000000, 5309447.470]*1e-9; %[kg*m^2]
 
IG5=[561630.6400, 0.000000000, 0.000000000
     0.000000000, 1131171.140, 0.000000000
     0.000000000, 0.000000000, 1238480.600]*1e-9; %[kg*m^2]
 
IG6=[2292.670000, 0.000000000, 0.000000000
     0.000000000, 2343.180000, 0.000000000
     0.000000000, 0.000000000, 4117.120000]*1e-9; %[kg*m^2]
 
IGT=[179916.3000, 0.000000000, 0.000000000
     0.000000000, 1572293.230, 0.000000000
     0.000000000, 0.000000000, 1674963.520]*1e-9; %[kg*m^2]
 
% Posizioni origini S.R. Centri di massa wrt S.R. D.H.
 
b0o=dh0Acad0*CM0o';
b1o=dh1Acad1*CM1o';
b2o=dh2Acad2*CM2o';
b3o=dh3Acad3*CM3o';
b4o=dh4Acad4*CM4o';
b5o=dh5Acad5*CM5o';
b6o=dh6Acad6*CM6o';
bTo=dhTAcadT*CMTo';
 
b0=b0o(1:3,1);
b1=b1o(1:3,1);
b2=b2o(1:3,1);
b3=b3o(1:3,1);
b4=b4o(1:3,1);
b5=b5o(1:3,1);
b6=b6o(1:3,1);
bT=bTo(1:3,1);
 
% Tensori di inerzia (wrt S.R. D.H. ma baricentrico)
 
dhIG0=(dh0Acad0(1:3,1:3)*cad0Acm0)*IG0*((dh0Acad0(1:3,1:3)*cad0Acm0)');
dhIG1=(dh1Acad1(1:3,1:3)*cad1Acm1)*IG1*((dh1Acad1(1:3,1:3)*cad1Acm1)');
dhIG2=(dh2Acad2(1:3,1:3)*cad2Acm2)*IG2*((dh2Acad2(1:3,1:3)*cad2Acm2)');
dhIG3=(dh3Acad3(1:3,1:3)*cad3Acm3)*IG3*((dh3Acad3(1:3,1:3)*cad3Acm3)');
dhIG4=(dh4Acad4(1:3,1:3)*cad4Acm4)*IG4*((dh4Acad4(1:3,1:3)*cad4Acm4)');
dhIG5=(dh5Acad5(1:3,1:3)*cad5Acm5)*IG5*((dh5Acad5(1:3,1:3)*cad5Acm5)');
dhIG6=(dh6Acad6(1:3,1:3)*cad6Acm6)*IG6*((dh6Acad6(1:3,1:3)*cad6Acm6)');
dhIGT=(dhTAcadT(1:3,1:3)*cadTAcmT)*IGT*((dhTAcadT(1:3,1:3)*cadTAcmT)');
 
% PROPRIETA' INERZIALI DEI LINK GIA' ESISTENTI
 
L(1).m=m1;
L(1).r=b1;
L(1).I=dhIG1;
 
L(2).m=m2;
L(2).r=b2;
L(2).I=dhIG2;
 
L(3).m=m3;
L(3).r=b3;
L(3).I=dhIG3;
 
L(4).m=m4;
L(4).r=b4;
L(4).I=dhIG4;
 
L(5).m=m5;
L(5).r=b5;
L(5).I=dhIG5;
 
L(6).m=m6;
L(6).r=b6;
L(6).I=dhIG6;
 
% NUOVA COSTRUZIONE DEL ROBOT
%(AGGIUNGO LA BASE E IL TOOL PER POTERNE DEFINIRE LE PROPRIETA' INERZIALI)
 
%definizione parametri di D-H
 
T(1)=Link('alpha',alphaT,'a',aT,'d',dT,'offset',tetaT0,'modified','qlim',qlimT); %robot link tool
tool=SerialLink(T,'name','tool'); % to build up a SerialLink object
 
T(1).m = mT; 
T(1).r = bT;
T(1).I = dhIGT; 
 
B(1)=Link('alpha',alphaB,'a',aB,'d',dB,'offset',tetaB0,'modified','qlim',qlimB); %robot link base
base=SerialLink(B,'name','base'); % to build up a SerialLink object
 
B(1).m = m0; 
B(1).r = b0; 
B(1).I = dhIG0; 
 
ABBirb120_Dyn=SerialLink([base,ABBirb120,tool],'name','ABBirb120_D_y_n');
ABBirb120_Dyn.tool=eeAtcp;
 
figure(2)
ABBirb120_Dyn
ABBirb120_Dyn.plot([0,qvec0,0])
hold on
trplot(ABBirb120_Dyn.base,'frame','A0')
 
%% Mappatura spazio di lavoro
q2vec=linspace(min(qlim2),max(qlim2),50);
q3vec=linspace(min(qlim3),max(qlim3),50);
 
 
i23=1;
for i2=1:length(q2vec)
    for i3=1:length(q3vec)
        q1=0;
        q2=q2vec(i2);
        q3=q3vec(i3);
        q4=0;
        q5=0;
        q6=0;
        zeroA4=ABBirb120.A([1 2 3 4],[q1 q2 q3 q4]); % Centro Polso
        zeroA6=ABBirb120.A([1 2 3 4 5 6],[q1 q2 q3 q4 q5 q6]); % End Effector
        C4=zeroA4(1:3,4);
        C6=zeroA6(1:3,4);
        wspace(i23,:)=[q1 q2 q3 q4 q5 q6 C4' C6' 0];
        i23=i23+1;
    end
end
 
% figure(2)
% plot(wspace(:,7),wspace(:,9),'*')
% axis equal
 
% nel ciclo for cerco i punti da escludere dallo spazio di lavoro potenziale del robot
 
for iCR=1:numrows(wspace)
    if wspace(iCR,9)<0 | wspace(iCR,9)<0.2 & abs(wspace(iCR,7))<0.1
       wspace(iCR,13)=1; % flag posto a uno per i valori critici
    end
end
 
wspace01=sortrows(wspace,13); % ordino le righe di wspace in modo che si abbiano prima quelle con flag=0 e poi quelle con flag=1
row1=find(wspace01(:,13),1); % trovo la prima riga in cui si ha flag=1 e ne salvo l'indice in row1 
wspace0=wspace01(1:row1-1,:); % porzione del wspace contenente i punti raggiungibili
wspace1=wspace01(row1:numrows(wspace01),:); % porzione del wspace contenente i punti critici
 
 
%wspace CP
figure(3)
subplot(2,2,1)
plot(radtodeg(wspace(:,2)),radtodeg(wspace(:,3)),'*')
axis equal
xlabel('q2 [deg]')
ylabel('q3 [deg]')
title('spazio q3 vs q2')
 
subplot(2,2,2)
plot(wspace(:,7),wspace(:,9),'*')
axis equal
xlabel('x')
ylabel('z')
title('wspace CP')
 
subplot(2,2,3)
plot(radtodeg(wspace0(:,2)),radtodeg(wspace0(:,3)),'b*')
hold on
plot(radtodeg(wspace1(:,2)),radtodeg(wspace1(:,3)),'r*')
axis equal
xlabel('q2 [deg]')
ylabel('q3 [deg]')
title('spazio q3 vs q2 punti critici in rosso');
 
subplot(2,2,4)
plot(wspace0(:,7),wspace0(:,9),'b*')
hold on
plot(wspace1(:,7),wspace1(:,9),'r*')
axis equal
xlabel('x')
ylabel('z')
title('wspace CP punti critici in rosso');
 
%% TRAIETTORIA CIRCOLARE
 
load('rvctools\traiettoria_6gdl_v02.mat');
 
% PLOTTAGGIO DEI GDL PER OGNI GIUNTO
 
figure(4)
subplot(3,2,1)
plot(tempovet,q1vet,tempovet,q1pvet,tempovet,q1ppvet),grid on
xlabel('Tempo (s)')
legend('q1[deg]','q1p[deg/s]','q1pp[deg/s^2]')
title('gdl Giunto 1')
     
subplot(3,2,2)
plot(tempovet,q2vet,tempovet,q2pvet,tempovet,q2ppvet),grid on
xlabel('Tempo (s)')
legend('q2[deg]','q2p[deg/s]','q2pp[deg/s^2]')
title('gdl Giunto 2')
 
subplot(3,2,3)
plot(tempovet,q3vet,tempovet,q3pvet,tempovet,q3ppvet),grid on
xlabel('Tempo (s)')
legend('q3[deg]','q3p[deg/s]','q3pp[deg/s^2]')
title('gdl Giunto 3')
 
subplot(3,2,4)
plot(tempovet,q4vet,tempovet,q4pvet,tempovet,q4ppvet),grid on
xlabel('Tempo (s)')
legend('q4[deg]','q4p[deg/s]','q4pp[deg/s^2]')
title('gdl Giunto 4')
 
subplot(3,2,5)
plot(tempovet,q5vet,tempovet,q5pvet,tempovet,q5ppvet),grid on
xlabel('Tempo (s)')
legend('q5[deg]','q5p[deg/s]','q5pp[deg/s^2]')
title('gdl Giunto 5')
 
subplot(3,2,6)
plot(tempovet,q6vet,tempovet,q6pvet,tempovet,q6ppvet),grid on
xlabel('Tempo (s)')
legend('q6[deg]','q6p[deg/s]','q6pp[deg/s^2]')
title('gdl Giunto 6')
 
% calcolo del vettore di velocità generalizzata per ogni tempo i
 
for i=1:length(tempovet)
 
   qvec_i=deg2rad([q1vet(i),q2vet(i),q3vet(i),q4vet(i),q5vet(i),q6vet(i)]);
   qpvec_i=deg2rad([q1pvet(i),q2pvet(i),q3pvet(i),q4pvet(i),q5pvet(i),q6pvet(i)]);
    
   J(:,:,i)=ABBirb120.jacob0(qvec_i); 
   V(:,i)=J(:,:,i)*qpvec_i';
 
   [zeroAtcp, A_all]=ABBirb120.fkine(qvec_i);
   
   zeropCP(:,i)=A_all(1:3,4,4);
   zeropEE(:,i)=A_all(1:3,4,6);
   zeropTcp(:,i)=zeroAtcp(1:3,4);
               
end
 
% jsingu(J(:,:,i))
 
% PLOTTAGGIO VETTORE DI VELOCITA' GENERALIZZATA
 
figure(5)
 
subplot(2,1,1)
plot(tempovet,V(1,:),tempovet,V(2,:),tempovet,V(3,:),tempovet,sqrt(V(1,:).^2+V(2,:).^2+V(3,:).^2)),grid on
xlabel('tempo [s]');
title('Velocità Tool Centre Point v_T')
legend('v_T_x [m/s]','v_T_y [m/s]','v_T_z [m/s]','|v_T| [m/s]')
 
subplot(2,1,2)
plot(tempovet,V(4,:),tempovet,V(5,:),tempovet,V(6,:)),grid on
xlabel('tempo [s]');
title('Velocità angolare Tool w_T')
legend('w_T_x [rad/s]','w_T_y [rad/s]','w_T_z [rad/s]')
 
% PLOTTAGGIO TRAIETTORIA E PERCORSO CP
 
figure(6)
 
subplot(2,2,1)
plot(tempovet,zeropCP(1,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('x_C_P [m]')
 
subplot(2,2,2)
plot(tempovet,zeropCP(2,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('y_C_P [m]')
 
subplot(2,2,3)
plot(tempovet,zeropCP(3,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('z_C_P [m]')
 
subplot(2,2,4)
plot3(zeropCP(1,:),zeropCP(2,:),zeropCP(3,:),'-c','Linewidth',2),grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Percorso CP')
 
% PLOTTAGGIO TRAIETTORIA E PERCORSO EE
 
figure(7)
 
subplot(2,2,1)
plot(tempovet,zeropEE(1,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('x_E_E [m]')
 
subplot(2,2,2)
plot(tempovet,zeropEE(2,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('y_E_E [m]')
 
subplot(2,2,3)
plot(tempovet,zeropEE(3,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('z_E_E [m]')
 
subplot(2,2,4)
plot3(zeropEE(1,:),zeropEE(2,:),zeropEE(3,:),'-m','Linewidth',2),grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Percorso EE')
 
% PLOTTAGGIO TRAIETTORIA E PERCORSO TCP
 
figure(8)
 
subplot(2,2,1)
plot(tempovet,zeropTcp(1,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('x_T_C_P [m]')
 
subplot(2,2,2)
plot(tempovet,zeropTcp(2,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('y_T_C_P [m]')
 
subplot(2,2,3)
plot(tempovet,zeropTcp(3,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('z_T_C_P [m]')
 
subplot(2,2,4)
plot3(zeropTcp(1,:),zeropTcp(2,:),zeropTcp(3,:),'-y','Linewidth',2),grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Percorso TCP')
 
%ricavo una delle 1001 configurazioni del robot mentre compie la traiettoria assegnata
 
%for riga=1:13:1001
riga=850;
qvecP=deg2rad([q1vet(riga),q2vet(riga),q3vet(riga),q4vet(riga),q5vet(riga),q6vet(riga)]);
 
figure(9)
ABBirb120_Dyn.plot([0,qvecP,0],'notiles','zoom',2) % plot del robot con i gradi di libertà assegnati
hold on
plot3(zeropCP(1,:),zeropCP(2,:),zeropCP(3,:),'-c','Linewidth',2)
plot3(zeropEE(1,:),zeropEE(2,:),zeropEE(3,:),'-m','Linewidth',2)
plot3(zeropTcp(1,:),zeropTcp(2,:),zeropTcp(3,:),'-y','Linewidth',2)
% trplot(ABBirb120.base,'frame','A0')
%end
 
 
%% DINAMICA
 
q0vet=zeros(1001,1);
 
Q=deg2rad([q0vet,q1vet,q2vet,q3vet,q4vet,q5vet,q6vet,q0vet]);
QD=deg2rad([q0vet,q1pvet,q2pvet,q3pvet,q4pvet,q5pvet,q6pvet,q0vet]);
QDD=deg2rad([q0vet,q1ppvet,q2ppvet,q3ppvet,q4ppvet,q5ppvet,q6ppvet,q0vet]);
 
GRAV=[0;0;g];
tcpFe=[0,0,forza]';
tcpMe=[0,0,0]';
 
tFet=eeAtcp(1:3,1:3)*tcpFe;
tMet=eeAtcp(1:3,1:3)*tcpMe+cross(eeAtcp(1:3,4),eeAtcp(1:3,1:3)*tcpFe);
 
FEXT=[tFet;tMet];
 
for h=1:length(tempovet)
 
[TAU(h,:),WBASE(:,h)]=ABBirb120_Dyn.rne_mdh_POLITO([Q(h,:),QD(h,:),QDD(h,:)],GRAV,FEXT');
 
end
 
% PLOTTAGGIO DELLE AZIONI MOTRICI
 
figure (10)
 
subplot(3,2,1)
plot(tempovet,TAU(:,2)),grid on
xlabel('tempo [s]')
ylabel('tau1 [Nm]')
title('azione motrice 1')
 
subplot(3,2,2)
plot(tempovet,TAU(:,3)),grid on
xlabel('tempo [s]')
ylabel('tau2 [Nm]')
title('azione motrice 2')
 
subplot(3,2,3)
plot(tempovet,TAU(:,4)),grid on
xlabel('tempo [s]')
ylabel('tau3 [Nm]')
title('azione motrice 3')
 
subplot(3,2,4)
plot(tempovet,TAU(:,5)),grid on
xlabel('tempo [s]')
ylabel('tau4 [Nm]')
title('azione motrice 4')
 
subplot(3,2,5)
plot(tempovet,TAU(:,6)),grid on
xlabel('tempo [s]')
ylabel('tau5 [Nm]')
title('azione motrice 5')
 
subplot(3,2,6)
plot(tempovet,TAU(:,7)),grid on
xlabel('tempo [s]')
ylabel('tau6 [Nm]')
title('azione motrice 6')
 
% PLOTTAGGIO DELLE AZIONI TRASMESSE A PAVIMENTO
 
 figure (11)
 
subplot(2,2,1) 
plot(tempovet,-WBASE(1,:),'-r'),grid on
xlabel('tempo [s]')
ylabel('F_p_x [N]')
 
subplot(2,2,2) 
plot(tempovet,-WBASE(2,:),'-r'),grid on
xlabel('tempo [s]')
ylabel('F_p_y [N]')
 
subplot(2,2,3) 
plot(tempovet,-WBASE(3,:),'-r'),grid on
xlabel('tempo [s]')
ylabel('F_p_z [N]')
 
 figure (12)
 
subplot(2,2,1) 
plot(tempovet,-WBASE(4,:),'-r'),grid on
xlabel('tempo [s]')
ylabel('M_p_x [Nm]')
 
subplot(2,2,2) 
plot(tempovet,-WBASE(5,:),'-r'),grid on
xlabel('tempo [s]')
ylabel('M_p_y [Nm]')
 
subplot(2,2,3) 
plot(tempovet,-WBASE(6,:),'-r'),grid on
xlabel('tempo [s]')
ylabel('M_p_z [Nm]')

