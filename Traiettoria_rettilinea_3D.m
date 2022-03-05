% PIANIFICAZIONE TRAIETTORIA RETTILINEA 3D

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
alfa6 = -pi/2 ;a6 = 0 ;d6 = 72e-3 ;

alfaT=0; aT=0; dT=0; tetaT0=0;
alfaB=0;aB=0;dB=0;tetaB0=0;

%Parametri tool
incl=60*pi/180;
a=0.06;
b=0.21;

% Forza di reazione dovuta al flusso di fluido uscente dall'ugello
% (intensità costante)
forza=30; % [N]

%% LIMITI DEI GRADI DI LIBERTA' DEI GIUNTI (DA CATALOGO)

qlim1=[-165 165]*pi/180;
qlim2=[-110 110]*pi/180;
qlim3=[-110 70]*pi/180;
qlim4=[-160 160]*pi/180;
qlim5=[-120 120]*pi/180;
qlim6=[-400 400]*pi/180;

%% LINK DEI ROBOT

L(1)=Link('alpha',alfa1,'a',a1,'d',d1,'modified');%,'qlim',qlim1);
L(2)=Link('alpha',alfa2,'a',a2,'d',d2,'offset',-pi/2,'modified');%,'qlim',qlim2);
L(3)=Link('alpha',alfa3,'a',a3,'d',d3,'modified');%,'qlim',qlim3);
L(4)=Link('alpha',alfa4,'a',a4,'d',d4,'modified');%,'qlim',qlim4);
L(5)=Link('alpha',alfa5,'a',a5,'d',d5,'modified');%,'qlim',qlim5);
L(6)=Link('alpha',alfa6,'a',a6,'d',d6,'modified');%,'qlim',qlim6);

%% COSTRUZIONE DEL ROBOT
ABBirb120=SerialLink(L,'name','ABBirb120');

%% MATRICE seiAtool

eeAtcp=[cos(incl) 0 -sin(incl) -a;
    0 1 0 0;
    sin(incl) 0 cos(incl) b;
    0 0 0 1];
ABBirb120.tool=eeAtcp;


%%
qvec=[0 0 0 0 0 0];
[zeroAtcp, A_all]=ABBirb120.fkine(qvec);
zeroA1=A_all(:,:,1);
zeroA2=A_all(:,:,2);
zeroA3=A_all(:,:,3);
zeroA4=A_all(:,:,4);
zeroA5=A_all(:,:,5);
zeroA6=A_all(:,:,6);
zeroAtcp;
% RobotIRB120.plot(qvec)

%% SPECIFICHE TRAIETTORIA RETTILINEA
deltax=0.08*(-1);
deltay=0.05*(+4);
deltaz=0.03*(+1);
spost_compl=sqrt(deltax^2+deltay^2+deltaz^2);
teta=asin(deltaz/spost_compl);
gamma=asin(deltay/(spost_compl*cos(teta)));


% [s,s_p,s_pp,tempo]=forma_cicloidale;
[s,s_p,s_pp,tempo]=forma_polinomiale345;
figure(1)
plot(tempo,s,tempo,s_p,tempo,s_pp),grid on
xlabel('tempo [s]')
ylabel('Parametro di curva e sue derivate')
legend('s[-]','s_p[s^-^1]','s_p_p[s^-^2]')


zeroAtcp_tempo(:,:,1)=zeroAtcp;
INVE(:,:,1)=inv(eeAtcp);
zeroA6(:,:,1)=zeroAtcp_tempo(:,:,1)*INVE(:,:,1);

for i=1:(length(tempo)-1)
    
    zeroAtcp_tempo(:,:,i+1)=zeroAtcp;
    zeroAtcp_tempo(:,4,i+1)=zeroAtcp(:,4)+s(i)*[deltax deltay deltaz 0]';
    INVE(:,:,i+1)=inv(eeAtcp);
    zeroA6(:,:,i+1)=zeroAtcp_tempo(:,:,i+1)*INVE(:,:,i+1);
    
end



%% CALCOLO DELLE 8 COMBINAZIONI PER PER OGNI ISTANTE DI TEMPO (2s)

for i=1:length(tempo)
    sol6gdl(:,:,i)=cin_inversa(zeroA6(:,:,i));
end

%% CERNITA DEI VETTORI Q

RIGAESATTA=2;
GRADI_LIBERTA(1,:)=sol6gdl(RIGAESATTA,:,1)
for i=1:(length(tempo)-1)
    for j=1:8
        vettore_norme(j)=norm(sol6gdl(j,:,i+1)-sol6gdl(RIGAESATTA,:,i));
        
    end
    [val index]=min(vettore_norme);
    RIGAESATTA=index;
    GRADI_LIBERTA(i+1,:)=sol6gdl(RIGAESATTA,:,i+1);
    
    
end
%%

% Vettori gradi di libertà dei giunti nel tempo

q1=GRADI_LIBERTA(:,1);
q2=GRADI_LIBERTA(:,2);
q3=GRADI_LIBERTA(:,3);
q4=GRADI_LIBERTA(:,4);
q5=GRADI_LIBERTA(:,5);
q6=GRADI_LIBERTA(:,6);



%% Calcolo della jacobiana e delle velocità qp

for i=1:length(tempo)
    
J(:,:,i)=ABBirb120.jacob0([q1(i) q2(i) q3(i) q4(i) q5(i) q6(i)]);
V(:,i)=[s_p(i)*cos(teta)*cos(gamma);s_p(i)*cos(teta)*sin(gamma); s_p(i)*sin(teta); 0; 0; 0];
qp(:,i)=inv(J(:,:,i))*V(:,i);

end


q1p=qp(1,:)';
q2p=qp(2,:)';
q3p=qp(3,:)';
q4p=qp(4,:)';
q5p=qp(5,:)';
q6p=qp(6,:)';

% PLOTTAGGIO VETTORE DI VELOCITA' GENERALIZZATA

figure(2)

subplot(2,1,1)
plot(tempo,V(1,:),tempo,V(2,:),tempo,V(3,:),tempo,sqrt(V(1,:).^2+V(2,:).^2+V(3,:).^2)),grid on
xlabel('tempo [s]');
title('Velocità Tool Centre Point v_T')
legend('v_T_x [m/s]','v_T_y [m/s]','v_T_z [m/s]','|v_T| [m/s]')

subplot(2,1,2)
plot(tempo,V(4,:),tempo,V(5,:),tempo,V(6,:)),grid on
xlabel('tempo [s]');
title('Velocità angolare Tool w_T')
legend('w_T_x [rad/s]','w_T_y [rad/s]','w_T_z [rad/s]')

%%

for i=1:length(tempo)
    [zeroAtcp, A_all]=ABBirb120.fkine([q1(i) q2(i) q3(i) q4(i) q5(i) q6(i)]);
    
    zeropCP(:,i)=A_all(1:3,4,4);
    zeropEE(:,i)=A_all(1:3,4,6);
    zeropTcp(:,i)=zeroAtcp(1:3,4);
end
%% PLOTTAGGIO TRAIETTORIA E PERCORSO CP

figure (3)

subplot(2,2,1)
plot(tempo,zeropCP(1,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('x_C_P [m]')


subplot(2,2,2)
plot(tempo,zeropCP(2,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('y_C_P [m]')


subplot(2,2,3)
plot(tempo,zeropCP(3,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('z_C_P [m]')


subplot(2,2,4)
plot3(zeropCP(1,:),zeropCP(2,:),zeropCP(3,:),'-k'),grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Percorso CP')

%% PLOTTAGGIO TRAIETTORIA E PERCORSO EE

figure (4)

subplot(2,2,1)
plot(tempo,zeropEE(1,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('x_E_E [m]')


subplot(2,2,2)
plot(tempo,zeropEE(2,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('y_E_E [m]')


subplot(2,2,3)
plot(tempo,zeropEE(3,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('z_E_E [m]')


subplot(2,2,4)
plot3(zeropEE(1,:),zeropEE(2,:),zeropEE(3,:),'-g'),grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Percorso EE')

%% PLOTTAGGIO TRAIETTORIA E PERCORSO TCP
figure (5)

subplot(2,2,1)
plot(tempo,zeropTcp(1,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('x_T_C_P [m]')


subplot(2,2,2)
plot(tempo,zeropTcp(2,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('y_T_C_P [m]')


subplot(2,2,3)
plot(tempo,zeropTcp(3,:),'-b'),grid on
xlabel('tempo [s]')
ylabel('z_T_C_P [m]')


subplot(2,2,4)
plot3(zeropTcp(1,:),zeropTcp(2,:),zeropTcp(3,:),'-r'),grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Percorso TCP')

%% PLOT ROBOT IN MOVIMENTO DURANTE TRAIETTORIA

% figure (6)
figure('units','normalized','outerposition',[0 0 1 1])% Maximizes the figure window for the figure with handle 5
    
    plot3(zeropCP(1,:),zeropCP(2,:),zeropCP(3,:),'-k','Linewidth',2),grid on
    view(80,50);
    hold on
    plot3(zeropEE(1,:),zeropEE(2,:),zeropEE(3,:),'-g','Linewidth',2),grid on
    plot3(zeropTcp(1,:),zeropTcp(2,:),zeropTcp(3,:),'-r','Linewidth',2),grid on
    trplot(ABBirb120.base,'frame','A0')
    ABBirb120.plot([q1(1:10:end) q2(1:10:end) q3(1:10:end) q4(1:10:end) q5(1:10:end) q6(1:10:end)],'notiles','loop','zoom',3)
    

    



