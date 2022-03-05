function [sol6gdl] = cin_inversa(zeroA6)


alfa1 = 0 ;a1 = 0 ; d1 = 290e-3 ;
alfa2 = -pi/2 ;a2 = 0 ;d2 = 0 ;
alfa3 = 0 ; a3 = 270e-3 ;d3 = 0 ;
alfa4 = -pi/2 ;a4 = 70e-3 ;d4 = 302e-3 ;
alfa5 = pi/2 ; a5 = 0 ; d5 = 0 ;
alfa6 = -pi/2 ;a6 = 0 ;d6 = 72e-3 ;

incl=60*pi/180;
a=0.06;
b=0.21;

qlim1=[-165 165]*pi/180;
qlim2=[-110 110]*pi/180;
qlim3=[-110 70]*pi/180;
qlim4=[-160 160]*pi/180;
qlim5=[-120 120]*pi/180;
qlim6=[-400 400]*pi/180;

L(1)=Link('alpha',alfa1,'a',a1,'d',d1,'modified');%,'qlim',qlim1);
L(2)=Link('alpha',alfa2,'a',a2,'d',d2,'offset',-pi/2,'modified');%,'qlim',qlim2);
L(3)=Link('alpha',alfa3,'a',a3,'d',d3,'modified');%,'qlim',qlim3);
L(4)=Link('alpha',alfa4,'a',a4,'d',d4,'modified');%,'qlim',qlim4);
L(5)=Link('alpha',alfa5,'a',a5,'d',d5,'modified');%,'qlim',qlim5);
L(6)=Link('alpha',alfa6,'a',a6,'d',d6,'modified');%,'qlim',qlim6);

RobotIRB120=SerialLink(L,'name','RobotIRB120');

seiAtcp=[cos(incl) 0 -sin(incl) -a;
    0 1 0 0;
    sin(incl) 0 cos(incl) b;
    0 0 0 1];
RobotIRB120.tool=seiAtcp;

% preallocazione matrici soluzioni braccio 3gdl e robot 6gdl completo
solbr=NaN(4,3);
sol6gdl=NaN(8,6);

% calcolo posizione centro polso nota matrice sr6 wrt sr0
pCP0=zeroA6(1:3,4)-d6*zeroA6(1:3,3);
xCP=pCP0(1); yCP=pCP0(2); zCP=pCP0(3);

% soluzione cinematica inversa braccio articolato 3gdl
% q1
q11=atan2(yCP,xCP)-atan2(0,1);
q12=atan2(yCP,xCP)-atan2(0,-1);
% q3
Bcost=(xCP^2+yCP^2+(zCP-d1)^2-a4^2-d4^2-a3^2)/(2*a3);
q31=atan2(a4,d4)-atan2(Bcost,sqrt(a4^2+d4^2-Bcost^2));
q32=atan2(a4,d4)-atan2(Bcost,-sqrt(a4^2+d4^2-Bcost^2));
% q2
solbr=[q11 NaN q31;
       q11 NaN q32;
       q12 NaN q31;
       q12 NaN q32];
for i=1:4
    q1=solbr(i,1);
    q3=solbr(i,3);

    s2sys=((cos(q1)*xCP+sin(q1)*yCP)*(cos(q3)*a4-sin(q3)*d4+a3)-(zCP-d1)*(sin(q3)*a4+cos(q3)*d4))/...
        ((cos(q3)*a4-sin(q3)*d4+a3)^2+(sin(q3)*a4+cos(q3)*d4)^2);
    c2sys=((cos(q1)*xCP+sin(q1)*yCP)*(sin(q3)*a4+cos(q3)*d4)+(zCP-d1)*(cos(q3)*a4-sin(q3)*d4+a3))/...
        ((cos(q3)*a4-sin(q3)*d4+a3)^2+(sin(q3)*a4+cos(q3)*d4)^2);
    q2=atan2(s2sys,c2sys);
    solbr(i,:)=[q1 q2 q3];
end

% soluzione cinematica inversa polso sferico 3gdl
for i=1:4

    % calcolo matrice polso sr6 wrt sr3
    q1=solbr(i,1);
    q2=solbr(i,2);
    q3=solbr(i,3);
    A3=RobotIRB120.A([1 2 3],[q1 q2 q3]);	% sr3 wrt sr0
    A63o=inv(A3)*zeroA6;              % matrice polso sr6 wrt sr3

    % soluzione cinematica inversa polso sferico 3gdl
    if A63o(2,3)==1
        q51=0;
        q41=0;
        q61=atan2(-A63o(1,2),A63o(1,1))-q41;
        multi=0;
    elseif A63o(2,3)==-1
        q51=pi;
        q41=0;
        q61=q41-atan2(-A63o(1,2),-A63o(1,1));
        multi=0;
    else
        q51=acos(A63o(2,3));
        q41=atan2(A63o(3,3)/sin(q51),-A63o(1,3)/sin(q51));
        q61=atan2(-A63o(2,2)/sin(q51),A63o(2,1)/sin(q51));
        q52=-acos(A63o(2,3));
        q42=atan2(A63o(3,3)/sin(q52),-A63o(1,3)/sin(q52));
        q62=atan2(-A63o(2,2)/sin(q52),A63o(2,1)/sin(q52));
        multi=1;
    end

    sol6gdl(i,1:6)=[q1 q2 q3 q41 q51 q61];
    if multi==1
        sol6gdl(i+4,1:6)=[q1 q2 q3 q42 q52 q62];
    end
end

end

