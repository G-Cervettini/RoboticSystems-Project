function [Y,Yp,Ypp,tau]=forma_cicloidale;

tau=linspace(0,1,100);
Y=tau-(sin(2*pi*tau))/(2*pi);
Yp=1-cos(2*pi*tau);
Ypp=2*pi*sin(2*pi*tau);

end

