function [s,s_p,s_pp,tempo]=forma_polinomiale345;

tempo=linspace(0,2,1001);
s=   10*(tempo./2).^3-15*(tempo./2).^4+6*(tempo./2).^5;
s_p= 30*(tempo./2).^2-60*(tempo./2).^3+30*(tempo./2).^4;
s_pp=60*(tempo./2)-180*(tempo./2).^2+120*(tempo./2).^3;

end

