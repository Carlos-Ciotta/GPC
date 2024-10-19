function [G, K1, F, Qd, Ql] = inicializacao_GPC(atraso, numerador_discreto, denominador_discreto, horizonte_pred, lambda, delta)
%numerador_discreto = [0.004419 0.004419];
%denominador_discreto = [1 -0.9994];
%G = tf(numc,denc);
%Gd = c2d(G, 0.1,'tustin');
%atraso = 0;
%horizonte_pred =3;

N2=atraso+horizonte_pred;   % Final Horizon
Nu=horizonte_pred;   % Input Horizon

Ql=eye(Nu)*lambda;
Qd=eye(Nu)*delta;

[En,F] = diophantine(denominador_discreto,N2,0); % Diophantine function
E = En(end,:);
F=F(1:horizonte_pred,1:end);

if numerador_discreto(1)==0
        numerador_discreto=numerador_discreto(2:end);
end

g=conv(E,numerador_discreto); %Calcula polinomio g
G=zeros(horizonte_pred,Nu);                       % Inicializa la matriz G
for k=1:Nu
    G(k:end,k)=g(1:Nu-k+1);         % Forma a matriz G                  
end

aux=inv(G'*Qd*G+Ql)*G'; %Calculo de la Funcion de Costo sin Restriccion
%Calculo do controldor K1 (Primeira fila de Mn)
K1=aux(1,:);
end
