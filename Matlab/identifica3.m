clear all
close all
clc
T=0.01;
load dados1; %utilizado para validação
valid=aq1;
load dados1; %utilizado para encontrar o modelo
%load afericao;
%load aquisicao_controle;
%load entrada_saida;
%dados = amostras;
%load entrada;
%input=valores;

format long
%% Descobrindo o modelo
u=aq1(230:end-150,2);
dy=-aq1(230:end-150,1)/1000;
%plot(u,'b');
%hold on
%plot(dy,'r*');
%hold off
%break
psi=[-dy(3:end-1) -dy(2:end-2) -dy(1:end-3) u(3:end-1) u(2:end-2) u(1:end-3)];
Y=dy(4:end);
theta=(psi'*psi)\(psi'*Y);
numd=[theta(4) theta(5) theta(6)];
dend=[1 theta(1) theta(2) theta(3)];
dGz=tf(numd,dend,T);
[nG,dG]=tfdata(dGz,'v');

%entrada=[[T:T:size(u(:,1))*T]',u(:,1)];
%saida=[[T:T:size(dy(:,1))*T]',dy(:,1)];
entrada=[[T:T:size(valid(230:end-150,2))*T]',valid(230:end-150,2)];
saida=[[T:T:size(valid(230:end-150,1))*T]',-valid(230:end-150,1)/1000];

nG=sum(nG);
G=tf(nG,dG,T);
%rltool(G*tf(1,[1 0],T));
cG=d2c(G,'Tustin')
[num,den] = tfdata(cG,'v');
num=num(end);
cG=tf(num,den);
%rltool(cG*tf(1,[1 0]));
%break;
nC =[0.12^2 0.21 1];
dC=conv([0.065 1],[ 0.015 1]);
C=tf(nC,dC);
[nCd,dCd]=tfdata(c2d(C,T),'v');
%rltool(dGz);
% sim('modelo');
%%
aux=roots(den);
polo=aux(end);
k = num/den(end);
ksi=.75;%6.24;
wn=7;%43.5
kp=(1/k)*wn^2;
kd=(1/k)*(2*ksi*wn+polo);
kp=-115;
kd=-33;

%% controle
beta=15;%20
gama=0.7;%0.75
KP=18;%20
KD=2;%6
KI=0;
 if 0
     e(1)=-1;
     for k= 2:2000
         e(k)=e(k-1)+0.001;
         %sig(k)=(1-1/(1+exp(beta*(-abs(e(k))+gama))));
         sig(k)=1/(1+exp(beta*(-abs(e(k))+gama)));
     end
     figure(1)
     plot(e,sig);
     title({'sigmoid function \beta = ',beta,' \gamma =',gama},'Interpreter','tex');
     xlabel('% error');
     ylabel('sigmoid value');
     %break;
 end
sim('simula');
break;
% KPr=10;
% KDr=1;
% KPl=10;
% KDl=-10;


%sim('modelo');
%% compensador de disturbio

Tc = T;

C=tf(KP*[KD/KP 1],1);
N=3;
Wn=0.05;
Wn=0.3;
[B,A]=butter(N,Wn);
Q=tf(B,A,Tc);
[nQ,dQ] = tfdata(Q,'v');
nQ=sum(nQ);
Q=tf(nQ,dQ,Tc);

dGz = c2d(cG,Tc);
[nG, dG]=tfdata(dGz,'v');
nG = sum(nG)
QinvG=tf((nQ*dG),(dQ*nG),Tc)
nQG=nQ*dG;
dQG=dQ*nG;



%break

sim('modelo');




