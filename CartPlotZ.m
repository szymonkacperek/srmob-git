function wy = CartPlotZ(we)
%% funkcja rysuje kontur robota (2,0) w konfiguracji [x0 y0 teta0] i pozycj� punktu Z

global L_Z beta_Z;

x0 = we(1);
y0 = we(2);
teta0 = we(3);

%-------------------------------------------------------------------------
if isempty(L_Z)
    L_Z = 0;
end
if isempty(beta_Z)
    beta_Z = 0;
end
%-------------------------------------------------------------------------
P0 = [x0; y0];

skala = 1.5;

a = skala*0.13;
b = skala*0.07;
c = skala*0.05;
D = 0.1;    %rozstaw k�
r = 0.025;   %promie� k�

%obliczenie wsp�rz�dnych �rodk�w k�:
D0x = [x0-0.5*D*sin(teta0); x0+0.5*D*sin(teta0)];
D0y = [y0+0.5*D*cos(teta0); y0-0.5*D*cos(teta0)];
%obliczenie wsp�rz�dnych ko�c�w k�:
Kp0x = [D0x(1)+r*cos(teta0); D0x(1)-r*cos(teta0)];
Kp0y = [D0y(1)+r*sin(teta0); D0y(1)-r*sin(teta0)];
Kl0x = [D0x(2)+r*cos(teta0); D0x(2)-r*cos(teta0)];
Kl0y = [D0y(2)+r*sin(teta0); D0y(2)-r*sin(teta0)];

PL1 = [a; 0];
PL2 = [-b; c];
PL3 = [-b; -c];
ZL1 = [L_Z*cos(beta_Z); L_Z*sin(beta_Z)];

R = [cos(teta0) -sin(teta0); sin(teta0) cos(teta0)];

PG1 = R*PL1 + P0;
PG2 = R*PL2 + P0;
PG3 = R*PL3 + P0;
ZG1 = R*ZL1 + P0;

PG = [PG1'; PG2'; PG3'; PG1'];
ZG = [P0';ZG1'];

plot(PG(:,1),PG(:,2),'b','LineWidth',0.8);
hold on;
plot(ZG(:,1),ZG(:,2),'r','LineWidth',0.8);
plot(x0,y0,'bo','LineWidth',0.5);
plot(ZG1(1),ZG1(2),'ro','LineWidth',0.5);


plot(D0x,D0y,'k','LineWidth',0.8);               %rysowanie osi k�
plot(Kp0x,Kp0y,'k','LineWidth',1.3);             %rysowanie ko�a P
plot(Kl0x,Kl0y,'k','LineWidth',1.3);             %rysowanie ko�a L





