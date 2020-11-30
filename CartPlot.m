function wy = CartPlot(q)
%% funkcja rysuje kontur robota (2,0) w konfiguracji [x0 y0 teta0]

x0 = q(1);
y0 = q(2);
fi0 = q(3);

P0 = [x0; y0];

skala = 1;

a = skala*0.13;
b = skala*0.07;
c = skala*0.05;

PL1 = [a; 0];
PL2 = [-b; c];
PL3 = [-b; -c];

R = [cos(fi0) -sin(fi0); sin(fi0) cos(fi0)];

PG1 = R*PL1 + P0;
PG2 = R*PL2 + P0;
PG3 = R*PL3 + P0;

PG = [PG1'; PG2'; PG3';PG1'];

plot(PG(:,1),PG(:,2),'LineWidth',0.8);
hold on;
plot(x0,y0,'bo','LineWidth',0.5);
