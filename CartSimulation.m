theta = q(:, 1);
x = q(:, 2);
y = q(:, 3);

N = size(x, 1);

figure(101);
grid on;
hold on;

for i=1:N
    figure(101);
    CartPlot([x(i); y(i); theta(i)]);
    axis([-2 2 -2 2]);
    plot(x(1:i), y(1:i), 'k--');
    grid on;
    hold off;
    pause(0.001);
end

figure(101);
hold off;
plot(x, y, 'k--');
hold on;
grid on;
axis([-2 2 -2 2]);
CartPlot([x(1); y(1); theta(1)]);
ind = round(N/2);
CartPlot([x(ind); y(ind); theta(ind)]);
CartPlot([x(end); y(end); theta(end)]);