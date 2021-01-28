%% Constants
global L_Z beta_Z
theta = q(:, 1);
x = q(:, 2);
y = q(:, 3);

theta_d = q_d(:, 1);
x_d = q_d(:, 2);
y_d = q_d(:, 3);

N = size(x, 1);

%% Choose plot format for:   
%   (1) - TT (linear controller);
%   (2) - PS;
%   (3) - PS (Pomet controller, with theta_d);
chosen_plot = 2;

%% TT
switch chosen_plot
    case 1
        figure(101);
        grid on;
        hold on;

        % Z point coordinates
        x_Z = x + L_Z*cos(theta + beta_Z);
        y_Z = y + L_Z*sin(theta + beta_Z);
        
        for i=1:N
            figure(101);
            CartPlotZ([x(i); y(i); theta(i)]);
            axis([-2 2 -2 2]);
%             plot(x(1:i), y(1:i), 'k');
            plot(x_Z(1:i), y_Z(1:i),'k');
            hold on;
            plot(q_d(:, 2), q_d(:, 3), 'g:');
%             plot(q_d(:, 2), q_d(:, 3), 'go');
            grid on;
            hold off;
            pause(0.001);
        end

        figure(101);
        hold off;
        % plot(x, y, 'k');
        plot(x_Z(1:i), y_Z(1:i),'k');
        hold on;
        plot(q_d(:, 2), q_d(:, 3), 'g:');
        hold on;
        grid on;
        axis([-2 2 -2 2]);
        CartPlot([x(1); y(1); theta(1)]);
        ind = round(N/2);
        CartPlot([x(ind); y(ind); theta(ind)]);
        CartPlot([x(end); y(end); theta(end)]);

        
    case 2 
%% PS
        figure(101);
        grid on;
        hold on;

        for i=1:N
            figure(101);
            CartPlotZ([x(i); y(i); theta(i)]);
            axis([-2 2 -2 2]);
            plot(x(1:i), y(1:i), 'k');
        %     plot(x_Z(1:i), y_Z(1:i),'k');
            hold on;
        %     plot(q_d(:, 2), q_d(:, 3), 'g:');
            plot(q_d(:, 2), q_d(:, 3), 'go');
            grid on;
            hold off;
            pause(0.001);
        end

        figure(101);
        hold off;
        % plot(x, y, 'k');
        plot(x_Z(1:i), y_Z(1:i),'k');
        hold on;
        plot(q_d(:, 2), q_d(:, 3), 'g:');
        hold on;
        grid on;
        axis([-2 2 -2 2]);
        CartPlot([x(1); y(1); theta(1)]);
        ind = round(N/2);
        CartPlot([x(ind); y(ind); theta(ind)]);
        CartPlot([x(end); y(end); theta(end)]);
        
    case 3
%% Pomet  
        figure(101);
        grid on;
        hold on;

        for i=1:N
            figure(101);
            CartPlotZ([x(i); y(i); theta(i)]);
            axis([-2 2 -2 2]);
            plot(x(1:i), y(1:i), 'k');
        %     plot(x_Z(1:i), y_Z(1:i),'k');
            hold on;
        %     plot(q_d(:, 2), q_d(:, 3), 'g:');
            plot(q_d(:, 2), q_d(:, 3), 'go');
            grid on;
            hold off;
            pause(0.001);
        end

        figure(101);
        hold off;
        % plot(x, y, 'k');
        plot(x_Z(1:i), y_Z(1:i),'k');
        hold on;
        plot(q_d(:, 2), q_d(:, 3), 'g:');
        hold on;
        grid on;
        axis([-2 2 -2 2]);
        CartPlot([x(1); y(1); theta(1)]);
        ind = round(N/2);
        CartPlot([x(ind); y(ind); theta(ind)]);
        CartPlot([x(end); y(end); theta(end)]);

end