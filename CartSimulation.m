%% Constants
global L_Z beta_Z
theta = q(:, 1);
x = q(:, 2);
y = q(:, 3);
N = size(x, 1);

%% Choose plot format for:   
%   (1) - TT (for linear controller)
%   (2) - PS (for linear controller)
%   (3) - PS (for Pomet controller & VFO, with theta_d)
%   (4) - TT (VFO)
chosen_plot = 3;

%% TT
switch chosen_plot
    case 1        
        theta_d = q_d(:, 1);
        x_d = q_d(:, 2);
        y_d = q_d(:, 3);
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
        CartPlotZ([x(1); y(1); theta(1)]);
        ind = round(N/2);
        CartPlotZ([x(ind); y(ind); theta(ind)]);
        CartPlotZ([x(end); y(end); theta(end)]);

        
    case 2 
%% PS
        theta_d = q_d(:, 1);
        x_d = q_d(:, 2);
        y_d = q_d(:, 3);
        figure(101);
        grid on;
        hold on;

        for i=1:N
            figure(101);
            CartPlot([x(i); y(i); theta(i)]);
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
        % desired point coordinates
        theta_d = q_d(1, 1);
        x_d = q_d(1, 2);
        y_d = q_d(1, 3);
        
        % point needed to plot straight line showing theta_d angle
        x_beta_begin = x_d - 1;
        y_beta_begin = tan(theta_d)*x_beta_begin + (y_d - tan(theta_d)*x_d);
        x_beta_end = x_d + 1;
        y_beta_end = tan(theta_d)*x_beta_end + (y_d - tan(theta_d)*x_d);
        
        figure(101);
        grid on;
        hold on;

        for i=1:N
            figure(101);
            plot(q_d(1, 2), q_d(1, 3), 'go', 'Linewidth', 0.9);
            hold on;
            plot([x_beta_begin x_d x_beta_end], [y_beta_begin y_d y_beta_end], 'g--');
            hold on;
            plot(x_beta_end, y_beta_end, 'k.');
            hold on;
            CartPlotZ([x(i); y(i); theta(i)]);
            axis([-2 2 -2 2]);
            plot(x(1:i), y(1:i), 'k');
            hold on;
            grid on;
            hold off;
            pause(0.001);
        end

        figure(101);
        hold off;
        % plot(x, y, 'k');
        hold on;
        plot(q_d(:, 2), q_d(:, 3), 'g:');
        hold on;
        grid on;
        axis([-2 2 -2 2]);
        CartPlot([x(1); y(1); theta(1)]);
        ind = round(N/2);
        CartPlot([x(ind); y(ind); theta(ind)]);
        CartPlot([x(end); y(end); theta(end)]);
    
    case 4        
%% VFO, TT        
        theta_d = q_d(:, 1);
        x_d = q_d(:, 2);
        y_d = q_d(:, 3);
        figure(101);
        grid on;
        hold on;
        
        for i=1:N
            figure(101);
            CartPlot([x(i); y(i); theta(i)]);
            axis([-2 2 -2 2]);
            plot(x(1:i), y(1:i), 'k', 'Linewidth', 1.5);
            hold on;
            plot(q_d(:, 2), q_d(:, 3), 'g--');
            grid on;
            hold off;
            pause(0.001);
        end

        figure(101);
        hold off;
        plot(x, y, 'k');
        hold on;
        plot(q_d(:, 2), q_d(:, 3), 'g--');
        hold on;
        grid on;
        axis([-2 2 -2 2]);
        CartPlot([x(1); y(1); theta(1)]);
        ind = round(N/2);
        CartPlot([x(ind); y(ind); theta(ind)]);
        CartPlot([x(end); y(end); theta(end)]);
end