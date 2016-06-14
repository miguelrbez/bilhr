%% Plot results of BILHR Tutorials
% Import the iteration, mse pairs from a text file. Name the variables
% iteration and mse.

%% Pre-define parameters
figure_name = 'Tutorial 4, Task 4, Training comparison between MLP and CMAC';
font_size = 20;


%% Create figure and plot
f = figure('Name', figure_name);
hold on
plot(iteration_2(1:150), mse_2(1:150));
plot(iteration_3(1:150), mse_3(1:150));
plot(iteration_a_3, mse_a_3);
plot(iteration_a_5, mse_a_5);
plot(iteration_b_3, mse_b_3);
plot(iteration_b_5, mse_b_5);
hold off
legend(	'Tutorial 3 MLP, 34 TS, Task 2', ...
		'Tutorial 3 MLP, 34 TS, Task 3', ...
		'Tutorial 4 CMAC, 75 TS, 3x3 PF', ...
		'Tutorial 4 CMAC, 75 TS, 5x5 PF', ...
		'Tutorial 4 CMAC, 150 TS, 3x3 PF', ...
		'Tutorial 4 CMAC, 150 TS, 5x5 PF');
xlabel('Iteration', 'FontSize', font_size);
ylabel('MSE', 'FontSize', font_size);
title(['Plot the Mean Square Error (MSE) over the Iterations for ', ...
    figure_name], 'FontSize', font_size + 2);
grid on
set(gca,'FontSize', font_size - 6)
