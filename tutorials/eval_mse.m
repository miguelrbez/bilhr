%% Plot results of BILHR Tutorials
% Import the iteration, mse pairs from a text file. Name the variables
% iteration and mse.

%% Pre-define parameters
figure_name = 'Tutorial 4, Task 2, Case B, Receptive Field Size 5';
font_size = 20;


%% Create figure and plot
f = figure('Name', figure_name);
plot(iteration, mse);
xlabel('Iteration', 'FontSize', font_size);
ylabel('MSE', 'FontSize', font_size);
title(['Plot the Mean Square Error (MSE) over the Iterations for ', ...
    figure_name], 'FontSize', font_size + 2);
grid on
set(gca,'FontSize', font_size - 6)
