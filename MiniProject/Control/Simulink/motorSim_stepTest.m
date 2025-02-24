%% Runmotorsim.m
% This script runs a simulation of a motor and plots the results
%
% required file: motor_control.slx
%
%% Define motor parameters
K = 1.5; % DC gain [rad/Vs]
sigma = 14; % time constant reciprocal [1/s]
Kp = 1.75; % proportional feedback gain
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('motor_control')
load('stepData1.mat')
%
% run the simulation
%
out=sim('motor_control');
%% A Plot of the results
%
figure
subplot(2,1,1)
plot(out.Voltage, '--', 'linewidth', 2)
hold on
plot(data1(:, 1), data1(:, 2), 'linewidth', 2)
legend('Simulated', 'Experimental', 'location', 'southeast')
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
subplot(2,1,2)
plot(out.Velocity,'linewidth',2)
hold on
plot(out.DesiredVelocity,'--','linewidth',2)

%calculate standard deviation for voltage
sd = std(out.Voltage);