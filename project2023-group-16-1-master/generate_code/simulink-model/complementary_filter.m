% clc
% clear
% load("FlightData.mat")

gamma = 0.99;

h = 0.01;

alpha = (gamma * h) / (1 - gamma); 