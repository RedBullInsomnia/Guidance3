
clc;
clear all;

%% Initialization

figure(1);
clf();
hold on;

waypoints = waypoints_collection;

legend_labels = {};

%% Plots

waypoints.plot_markers('k');
legend_labels = {legend_labels{:} 'Waypoints'};

waypoints.plot_continuous_interpolation('r');
legend_labels = {legend_labels{:} 'Cubic Hermite interpolation'};

waypoints.plot_piecewise_continuous('b');
legend_labels = {legend_labels{:} 'Piecewise continuous interpolation'};

waypoints.plot_circles('b');
legend_labels = {legend_labels{:} 'Circles'};

%% Figure properties

legend(legend_labels);

ylabel('Position, north');
xlabel('Postition, east');

axis equal;
