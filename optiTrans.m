% Trajectory optimization for transition
% Author: Boyang Li
% The Hong Kong Polytechnic University
% email: boyang.li@connect.polyu.edu.hk
% Website: https://boyangli.com
% May 2018;

% Require: OptimTraj Toolbox https://github.com/MatthewPeterKelly/OptimTraj

% function [theta_cmd, throttle_cmd] = optiTrans(iniState,finState,nGrid,Time)
% solve optimal transition for tail-siiter UAV
clc; clear;

% parameters
p.mass = 1.5; % kg mass
p.g = 9.81; % m/s^2
p.rho = 1.29; % kg/m^3 density
p.Ts = 0.2; % s time constant
p.S = 0.326; % m^2 area
duration = 2; % s time to finish

% airfoid data pre-process fitting
load("NACA0012Estimation.mat");
p.pp_cl = csape(NACA_AOA,NACA_CL);
p.pp_cd = csape(NACA_AOA,NACA_CD);
% p.aoa = NACA_AOA;
% p.cl = NACA_CL;
% p.cd = NACA_CD;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up function handles                             %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.func.dynamics = @(t,x,u)( tailsitterDyna(x,u,p) );
problem.func.pathObj = @(t,x,u)( tailsitterObj(x,u,p));  
% object function cost function: change of height, throttle output?
% problem.func.pathObj = @(t,x,u)transObj(x,u,p);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Set up problem bounds                               %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = duration;
problem.bounds.finalTime.upp = duration;

% states s = [x_dot; z_dot; theta]  
problem.bounds.initialState.low = [0;0;0];
problem.bounds.initialState.upp = [0;0;0];
problem.bounds.finalState.low = [8;-0.5;-0.5*pi];
problem.bounds.finalState.upp = [inf;0.5;-7/18*pi]; %-80¡ã4/9pi -70 7/18pi

problem.bounds.state.low = [0;-1;-pi];
problem.bounds.state.upp = [15;0;pi];

% control u = [ft_cmd; theta_cmd] u0 = [0;0.5];
problem.bounds.control.low = [0;-pi];
problem.bounds.control.upp = [30;pi];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                    Initial guess at trajectory                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.guess.time = [0,duration];
problem.guess.state = [problem.bounds.initialState.low, problem.bounds.finalState.low];
problem.guess.control = [30,0;5,-4/9*pi];

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                         Solver options                                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
problem.options.nlpOpt = optimset(...
    'Display','iter-detailed',...
    'MaxFunEvals',1e5);

% problem.options.method = 'trapezoid';
% problem.options.method = 'hermiteSimpson';
% problem.options.method = 'rungeKutta';
problem.options.method = 'chebyshev';
% problem.options.method = 'gpops';
%  problem.options.chebyshev.nColPts = 50;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                            Solve!                                       %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
% NLP solve
soln = optimTraj(problem);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                        Display Solution                                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%%%% Unpack the simulation
t = linspace(soln.grid.time(1), soln.grid.time(end), 150);
z = soln.interp.state(t);
u = soln.interp.control(t);
% t = soln.grid.time;
% z = soln.grid.state;
% u = soln.grid.control;
%%%% Plots:
plot_trans(t,u,z);
