clear all; close all; clc;

%% Robot ideale
robot = loadrobot("abbIrb120", "DataFormat", "column");
ee = robot.Bodies{end}.Name;

q = homeConfiguration(robot);
T0 = getTransform(robot, q, ee);

p0  = tform2trvec(T0);
eul = tform2eul(T0);
ee0 = [p0, eul]';

showdetails(robot)

%% Robot reale
robot_real = copy(robot);
ee_r = robot_real.Bodies{end}.Name;

% Parametri di incertezza
sigma_pos = 0.01;   % [m] incertezza geometrica (1 mm)
sigma_m   = 0.1;    % ±5% massa
sigma_I   = 0.2;    % ±10% inerzia

% Incertezza cinematica (lunghezze / offset dei link)
for i = 1:length(robot_real.Bodies)
    joint = robot_real.Bodies{i}.Joint;
    T = joint.JointToParentTransform;
    T(1:3,4) = T(1:3,4) + sigma_pos * randn(3,1);
    setFixedTransform(joint, T);
end

% Incertezza dinamica (masse e inerzie)
for i = 1:length(robot_real.Bodies)
    robot_real.Bodies{i}.Mass = robot_real.Bodies{i}.Mass * (1 + sigma_m * randn);
    robot_real.Bodies{i}.Inertia = robot_real.Bodies{i}.Inertia .* (1 + sigma_I * randn(1,6));
end

q_real = homeConfiguration(robot_real);
T0_real = getTransform(robot_real, q_real, ee_r);

p0_real  = tform2trvec(T0_real);
eul_real = tform2eul(T0_real);
ee0_real = [p0_real, eul_real]';


