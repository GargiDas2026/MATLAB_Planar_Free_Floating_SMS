%% This matlab code is to test the FK-code %%
% Define the joint angles of the SMS
q = [0; 0; 0] * pi/180;  % joint angle - q is converted in radian

% Calculate the end-effector position 
ee_position = Forward_kinematics_planar_FFSMS(q);
% Finally display the EE-position 
fprintf('The end-effector position is: [%.4f, %.4f, %.4f]\n', ee_position(1),...
    ee_position(2), ee_position(3));