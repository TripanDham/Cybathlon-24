% %% Initialisation
clearvars 

%Input data from 4bar Kinematics (all angles and positions of each node)
%IMPORTANT: Reference of all positions wrt node B

load("Stair_Ascent_Kinematic_Parameters.mat")
alpha3 = 0;

%Input Time Series Stair Ascent Dynamics Data
GRF = 60*9.81;
cop_x = 50.396499947; 
Friction = 0;

%Initialise Body Parameters
I_body = 0;

%% 

alpha = 20*pi/180; %inclination angle of body
theta = 45*pi/180; %knee angle
height = 1700; %human height
mass = 60; %human mass
m_upper = 0.65*mass;
m_thigh = 0.12*mass;
m_rearleg = 0.18*mass;
h_step = 200;
g = 9.81;
prosthetic_mass = 7;

% %positions of coms of 3 body segments with respect to heel
% upper_pos = height*[(-0.23*cos(theta) + 0.2*cos(alpha)), (0.25 + 0.23*sin(theta) + 0.2*sin(alpha))];
% thigh_pos = height*[-0.125*cos(theta), 0.25 + 0.125*sin(theta)];
% rearleg_pos = [height*(-0.23*cos(theta) + 0.183*cos(alpha)), height*0.317*sin(alpha) - h_step];
% 
% payload_com = (m_upper*upper_pos + m_thigh*thigh_pos + m_rearleg*rearleg_pos)/(m_rearleg + m_thigh + m_upper);
% M_hip = 1.0*mass;
% M_knee = mass*payload_com(1) - M_hip;
% 
% R_com = payload_com - D_Pos(i,:);
% 
% C = zeros(301,2);
% D = zeros(301,2);
% Patella = [130,30];
% Motor = [-100*cos(theta),100*sin(theta)];
% Shank = [-400, 0];
pulley_radius = 0.050; %metres

knee_offset = 8 * pi/180; %Offset Angle between CD link and Knee axis
    
results = [];
fileID = fopen('log.txt','w');
fprintf(fileID,'Patella_x, Patella_y, Knee Flexion, Torque\n');

for j = 100:200
    for k = 10:50
        Patella = [j,k];
        
        Tension = zeros(length(C_Pos),1);
        String_Tension = zeros(length(C_Pos),1);
        Torque = zeros(length(C_Pos),1);

        for i=40:300
            theta = atan2(D_Pos(i,2) - C_Pos(i,2), D_Pos(i,1) - C_Pos(i,1)) + knee_offset;
            upper_pos = height*[(-0.23*cos(theta) + 0.2*cos(alpha)), (0.25 + 0.23*sin(theta) + 0.2*sin(alpha))];
            thigh_pos = height*[-0.125*cos(theta), 0.25 + 0.125*sin(theta)];
            rearleg_pos = [height*(-0.23*cos(theta) + 0.183*cos(alpha)), height*0.317*sin(alpha) - h_step];
            
            payload_com = (m_upper*upper_pos + m_thigh*thigh_pos + m_rearleg*rearleg_pos)/(m_rearleg + m_thigh + m_upper);
            M_hip = 1.0*mass;
            M_knee = mass*payload_com(1) - M_hip;
            
            R_com = payload_com - D_Pos(i,:);
            Patella = [j,k];
            rotation_mat = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            Patella = Patella * rotation_mat; %Local Frame Rotation
            Motor = [-100*cos(theta),100*sin(theta)];
            Shank = [0, -400];

            C(i,1) = (GRF*cop_x + B_Pos(i,1)*Friction*tan(theta4(i)) - B_Pos(i,2)*Friction)./(A_Pos(i,2)*tan(theta2(i)) - A_Pos(i,2) - B_Pos(i,1)*tan(theta4(i)) + B_Pos(i,2));
            C(i,2) = C(i,1)*tan(theta2(i));
            D(i,1) = -Friction - C(i,1);
            D(i,2) = D(i,1)*tan(theta4(i));
            
            thetaA = atan2((Patella(2) - Motor(2)),(Patella(1) - Motor(1)));
            thetaB = 1/2* (thetaA + atan2((Patella(2)-Shank(2)),(Patella(1)-Shank(1))));
            
            Tension(i) = 1/(cross(Patella - Motor, [-cos(thetaB), -sin(thetaB)])) * (I_body*alpha3 + cross(R_com-Motor, [0,-50*9.81]) - cross(C_Pos(i,:)-Motor, C(i,:)) - cross(D_Pos(i,:) - Motor, D(i,:))- M_hip);
            String_Tension(i) = Tension(i)/(2*cos(thetaB-thetaA));
            Torque(i) = String_Tension(i)*pulley_radius;
            fprintf(fileID,'%f %f %f %f\n',j,k,theta,Torque(i));
        end
        
        results = [results ; j,k,Torque(i)];

    end
end


function tau = cross(r,f)
    tau = r(1)*f(2) - r(2)*f(1); 
end



