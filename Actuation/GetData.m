data = csvread("Stair_Ascent_Kinematic_Data.csv");
D_x = data(:,1);
D_y = data(:,2);
C_x = data(:,3);
C_y = data(:,4);

%Grounded Links
A_x = -32.014*ones(301,1);
A_y = zeros(301,1);
B_x = zeros(301,1);
B_y = zeros(301,1);

theta2 = mod(atan2((C_y - A_y),(C_x - A_x)),2*pi);
theta4 = mod(atan2((D_y - B_y),(D_x - B_x)),2*pi);
theta3 = mod(atan2((D_y - C_y),(D_x - C_x)),2*pi);

A_Pos = [A_x, A_y];
B_Pos = [B_x, B_y];
C_Pos = [C_x, C_y];
D_Pos = [D_x, D_y];

% Make D the Origin
A_Pos = -D_Pos + A_Pos;
B_Pos = -D_Pos + B_Pos;
C_Pos = -D_Pos + C_Pos;
D_Pos = -D_Pos + D_Pos;

% i=1;
% while i<301
%     syms x
%     eqn = (x-B_Pos(i,1)) * (B_Pos(i,2) - D_Pos(i,2))/(B_Pos(i,1) - D_Pos(i,1)) + B_Pos(i,2) == (x-A_Pos(i,1)) * (A_Pos(i,2) - C_Pos(i,2))/(A_Pos(i,1) - C_Pos(i,1)) + A_Pos(i,2);
%     ICORx(i,1) = solve(eqn, x);
%     ICORx = double(ICORx);
%     ICORy(i,1) = (ICORx(i,1)-B_Pos(i,1)) * (B_Pos(i,2) - D_Pos(i,2))/(B_Pos(i,1) - D_Pos(i,1)) + B_Pos(i,2);
%     i=i+1;
% end

save("Stair_Ascent_Kinematic_Parameters.mat","A_Pos", "B_Pos", "C_Pos", "D_Pos", "theta2", "theta3", "theta4");
