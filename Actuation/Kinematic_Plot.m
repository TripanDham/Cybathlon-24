load("Stair_Ascent_Kinematic_Parameters.mat")

i=1;

while i<300
    plot([A_Pos(i,1), B_Pos(i,1), D_Pos(i,1), C_Pos(i,1), A_Pos(i,1)], [A_Pos(i,2), B_Pos(i,2), D_Pos(i,2), C_Pos(i,2), A_Pos(i,2)] )
    hold on
%     scatter(ICORx(i,1), ICORy(i,1))
%     axis([25 125 500 600])
    i=i+1;
    pause(0.1)
    hold off
end
