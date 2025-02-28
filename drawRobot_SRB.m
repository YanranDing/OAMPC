function fig_out = drawRobot_SRB(t, q, U, p)
U = U(:);           % U = [fx fz tau]
dcopL = U(3) / U(2);
dcopR = U(6) / U(5);

p_j = fcn_p_joints(q, p.params);

p_tor = p_j(:,1);
p_hip = p_j(:,2);
p_shoulder = p_hip + 2 * (p_tor - p_hip);
pfL = q(7:8);pfL = pfL(:);
pfR = q(9:10);pfR = pfR(:);

%% given phip + pf, calc leg angles using inverse kinematics
l1 = p.l1;
l2 = p.l2;
vL = pfL - p_hip;
vR = pfR - p_hip;
lL = norm(vL);
alpha = acos((l1^2+lL^2-l2^2)/(2*l1*lL));
pLknee = p_hip + rot(alpha) * vL / lL * l1;

lR = norm(vR);
alpha = acos((l1^2+lR^2-l2^2)/(2*l1*lR));
pRknee = p_hip + rot(alpha) * vR / lR * l1;

% --- body segments ----
r_torso = 0.05;
r_thigh = 0.025;
r_shank = 0.015;
r_foot = 0.015;

% --- center of mass
% p_CoM = fcn_CoM(q, p.params);
% drawCoM(p_CoM)
p_CoM = p_tor;
v_CoM = [q(4);q(5)];
k = 0.5;
% chain = [p_CoM,p_CoM + k * v_CoM];
% plot(chain(1,:), chain(2,:), 'color','r','linewidth',1)

% draw foot
pheelL = pfL + [-p.lheel;0];
ptoeL = pfL + [p.lfoot;0];
pheelR = pfR + [-p.lheel;0];
ptoeR = pfR + [p.lfoot;0];

%% output
fig_out.torso = drawOneLink_interface(p_shoulder, p_hip, r_torso, 'k');
fig_out.Lthigh = drawOneLink_interface(p_hip, pLknee, r_thigh, 'k');
fig_out.Rthigh = drawOneLink_interface(p_hip, pRknee, r_thigh, 'k');
fig_out.Lshank = drawOneLink_interface(pLknee, pfL, r_shank, 'k');
fig_out.Rshank = drawOneLink_interface(pRknee, pfR, r_shank, 'k');
fig_out.Lfoot = drawOneLink_interface(pheelL, ptoeL, r_foot, 'k');
fig_out.Rfoot = drawOneLink_interface(pheelR, ptoeR, r_foot, 'k');

fig_out.Lankle = drawOneLink_interface(pfL, pfL, r_shank, 'b');
fig_out.Rankle = drawOneLink_interface(pfR, pfR, r_shank, 'r');
alpha = 5e-3;
fig_out.LGRF = pfL + [dcopL;0] + [[0;0],alpha * U(1:2)];
fig_out.RGRF = pfR + [dcopR;0] + [[0;0],alpha * U(4:5)];
fig_out.ground = [[-20 20];-[r_shank r_shank]];
fig_out.vcom = [p_CoM,p_CoM + k * v_CoM];

end

function pts = drawOneLink_interface(in1, in2, in3, color)
% (p1, p2, r)
% (L, r, T)

if length(in1) == 2         % two points: (p1, p2, r)
    p1 = in1(:);
    p2 = in2(:);
    r = in3;
    
    L = norm(p1 - p2);
    pos = (p1+p2)/2;
    
    if L < 1e-4
        T = [eye(2), pos; 0, 0, 1];
    else
        vec = (p1 - p2) / L;
        c = vec(2);
        s = -vec(1);
        Rot = [c -s; s c];
        T = [Rot, pos; 0, 0, 1];
    end
    
    pts = drawOneLink(L, r, T, color);
    
elseif length(in1) == 1     % T: (L, r, T)
    L = in1;
    r = in2;
    T = in3;
    pts = drawOneLink(L, r, T, color);
else
    disp('drawOneLink input dimension error')
end


end


function pts = drawOneLink(L, r, T, color)

pts = [r;-L/2];
th = linspace(0,pi,51);
pts = [pts,[r*cos(th);r*sin(th)]+[0;L/2]];
th = linspace(pi,2*pi,51);
pts = [pts,[r*cos(th);r*sin(th)]+[0;-L/2]];
pts = [pts;ones(1,size(pts,2))];

pts = T * pts;

% plot(pts(1,:), pts(2,:), color);


end

function drawCoM(CoM)

r = 0.02;
th = linspace(0,2*pi,41);
chain = r * [cos(th);sin(th)];

hold on
plot(CoM(1)+chain(1,:),CoM(2)+chain(2,:),'k')

th = linspace(0,pi/2,21);
chain = [[0;0],[r*cos(th);r*sin(th)],[0;0]];
fill(CoM(1)+chain(1,:),CoM(2)+chain(2,:),'k')

th = linspace(pi,pi*3/2,21);
chain = [[0;0],[r*cos(th);r*sin(th)],[0;0]];
fill(CoM(1)+chain(1,:),CoM(2)+chain(2,:),'k')

end

function R = rot(a)
c = cos(a); s = sin(a);
R = [c -s; s c];
end

function [p_joints] = fcn_p_joints(q,p)

p_joints = zeros(2,6);

  p_joints(1,1)=q(1);
  p_joints(1,2)=q(1) + (p(2)*sin(q(3)))/2;
  p_joints(1,3)=q(1) + p(3)*sin(q(4) + q(3)) + (p(2)*sin(q(3)))/2;
  p_joints(1,4)=q(1) + p(3)*sin(q(4) + q(3)) + (p(2)*sin(q(3)))/2 + p(4)*sin(q(4) + q(5) + q(3));
  p_joints(1,5)=q(1) + p(3)*sin(q(6) + q(3)) + (p(2)*sin(q(3)))/2;
  p_joints(1,6)=q(1) + p(3)*sin(q(6) + q(3)) + (p(2)*sin(q(3)))/2 + p(4)*sin(q(6) + q(7) + q(3));
  p_joints(2,1)=q(2);
  p_joints(2,2)=q(2) - (p(2)*cos(q(3)))/2;
  p_joints(2,3)=q(2) - p(3)*cos(q(4) + q(3)) - (p(2)*cos(q(3)))/2;
  p_joints(2,4)=q(2) - p(3)*cos(q(4) + q(3)) - (p(2)*cos(q(3)))/2 - p(4)*cos(q(4) + q(5) + q(3));
  p_joints(2,5)=q(2) - p(3)*cos(q(6) + q(3)) - (p(2)*cos(q(3)))/2;
  p_joints(2,6)=q(2) - p(3)*cos(q(6) + q(3)) - (p(2)*cos(q(3)))/2 - p(4)*cos(q(6) + q(7) + q(3));

end
