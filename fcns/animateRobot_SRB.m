function animateRobot_SRB(p)

data = p.visData;
tin = data.t;
Xin = data.X;
Uin = data.U;

if(size(tin,1) ~= length(tin))
    tin = tin';
end

if(size(Xin,1) ~= length(Xin))
    Xin = Xin';
end

if(size(Uin,1) ~= length(Uin))
    Uin = Uin';
end

playBackSpeed = p.playBackSpeed;
[t, X] = even_sample(tin, Xin, 30 * playBackSpeed);
[~, U] = even_sample(tin, Uin, 30 * playBackSpeed);

len = length(t);
multiplier = 10;
t_ = linspace(tin(1),tin(end),multiplier * len);
X_ = interp1(tin,Xin,t_);
U_ = interp1(tin,Uin,t_);
Xd_ = interp1(tin,Xin,t_);

flag_movie = ~(exist('movie.mp4'));
% flag_movie = 0;

if flag_movie
    name = ['movie'];
    vidfile = VideoWriter(name,'MPEG-4');
    open(vidfile);
end

fig = figure;
fig.Renderer = 'painters';
fig.GraphicsSmoothing = 'on';
set(gcf, 'Color', 'white')
set(gcf, 'Position',  [100, 100, 1200, 600])

mm = 3;
nn = 4;
h_main = subplot(mm,nn,[1,2,5,6]);
h3 = subplot(mm,nn,3);
h4 = subplot(mm,nn,4);
h7 = subplot(mm,nn,7);
h8 = subplot(mm,nn,8);
h9 = subplot(mm,nn,9);
h10 = subplot(mm,nn,10);
h11 = subplot(mm,nn,11);
h12 = subplot(mm,nn,12);

% torso + feet + ground + GRF + vcom
hf_main = plot(h_main,0,0,'k',0,0,'b',0,0,'r',0,0,'k',...
                0,0,'b',0,0,'r',0,0,'r',...
                0,0,'k',0,0,'k',0,0,'k',0,0,'k',0,0,'k',0,0,'k');
h_main.XLim = [-0.8 0.8];
h_main.YLim = [-0.1 1];
hf_time = text(h_main,0,0,'');
set( get(h_main,'Title'), 'String', 'SRB MPC for planar robot');

potentialEnergy = p.mass * p.g * X_(:,2);
kineticEnergy = 0.5 * X_(:,4:6).^2 * [p.mass;p.mass;p.J];
mechEnergy = potentialEnergy + kineticEnergy;
hf3 = plot(h3,0,0,'b',0,0,'r');
h3.XLim = [0 t(end)];
set( get(h3,'Title'), 'String', 'Tot mech Energy (r), PE (b)' );

hf4 = plot(h4,0,0,'b');
h4.XLim = [min(X_(:,3)) max(X_(:,3))];
h4.YLim = [min(X_(:,6)) max(X_(:,6))];
set( get(h4,'Title'), 'String', 'omega v.s. theta' );
set( get(h4,'XLabel'), 'String', 'theta [rad]' );
set( get(h4,'YLabel'), 'String', 'omega [rad/s]' );

hf12 = plot(h12,0,0,'b',0,0,'r',0,0,'b--',0,0,'r--');
h12.XLim = [0 t(end)];
set( get(h12,'Title'), 'String', 'vx(b), vz(r)' );
set( get(h12,'XLabel'), 'String', 'Time [s]' );

hf8 = plot(h8,0,0,'b',0,0,'r');
angMomentum = p.J * X_(:,6);
l_leg_L = sqrt((X_(:,7)-X_(:,1)).^2 + (X_(:,8)-X_(:,2)).^2);
l_leg_R = sqrt((X_(:,9)-X_(:,1)).^2 + (X_(:,10)-X_(:,2)).^2);
h8.XLim = [0 t(end)];
set( get(h8,'Title'), 'String', 'leg extension [m]' );
set( get(h8,'XLabel'), 'String', 'Time [s]' );

hf7 = plot(h7,0,0,'b',0,0,'r');
h7.XLim = [0 t(end)];
h7.YLim = [-100 250];
set( get(h7,'Title'), 'String', 'GRF left' );

hf11 = plot(h11,0,0,'b',0,0,'r');
h11.XLim = [0 t(end)];
h11.YLim = [-100 250];
set( get(h11,'Title'), 'String', 'GRF right' );

hf10 = plot(h10,0,0,'b',0,0,'b--');
h10.XLim = [0 t(end)];
set( get(h10,'Title'), 'String', 'z(b), zd(b--)' );

hf9 = plot(h9,0,0,'b',0,0,'b--',0,0,'r',0,0,'r--');
h9.XLim = [0 t(end)];
h9.YLim = [min(min(X_(:,1)),min(Xd_(:,1))),max(max(Xd_(:,1)),max(X_(:,1)))];
set( get(h9,'Title'), 'String', 'x(b), xd(b--)' );
set( get(h9,'YLabel'), 'String', 'CoM position [m]' );

for ii = 1:length(t)
    
    %% Robot
    fig_out = drawRobot_SRB(t(ii), X(ii,:), U(ii,:), p);
    hf_main(1).XData = fig_out.torso(1,:);
    hf_main(1).YData = fig_out.torso(2,:);
    hf_main(2).XData = fig_out.Lankle(1,:);
    hf_main(2).YData = fig_out.Lankle(2,:);
    hf_main(3).XData = fig_out.Rankle(1,:);
    hf_main(3).YData = fig_out.Rankle(2,:);
    hf_main(4).XData = fig_out.ground(1,:);
    hf_main(4).YData = fig_out.ground(2,:);
    hf_main(5).XData = fig_out.LGRF(1,:);
    hf_main(5).YData = fig_out.LGRF(2,:);
    hf_main(6).XData = fig_out.RGRF(1,:);
    hf_main(6).YData = fig_out.RGRF(2,:);
    hf_main(7).XData = fig_out.vcom(1,:);
    hf_main(7).YData = fig_out.vcom(2,:);
    hf_main(8).XData = fig_out.Lthigh(1,:);
    hf_main(8).YData = fig_out.Lthigh(2,:);
    hf_main(9).XData = fig_out.Rthigh(1,:);
    hf_main(9).YData = fig_out.Rthigh(2,:);
    hf_main(10).XData = fig_out.Lshank(1,:);
    hf_main(10).YData = fig_out.Lshank(2,:);
    hf_main(11).XData = fig_out.Rshank(1,:);
    hf_main(11).YData = fig_out.Rshank(2,:);
    hf_main(12).XData = fig_out.Lfoot(1,:);
    hf_main(12).YData = fig_out.Lfoot(2,:);
    hf_main(13).XData = fig_out.Rfoot(1,:);
    hf_main(13).YData = fig_out.Rfoot(2,:);
    
    h_main.XLim = X(ii,1) + [-0.8 0.8];
    hf_time.Position = [X(ii,1) 0.8 0];
    hf_time.String = [num2str(t(ii),'%6.2f') 's'];

    
    %% mechanical energy
    hf3(1).XData = t_(1:ii*multiplier);
    hf3(1).YData = potentialEnergy(1:ii*multiplier,1);
    hf3(2).XData = t_(1:ii*multiplier);
    hf3(2).YData = mechEnergy(1:ii*multiplier,1);

    %% dth v.s. th
    hf4.XData = X_(1:ii*multiplier,3);
    hf4.YData = X_(1:ii*multiplier,6);

    %% leg extension
    hf8(1).XData = t_(1:ii*multiplier);
    hf8(1).YData = l_leg_L(1:ii*multiplier,1);
    hf8(2).XData = t_(1:ii*multiplier);
    hf8(2).YData = l_leg_R(1:ii*multiplier,1);
    
    %% angular momentum about contact point
    hf12(1).XData = t_(1:ii*multiplier);
    hf12(1).YData = X_(1:ii*multiplier,4);
    hf12(2).XData = t_(1:ii*multiplier);
    hf12(2).YData = X_(1:ii*multiplier,5);
    hf12(3).XData = t_(1:ii*multiplier);
    hf12(3).YData = Xd_(1:ii*multiplier,4);
    hf12(4).XData = t_(1:ii*multiplier);
    hf12(4).YData = Xd_(1:ii*multiplier,5);
    
    %% GRF
    % left
    hf7(1).XData = t_(1:ii*multiplier);
    hf7(1).YData = U_(1:ii*multiplier,1);
    hf7(2).XData = t_(1:ii*multiplier);
    hf7(2).YData = U_(1:ii*multiplier,2);
        
    % right
    hf11(1).XData = t_(1:ii*multiplier);
    hf11(1).YData = U_(1:ii*multiplier,4);
    hf11(2).XData = t_(1:ii*multiplier);
    hf11(2).YData = U_(1:ii*multiplier,5);

    %% x position
    hf9(1).XData = t_(1:ii*multiplier);
    hf9(1).YData = X_(1:ii*multiplier,1);
    hf9(2).XData = t_(1:ii*multiplier);
    hf9(2).YData = Xd_(1:ii*multiplier,1);
    
    
    %% z position
    hf10(1).XData = t_(1:ii*multiplier);
    hf10(1).YData = X_(1:ii*multiplier,2);
    hf10(2).XData = t_(1:ii*multiplier);
    hf10(2).YData = Xd_(1:ii*multiplier,2);

    %%
    drawnow
    
    if flag_movie
        writeVideo(vidfile, getframe(gcf));
    end
end
% drawRobot_SRB(t(end),q(end,:),F(ii,:));

if flag_movie
    close(vidfile);
end

end
