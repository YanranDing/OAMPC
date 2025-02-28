classdef OAMPC_Class
properties
    visData;
    flag;
end

methods
    %%
    function p = init_MPC(obj)
        p.dt_sim = 0.01;
        p.dt_mpc = 0.05;
        p.n_hor = 10;

        p.Tst = 0.4;
        p.Tsw = p.Tst;
        p.v0 = 0.5;
        p.vd = 0.5;
        p.z0 = 0.55;

        % MPC gain values
        Qqb = [8e3 8e4 5e3];
        Qdqb = [3e3 8 2];
        Qc = 0;

        p.Q = diag([Qqb Qdqb Qc]);
        p.Qdp = 0.1;
        p.R = diag([1e-1 1e-1 1e-1]);
        p.Ru = 0.5 * diag([1 1 1]);
        p.Rdu = diag([1 1 1]);
        p.decay = 0.98;

        % parameters
        p.mass = 12;
        p.g = 9.81;
        p.J = 0.2;
        p.mu = 1;
        p.Umax = 400;

        p.nX = 7;       % [x z th dx dz dth c]
        p.nU = 3;       % [fx, fz, tau]

        p.idx_c = 7;
        p.idx_pf = 8:11;
        p.idx_dpf = 12:15;
        p.idx_ddpf = 16:19;

        p.lheel = 0.0;
        p.lfoot = 0.0;

        p.L = 0.25;
        p.l1 = 0.24916;
        p.l2 = 0.2785;
        p.params = [p.g,p.L,p.l1,p.l2,p.mass];

        % ---------------------------
        % X = [x,z,th,dx,dz,dth,c,pf,dpf]
        Xt = [0 p.z0 0 p.v0 0 0 0 zeros(1,8)]';
        Ut = [0, p.mass*p.g 0]';

        p.Xt = Xt;
        p.Ut = Ut;

        % ------------------------------
        p.get_qp_matrices = OAMPC_formulation(p);
    end

    %%
    function p = MAIN_LOOP(obj,p)
        n_hor = p.n_hor;
        dt_sim = p.dt_sim;
        MAX_ITER = floor(p.SimDuration / dt_sim);

        % U = [fx,fz,tau]
        Xt = p.Xt;
        Ut = p.Ut;
        dc = 0;

        % --- logging ---
        tstart = 0;
        tend = dt_sim;

        [tout,Xout,Uout,Xdout,Udout,FSMout] = deal([]);

        phase = [0;0.5;0];
        pf_trans = zeros(4,1);
        p.flightLeg = 1;     % 1-left, 0-right

        for ii = 1:MAX_ITER

            % update gait params
            gait = obj.update_gait_params(tstart, phase, p);

            eta = gait.eta;
            idx = gait.idx;
            phase = gait.phase;
            t_ = gait.t_pred;

            [Xd,Ud] = obj.get_ref_traj(tstart,Xt,dc,phase,pf_trans,p);

            params = [Xt(1:p.nX);Ut(:);obj.vec(Xd(1:p.nX,:));Ud(:);eta(:);idx(:)];
            [HH,gg,AAeq,bbeq,AAineq,bbineq] = ...
                obj.getQPMatValue(p.get_qp_matrices, params,p);

            opts = optimoptions("quadprog", "Display","none");
            zval_ = quadprog(sparse(HH),gg,sparse(AAineq),bbineq,sparse(AAeq),bbeq,[],[],[],opts);
            Ut = zval_(1:p.nU);

            j = sum(eta > 0);
            dc = zval_(end-1);
            if eta(1) > 0.5
                if p.flightLeg == 1     % left touchdown
                    pf_trans(1:2) = Xt(p.idx_pf(1:2));
                    Xt(p.idx_c) = Xt(p.idx_pf(1));
                    phase = [0;0;0.5];
                    fprintf('left touchdown at %.2f s\n',tstart)
                else                    % right touchdown
                    pf_trans(3:4) = Xt(p.idx_pf(3:4));
                    Xt(p.idx_c) = Xt(p.idx_pf(3));
                    phase = [0;0.5;0];
                    fprintf('right touchdown at %.2f s\n',tstart)
                end
                p.flightLeg = 1 - p.flightLeg;
            end

            % --- time-based simulate ---
            [t,X] = ode45(@(t,X)obj.dyn_SRB(t,X,Ut,Xd,p),[tstart,tend],Xt);

            % --- update ----
            Xt = X(end,:)';
            tstart = tend;
            tend = tstart + dt_sim;

            % --- log data ---
            lent = length(t(2:end));
            tout = [tout;t(2:end)];
            Xout = [Xout;X(2:end,:)];
            if p.flightLeg == 1
                Uout = [Uout;repmat([0 0 0 Ut'],[lent,1])];
            else
                Uout = [Uout;repmat([Ut' 0 0 0],[lent,1])];
            end
            Xdout = [Xdout;repmat(Xd(:,1)',[lent,1])];
            Udout = [Udout;repmat(Ud(:,1)',[lent,1])];
        end
        toc

        % post-processing
        Xanim = Xout(:,1:10);
        Xanim(:,7:10) = Xout(:,8:11);

        obj.visData.t = tout;
        obj.visData.X = Xanim;
        obj.visData.U = Uout;
        
        p.visData = obj.visData;
    end

    %%
    function out = update_gait_params(obj,tstart,phaseIn, p)
        T = p.Tst;
        dtMPC = p.dt_mpc;
        n_hor = p.n_hor;
        dt_sim = p.dt_sim;

        [phase,phaseLeft,phaseRight] = ...
            deal(phaseIn(1),phaseIn(2),phaseIn(3));

        t_pred = tstart + dtMPC * (0:n_hor-1);
        dphase = dt_sim / T;
        eta = zeros(1,n_hor);
        if phase + dphase > 1
            eta(1) = 1;
        end
        phase = mod(phase + dphase, 1);

        % touch-down detection
        % output: bool vector indicating left/right foot touchdown
        touchdown = [0;0];
        if phaseLeft + dphase/2 >= 1 - 1e-3
            phaseLeft = phaseLeft + dphase/2 - 1;
            touchdown(1) = 1;
        else
            phaseLeft = phaseLeft + dphase/2;
        end
        
        if phaseRight + dphase/2 >= 1 - 1e-3
            phaseRight = phaseRight + dphase/2 - 1;
            touchdown(2) = 1;
        else
            phaseRight = phaseRight + dphase/2;
        end

        % contact swith within prediction horizon
        % output: eta, idx
        s_ = [phase,zeros(1,n_hor-1)];
        for kk = 2:n_hor
            ds = dtMPC / T;
            s_(kk) = s_(kk-1) + ds;
            if s_(kk) > 1
                eta(kk) = 1;
                s_(kk) = s_(kk) - 1;
            end
        end
        idx = ones(1,n_hor);
        counter = 0;
        for kk = 1:n_hor
            if eta(kk) > 0.5
                counter = counter + 1;
            end
            if counter == 2
                idx(kk) = 2;
            end
        end

        % output
        out.eta = eta;
        out.idx = idx;
        out.dtMPC = dtMPC;
        out.phase = [phase;phaseLeft;phaseRight];
        out.t_pred = t_pred;
        out.touchdown = touchdown;
    end

    %%
    function [Xd,Ud] = get_ref_traj(obj,t,Xt,dc,phase,pf_trans,p)
        n_hor = p.n_hor;
        idx_pf = p.idx_pf;
        idx_dpf = p.idx_dpf;
        idx_ddpf = p.idx_ddpf;
        stompDepth = 0.005;
        footClearance = 0.06;

        Ud = repmat([0; p.mass*p.g; 0],[1,p.n_hor]);
        Xd = repmat([0 p.z0 0 0 0 0 0 zeros(1,12)]',[1,n_hor]);

        for kk = 1:n_hor
            if kk == 1
                Xd(1,kk) = Xt(1);
            else
                Xd(1,kk) = Xd(1,kk-1) + p.vd(1) * p.dt_mpc;
            end
            Xd(2,kk) = p.z0;
            Xd(4,kk) = p.vd;
        end
        phaseLeg = phase(2:3);

        cd = Xt(p.idx_c) + dc;
        pfd = Xt(idx_pf);
        dpfd = Xt(idx_dpf);
        ddpfd = zeros(4,1);
        s_sw = [0;0];
        for i_leg = 1:2
            idx = (i_leg - 1) * 2 + (1:2);
            if phaseLeg(i_leg) > 0.5    % swing phase
                s_sw(i_leg) = (phaseLeg(i_leg) - 0.5) / 0.5;
                s_sw(s_sw > 1) = 1;
                co_x = [linspace(pf_trans(idx(1)),cd,3),cd];

                pzi = pf_trans(idx(2)) + 0.02;
                pzf = - stompDepth;
                apex = footClearance;
                co_z = [linspace(pzi,apex,3), linspace(apex,pzf,3), pzf];

                pfd(idx) = [polyval_bz(co_x,s_sw(i_leg));
                            polyval_bz(co_z,s_sw(i_leg))];
                dpfd(idx) = [polyval_bz_d(co_x,s_sw(i_leg));
                             polyval_bz_d(co_z,s_sw(i_leg))];
                ddpfd(idx) = [polyval_bz_dd(co_x,s_sw(i_leg));
                              polyval_bz_dd(co_z,s_sw(i_leg))];
            end
            Xd(idx_pf,:) = repmat(pfd,[1,n_hor]);
            Xd(idx_dpf,:) = repmat(dpfd,[1,n_hor]);
            Xd(idx_ddpf,:) = repmat(ddpfd,[1,n_hor]);
        end     % end of leg loop
    end

    %%
    function dXdt = dyn_SRB(obj,t,Xt,Ut,Xd,p)
        mass = p.mass;
        g = p.g;
        J = p.J;
        idx_pf = p.idx_pf;
        idx_dpf = p.idx_dpf;
        idx_ddpf = p.idx_ddpf;

        [x,z,px] = deal(Xt(1),Xt(2),Xt(7));
        [F,tau] = deal(Ut(1:2),Ut(3));

        ddp = F / mass + [0;-g];
        ddth = 1/J * obj.wedgeMap([px-x;-z]) * F + 1/J * tau;

        Kp_sw = 500;
        Kd_sw = 20;

        pf = Xt(idx_pf);
        dpf = Xt(idx_dpf);
        pfd = Xd(idx_pf,1);
        dpfd = Xd(idx_dpf,1);
        ddpfd = Xd(idx_ddpf,1);
        ddpf = ddpfd + Kp_sw * (pfd - pf) + Kd_sw * (dpfd - dpf);

        if p.flightLeg == 1
            dpf(3:4) = 0;
            ddpf(3:4) = 0;
        else
            dpf(1:2) = 0;
            ddpf(1:2) = 0;
        end
        dXdt = [Xt(4:6);ddp;ddth;0;dpf;ddpf];

    end

    %%
    function [Hess,Grad,Aeq,beq,Aineq,bineq] = getQPMatValue(obj,get_qp_matrices,params,p)

        [Grad,Hess,A,b,lbg,ubg] = get_qp_matrices(params);
        Hess = full(Hess);
        Grad = full(Grad);
        A = full(A);
        b = full(b);
        lbg = full(lbg);
        ubg = full(ubg);

        eq = (1 : p.nX*p.n_hor);
        ineq = ones(size(A,1),1);
        ineq(1:p.nX*p.n_hor) = 0;

        Aeq = A(eq,:);
        beq = b(eq);

        lbg_finite = (lbg ~= -inf);
        ubg_finite = (ubg ~= inf);

        idx_lb = (ineq & lbg_finite);
        idx_ub = (ineq & ubg_finite);

        Aineq = [-A(idx_lb,:);
            A(idx_ub,:)];
        bineq = [-lbg(idx_lb);
            ubg(idx_ub)] + ...
            [b(idx_lb);
            b(idx_ub)];
    end

    %%
    function out = wedgeMap(obj,in)
        out = [-in(2) in(1)];
    end

    %%
    function out = vec(obj,in)
        out = in(:);
    end

end     % end of methods
end