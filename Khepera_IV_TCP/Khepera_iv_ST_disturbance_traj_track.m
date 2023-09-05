clear; close all;

port=3000;
host_address="192.168.1.202";

%% Simulation Parameters

Ts=0.15;
b=0.1;    %b distance for FL
r=0.0205; %wheels' radius
d=0.053;  %wheels' axis length

%Linearized system
Ad = eye(2);
Bd = eye(2)*Ts;
model = LTISystem('A',Ad,'B',Bd,'Ts',Ts);


%Worst-case squared input set computation
max_wheels_speed=310;
max_speed_rad=0.813/0.0210;
wrlmax=max_wheels_speed*max_speed_rad/1200;

r_u=(2*wrlmax*r*b)/(sqrt(4*b^2+d^2));
side=r_u*sqrt(2);
Qu=r_u^2*eye(2);% disturbance Shaping matrix



%Generation of xr, yr, tehtar, xpr, ypr, xppr, yppr defining the reference
%trajectory
eight_traj_generation
%circular_traj_generation
% Z_traj_generation

%control effort matrices
R=0*eye(2);
Rd=0*eye(2);




%%Bound on the disturbance
disturbance=zeros(2,length(xr));
disturbance_norm=zeros(1,length(xr));
for i=1:length(xr)
    T_FL_i=[cos(thetar(i)) sin(thetar(i)); -sin(thetar(i))/b cos(thetar(i))/b];
    disturbance(1:2,i)=inv(T_FL_i)*[vr(i);wr(i)];
    
    
    disturbance_norm(i)=disturbance(:,i)'*disturbance(:,i);
end


rd=sqrt((max(disturbance_norm)));

Qd=rd^2*eye(2);% disturbance Shaping matrix

return

Nsets=700;
Q0=Bd'*Qd*Bd;

plot_sets=false;
if plot_sets
    ell_0=ellipsoid(Q0);
    figure
    plot(ell_0)
    grid
    hold on
    axis([-0.8 0.8 -0.8 0.8])
end



Qcurr=Q0;
Q_k=Q0;

for i=1:Nsets
    
    Qi=one_step_ellipsoidal_reachable_set(Ad,Bd,Bd,Qcurr,Qu,Qd);
    
    if plot_sets
        ell_i=ellipsoid(Qi);
        plot(ell_i,'b')
    end
    
    
    pause(0.00001)
    Q_k=[Q_k Qi];
    Qcurr=Qi;
    
end


N_step_sim=length(xr);
index_seq=[];
param=ST_MPC_disturbance_control_parameters(Ts,b,R,Rd,Q_k,index_seq);
                                           


%Creating the Khepera IV object
khep=Khepera4(host_address ,port,[x0;y0;theta0],param,max_wheels_speed);



khep.x_r=[xr xr(end)*ones(1,100)];
khep.y_r=[yr yr(end)*ones(1,100)];
khep.xp_r=[xpr xpr(end)*ones(1,100)];
khep.yp_r=[ypr ypr(end)*ones(1,100)];
khep.xpp_r=[xppr xppr(end)*ones(1,100)];
khep.ypp_r=[yppr yppr(end)*ones(1,100)];
khep.theta_r=thetar;

%Connecting the robot through TCP
[khep,result]=khep.connect();

time=0;

pose_seq=[];
wr_wl_seq=[];





avg_time=0;

%%

K_f=length(xr);
% pause(5)
i_time=1;
t=zeros(1,K_f);




while i_time<=K_f
    tic
    
    khep.x_r=xr(i_time); khep.y_r=yr(i_time); khep.theta_r=thetar(i_time);
    khep.xp_r=xpr(i_time); khep.yp_r=ypr(i_time);
    khep.xpp_r=xppr(i_time); khep.ypp_r=yppr(i_time);
    
    khep=khep.ST_MPC_disturbance_trajectory_tracking();
    
    avg_time=avg_time+toc;
    
    pose_seq=[pose_seq khep.pose()];
    wr_wl_seq=[wr_wl_seq khep.wheels_vel];
    toc
    while toc<Ts
    end
    time=time+toc;
    t(i_time)=time;
    i_time=i_time+1;
end

index_seq=khep.control_parameters.index_seq;


khep.set_motors_speed(0,0);
khep=khep.disconnect;

err_seq=pose_seq(1:2,:)-[xr;yr];

norm_err=zeros(1,size(err_seq,2));
norm_eng=zeros(1,size(err_seq,2));

for j=1:length(norm_err)
    e_j=err_seq(:,j);
    norm_err(j)=sqrt(e_j'*e_j);
    
    en_j=wr_wl_seq(:,j);
    norm_eng(j)=en_j'*en_j;
    
end


MSE=sum(norm_err.^2)/K_f

F_IAE = griddedInterpolant(t,abs(norm_err));
fun_IAE = @(t) F_IAE(t);
IAE = integral(fun_IAE, t(1), t(end))


F_ISE = griddedInterpolant(t,norm_err.^2);
fun_ISE = @(t) F_ISE(t);
ISE = integral(fun_ISE, t(1), t(end))

F_ITAE = griddedInterpolant(t,t.*abs(norm_err));
fun_ITAE = @(t) F_ITAE(t);
ITAE = integral(fun_ITAE, t(1), t(end))

F_ITSE = griddedInterpolant(t,t.*norm_err.^2);
fun_ITSE = @(t) F_ITSE(t);
ITSE = integral(fun_ITSE, t(1), t(end))

% avg_enrg=sum(norm_eng)/(K_f)

% for i=1:length(pose_seq)
%     
%     Xr=[xr(i);yr(i)];
%     xtilde_c=pose_seq(1:2,i)-Xr;
%     abs(xtilde_c)-0.05*abs(Xr);
%     if abs(xtilde_c)>0.05*abs(Xr)
%         convergence_time=inf;
%     elseif abs(xtilde_c)<=0.05*abs(Xr)
%         convergence_time=(i-1)*Ts;
%     end
%     
% end



disp('ST-MPC-eight')
save('sim_ST_MPC_eight','pose_seq','wr_wl_seq','t','xr','yr')


figure
grid
hold on
p1=plot(pose_seq(1,:),pose_seq(2,:));
p8=plot(xr,yr,'k--');
p9=plot(pose_seq(1,1),pose_seq(2,1),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',15);

p1.LineWidth=1.5;
p8.LineWidth=1.5;
p9.LineWidth=1.5;

axis([-0.8 1 -0.8 0.8])
% axis([-1 1.2 -0.9 0.9])
l=legend([p1,p8,p9],'Proposed','Reference','Initial point');
l.FontSize=27;
xlabel('x[m]')
ylabel('y[m]')
title('Eight-Shaped Trajectory')

figure
title('Wheel Angular Velocities')
subplot(2,1,1)
grid 
hold on 
p1=plot(t,wr_wl_seq(1,:)*max_speed_rad/1200);

p8=plot(t,wrlmax*ones(1,length(t)),'r--');
plot(t,-wrlmax*ones(1,length(t)),'r--')

p1.LineWidth=1.5;

axis([0 45.3 -10 10])
xlabel('t[sec]')
ylabel('\omega_R[RAD/sec]')


l=legend([p1,p8],'Proposed','\omega_{R,MAX}');
l.FontSize=27;


subplot(2,1,2)
grid 
hold on 
p1=plot(t,wr_wl_seq(2,:)*max_speed_rad/1200);


p8=plot(t,wrlmax*ones(1,length(t)),'r--');
plot(t,-wrlmax*ones(1,length(t)),'r--')
axis([0 45.3 -10 10])
xlabel('t[sec]')
ylabel('\omega_L[RAD/sec]')


p1.LineWidth=1.5;


p8.LineWidth=1.5;

l=legend([p1,p8],'Proposed','\omega_{L,MAX}');
l.FontSize=27;

figure
title('Orientation \theta(k)')
grid 
hold on 
p1=plot(t,pose_seq(3,:));
p8=plot(t,thetar,'k--');
axis([0 45 0 6])

xlabel('t[sec]')
ylabel('\theta[RAD]')


p8.LineWidth=1.5;


l=legend([p1,p8],'Proposed','\theta_{ref}');
l.FontSize=27;




err_seq=pose_seq(1:2,:)-[xr;yr];

norm_err=zeros(1,size(err_seq,2));


for j=1:length(norm_err)
    e_j=err_seq(:,j);
    norm_err(j)=sqrt(e_j'*e_j);
    
end

figure
title('Tracking Error')
grid 
hold on 
p1=plot(t,norm_err);

p1.LineWidth=1.5;

l=legend([p1,p8],'Proposed','[21]');
l.FontSize=27;

xlabel('t[sec]')
ylabel('e[m]')

axis([0 45 0 0.65])





function [Pn] = computeOneStepControllableSets(discreteModel,Nsets,T0,U)
%COMPUTEONESTEPCONTROLLABLESETS Summary of this function goes here
%   Detailed explanation goes here
N=Nsets;
model=discreteModel;

% compute reachable sets starting from T0
Pn = T0; % save intermediate set during computation
hold on
plot(T0,'Alpha',0.8,'Color','blue');

for i =1:N
    fprintf('building reachable set %i of %i\n',i,N)
    % backward reachable set
    R = model.reachableSet('X', T0, 'U', U, 'direction', 'backward');
    %       R = model.reachableSet('U', U, 'direction', 'backward');
    % intersect with the state constraints
    %    R = R.intersect(X);
    hold on
    plot(R,'Alpha',0,'Color','green','wired','true','LineWidth',0.5)
    pause(0.5)
    Pn = [Pn,R];
    if R==T0
        break
    else
        T0 = R;
    end
end

end



function Qnext = one_step_ellipsoidal_reachable_set(A,Bu,Bd,Qcurr,Qu,Qd)



if A'*Qcurr*A+Bu'*Qu*Bu-Bd'*Qd*Bd<=zeros(2,2)
    Qnext=zeros(2,2);
    error('The one-step reachable set is void ')
    
end


Qnext= (sqrt(Qcurr)+sqrt(-Bu'*Qu*-Bu)-sqrt(-Bd'*Qd*-Bd))^2;



end

