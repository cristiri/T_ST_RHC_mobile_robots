clear;
close all;


%%  System, Constraints and Disrturbances
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%if these are changed, then all the offline
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%parts must be re-computed

%% Robot modeling
Ts=0.15; %Sampling time
r=0.0205; %wheels' radius
d=0.053;  %wheels' axis length
b=0.1;    %b distance for FL


A = eye(2);
Bu = eye(2)*Ts;
Bd = eye(2)*Ts;
C = eye(2);
D = [0; 0];


wrlmax=10;%Differential-drive wheels' angular velocities limits



%% TRAJECTORY PARAMETERS
%Initial conditions
%scaling factor of the trajectory
eta=0.6;
alpha=3.5;
k=0:Ts:2*pi*alpha*2-Ts;
xr=eta*sin(k/alpha);
yr=eta*sin(k/(2*alpha));

%Velocity trajectory
xpr=eta*cos(k/alpha)*(1/alpha);
ypr=eta*cos(k/(2*alpha))*(1/(2*alpha));

%Acceleration trajectory
xppr=-eta*sin(k/alpha)*(1/alpha)*(1/alpha);
yppr=-eta*sin(k/(2*alpha))*(1/(2*alpha))*(1/(2*alpha));

%Driving velocity reference
vr=sqrt(xpr.^2+ypr.^2);
wr=(yppr.*xpr-xppr.*ypr)./(xpr.^2+ypr.^2);


% Orientation reference
thetar=atan2(ypr,xpr);

%Adjusting Orientation ( a part of tetha(k) is out of phase
thetar_diff=diff(thetar);
for i=1:length(thetar_diff)
    if thetar_diff(i)<-6
        i1=i+1;
    elseif thetar_diff(i)>6
        i2=i;
    end
end
thetar(i1:i2)=thetar(i1:i2)+2*pi;

x0=0.6; y0=0; theta0=pi;



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




figure
plot(disturbance(1,:),disturbance(2,:))
hold on
ell_d=ellipsoid(Qd);
plot(ell_d)
hold off




T=[[r/2 r/2];[r/d -r/d]];
T_inv=inv(T);
Hd=[-1/wrlmax 0 ; 0 -1/wrlmax ; 1/wrlmax 0 ; 0 1/wrlmax];  %shaping matrix of box-like constraint set for differential-drive


U_dd= Polyhedron('lb',-[wrlmax;wrlmax],'ub',[wrlmax;wrlmax]); %Differential-drive input constraint set
U_uni=T*U_dd; %Unicycle input constraint set


ru=(2*wrlmax*r*b)/(sqrt(4*b^2+d^2));
Qu=ru^2*eye(2);% disturbance Shaping matrix



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
    
    Qi=one_step_ellipsoidal_reachable_set(A,Bu,Bd,Qcurr,Qu,Qd);
    
    if plot_sets
        ell_i=ellipsoid(Qi);
        plot(ell_i,'b')
    end
    
    
    pause(0.00001)
    Q_k=[Q_k Qi];
    Qcurr=Qi;
    
end




%% Variables to store and plot the results
q0=[x0;y0;theta0];
z0=[x0+b*cos(theta0);y0+b*sin(theta0)];

tt=[];
qseq=q0;
wrwlseq=[];

k=1;
N_steps_simulation=length(xr);

%control effort matrix
R=0*eye(2);
R_d=0*eye(2);
u_pre=[0;0];



index_seq=[];

uk_seq=[];


%% Plots
figure
grid
hold on
axis([-1 1 -1 1])
plot(q0(1,1),q0(2,1),'b-p','MarkerIndices',[1 1],'MarkerFaceColor','yellow','MarkerSize',15)

plot(xr,yr,'r--')


avg_comp_time1=0;
optim1_steps=0;
optim2_steps=0;

avg_comp_time2=0;


for i=0:Ts:10000000
    %
    tt=[tt i];
    
    %extracting current state of differential-drive
    x=qseq(1,end);
    y=qseq(2,end);
    theta=qseq(3,end);
    
    z_curr=[x+b*cos(theta)-(xr(k)+b*cos(thetar(k)));
        y+b*sin(theta)-(yr(k)+b*sin(thetar(k)))];
    
    
    
    T_FL = [cos(theta) sin(theta); -sin(theta)/b cos(theta)/b];
    T_FL_inv=inv(T_FL);
    
    %Compute current input constraint set
    %     U_curr=T_FL_inv*U_uni;
    
    H_curr=Hd*T_inv*T_FL;
    g=ones(4,1);
    U_curr=Polyhedron(H_curr,g);
    
    isintern=0;
    for j=1:2:length(Q_k)
        Q_curr=Q_k(1:2,j:j+1);
        if  z_curr'*inv(Q_curr)*z_curr<=1
            
            index=(j-1)/2+1;
            index_seq=[index_seq index];
            isintern=1;
            if index>1
                Qpre=Q_k(1:2,j-2:j-1);
            end
            break
        end
    end
    
    
    
    if isintern==0
        error('The current point is outside the domain of attraction of the algorithm')
    end
    
    
    
    T_FL_r=[cos(thetar(k)) sin(thetar(k)); -sin(thetar(k))/b cos(thetar(k))/b];
    ur=inv(T_FL_r)*[vr(k);wr(k)];
   

    
    if index>1
        
        %     ukhat = online_phase_yal(z_curr,Qpre,A,Bu,H_curr,g,R,R_d,ur,u_pre);
        %     ukhat = online_phase_yal_conservative(z_curr,Qpre,A,Bu,H_curr,g,R,R_d,ur,u_pre,Qu);
        %     ukhat = online_phase_fmincon_conservative(z_curr,Qpre,A,Bu,H_curr,g,R,R_d,ur,u_pre,Qu);
        
        tic
%         ukhat2 = online_phase_fmincon(z_curr,Qpre,A,Bu,H_curr,g,R,R_d,ur,u_pre);
        ukhat = online_phase_CDC(z_curr,Qpre,A,Bu,H_curr,g,R_d,ur,u_pre);
%         ukhat3 = online_phase_gurobi(z_curr,Qpre,A,Bu,H_curr,g,R_d,ur,u_pre);
     
        
        
        curr_comp_tim1=toc;
        
        avg_comp_time1=avg_comp_time1+curr_comp_tim1;
        
        optim1_steps=optim1_steps+1;
        
    else
        
        tic
        ukhat=-inv(Bd)*z_curr;
        
        ur=online_terminal_optim_QP(ur,ukhat,H_curr,g);
%         ur=online_terminal_optim(ur,ukhat,H_curr,g);
        
        curr_comp_tim2=toc;
        
        avg_comp_time2=avg_comp_time2+curr_comp_tim2;
        
        optim2_steps=optim2_steps+1;
    end
    
    
    uk=ukhat+ur;
    u_pre=ukhat;
    
    
    
    uk_seq=[uk_seq uk];
    
    
    
    vw=T_FL*uk;
    
    wrwl=T_inv*vw;
    
    wrwlseq=[wrwlseq wrwl];
    
    %Applico la legge di controllo al sitema non lineare
    v1=vw(1); w1=vw(2);
    t=0:0.00001:Ts;
    [t,q]= ode45(@(t,q,v,w)DiffDrive(t,q,v1,w1),t,qseq(:,end));
    
    %aggiorno la sequenza di stato e ingresso
    qseq=[qseq q(end,:)'];
    
    %plotto la traiettoria del robot
    plot(qseq(1,end),qseq(2,end),'b--x')
    pause(0.00001)
    
    if k>=N_steps_simulation
        break
    else
        k=k+1;
    end
end


avg_comp_time1=avg_comp_time1/optim1_steps;
avg_comp_time2=avg_comp_time2/optim2_steps;

avg_comp_time1*1000
avg_comp_time2*1000


figure
subplot(3,1,1);
hold on;
p1=plot(tt,wrwlseq(1,:));
p2=plot(tt,ones(1,length(tt))*wrlmax);
p2.LineStyle='--';
p2.LineWidth=2;
p2.Color='red';
p2=plot(tt,ones(1,length(tt))*-wrlmax);
p1.LineWidth=2;
p2.LineStyle='--';
p2.LineWidth=2;
p2.Color='red';
l1=legend(p2,'\omega_{r,max}','Interpreter','latex');
l1.set('FontSize',10)
grid;
xlbl1=xlabel('Time[sec]','Interpreter','latex');
ylbl1=ylabel('$\omega_r(t)[RAD/sec]$','Interpreter','latex');
ylbl1.FontSize=13;
xlbl1.FontSize=13;


subplot(3,1,2);
hold on;
p3=plot(tt,wrwlseq(2,:));
p3.LineWidth=2;

p4=plot(tt,ones(1,length(tt))*wrlmax,'r');
p4.LineStyle='--';
p4.LineWidth=2;

p4=plot(tt,ones(1,length(tt))*-wrlmax,'r');

p4.LineStyle='--';
p4.LineWidth=2;
p4.Color='red';
l2=legend(p4,'\omega_{l,max}','Interpreter','latex');
l2.FontSize=10;
grid;
xlbl2=xlabel('Time[sec]','Interpreter','latex');
ylbl2=ylabel('$\omega_l(t)[RAD/sec]$','Interpreter','latex');
ylbl2.FontSize=13;
xlbl2.FontSize=13;



subplot(3,1,3)
pl=plot(tt,qseq(3,1:end-1));
pl.LineWidth=2;
grid
% axis([0 tt(length(tt)) 0 6])
xlbl2=xlabel('Time [sec]','Interpreter','latex');
ylbl2=ylabel('$\theta(t)[RAD]$','Interpreter','latex');


figure
p1=plot(tt,index_seq);
grid
p1.LineWidth=2;



eps=0;
for k=1:N_steps_simulation
    x_tilde=[xr(k);yr(k)]-qseq(1:2,k);
    eps=eps+x_tilde'*x_tilde;
    
end

eps=eps/N_steps_simulation