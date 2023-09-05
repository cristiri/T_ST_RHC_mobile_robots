classdef ST_MPC_disturbance_control_parameters < control_parameters
    
    
    properties (SetAccess=protected)
        Ts,b,R,Rd,Ad,Bd,Q_k
    end
    
    properties
        u_pre,index_seq
    end
    
    methods
        function obj=ST_MPC_disturbance_control_parameters(Ts,b,R,Rd,Q_k,index_seq)
            if nargin==0
                obj.Ts=0.2;
                obj.b=0.1;
                obj.R=0.01*eye(2);
                obj.R=zeros(2,2);
            else
                
                if Ts<=0
                    error('The sampling time must be a positive real number')
                end
                if b==0
                    error('The traslation factor b must not be zero')
                end
                if size(R,1)~=2 || size(R,2)~=2
                    error('The weight magtrix R must be 2x2')
                end
                obj.Ts=Ts;
                obj.b=b;
                obj.R=R;
                obj.Rd=Rd;
                obj.Q_k=Q_k;
                obj.index_seq=index_seq;
                
            end
            obj.Ad=eye(2);
            obj.Bd=obj.Ts*eye(2);
            obj.u_pre=[0;0];
            
        end
    end
end

