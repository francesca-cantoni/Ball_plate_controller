clc; clear; close all;
load('Linsys.mat');

%% System definition

A=linsys1.A;    % (8,8)
B=linsys1.B;    % (8,2)
C=linsys1.C;    % eye(8)
D=linsys1.D;    % zeros(8,2)

[n,m] = size(B);

fprintf("---ANALYSIS WITH ALL SENSORS---\n\n");
sys_prop(A,B,C, 'Properties of OPEN LOOP SYSTEM:');

sys=ss(A,B,C,D);
T=tf(sys); %Transfer function (per regola permanenza del segno, ho degli zeri a parte reale positiva)

%% State Space Observer

Q1 = eye(n);
gamma = 1;
Q = gamma * Q1;
R = eye(n);

[Ht,S,egn_val_H] = lqr(A',C',Q,R);
H = Ht';            %(8,2)

%% Feedback Control Low 
Q = eye(n);
R = eye(m);
[K,S,egn_val_K] = lqr(A,B,Q,R);

%% Nx and Nu prefilters
% since G is not square I cannot invert it therefore I cannot compute Nx
% and Nu

% G = C*(inv(eye(8)-A))*B;
% G_inv = inv(G);

%% Reduced State Space Observer
% Hp: only x1(t) and x2(t) are measured
% number of measured states = 2
% number of unmeasured states = 6

C_red=[1 0 0 0 0 0 0 0;     
       0 1 0 0 0 0 0 0; 
       zeros(6,8)];
   
A11=A(1:2,1:2);     %(2x2)  
A12=A(1:2,3:8);     %(2x6)  
A21=A(3:8,1:2);     %(6x2)
A22=A(3:8,3:8);     %(6x6) 

B1=B(1:2,:);        %(2x2)
B2=B(3:8,:);        %(6x2)

Q= eye(6);
R = eye(2);
[Hbt,S,egn_val_Hb] = lqr(A22',A12',Q,R);
Hb = Hbt';          %(6,2)

K1=A22-Hb*A12;
K2=(A22-Hb*A12)*Hb;
K3=Hb*B1;
K4=Hb*A11;

%% Dynamic Compensator
Ak=[A zeros(8,8); C zeros(8,8)];    %(16,16)
Bk=[B; zeros(8,2)];                 %(16,2)
Ck=[C zeros(8,8)];                  %(16,16)

sys_prop(Ak,Bk,Ck, 'Properties of AUGMENTED SYSTEM FOR DYNAMIC COMPENSATOR: (check controllability)'); %check for controllability

% The system with the augmented state is not controllable therefore none
% value of K can stabilize it (I cannot place the poles)

% Q = eye(16);
% R = eye(2);
% [Kbig,S,egn_val_Kbig] = lqr(Ak,Bk,Q,R);
% Kk=Kbig(:,1:8);
% Ki=Kbig(:,8+1:16);

%% Disturbance Observer
%d(t) = d0 1(t)

Ad=0;                   %(1x1)
D=[ones(8,1)];          %(8x1)
Cd=1;                   %(1x1)

sys_prop(Ad,0,Cd, 'Properties of disturbance system Sd:'); %check for controllability

Aa=[A D*Cd; zeros(1,8) Ad];     %(9x9)
Ba=[B; zeros(1,2)];             %(9x2)
Ca=[C zeros(8,1)];              %(8x9)

sys_prop(Aa,Ba,Ca, 'Properties of AUGMENTED SYSTEM FOR DISTURBANCE OBSERVER: (check observability)'); %check for observability

Q= eye(9);      %same dimension as the augmented state
R = eye(8);     %same dimension as the augmented output
[Hat,S,egn_val_Ha] = lqr(Aa',Ca',Q,R);
Ha = Hat';              %(9,8)

G=pinv(B)*D;            %(1,1)

%% Considering the case with only sensors for x1 and x2 (Sr)
%I can reconstruct other states since the system remains fully
%observable
fprintf("---ANALYSIS WITH ONLY TWO SENSORS---\n\n");

Cr=[1 0 0 0 0 0 0 0;    %(2,8)
    0 1 0 0 0 0 0 0];

sys_prop(A,B,Cr, 'Properties of REDUCED SYSTEM Sr: (check everything)');

%% State space observer on Sr
Q= eye(8);
R = eye(2);
[Hrt,S,egn_val_Hr] = lqr(A',Cr',Q,R); 
Hr = Hrt';              %(8,2)

%% Reduced order observer on Sr 
%Same computation as Hb but different scheme
Hbr = Hb;               %(6,2)

%% Dynamic Compensator
Ak_r=[A zeros(8,8); Cr zeros(2,8)];     %(10,16)
Bk_r=[B; zeros(2,2)];                   %(10,2)
Ck_r=[Cr zeros(2,8)];                   %(16,16)

% Since Cr is not square matrix, leads to a non-square Ak_r --> I cannot
% implement a dynamic compensator
% sys_prop(Ak_r,Bk_r,Ck_r, 'Properties of AUGMENTED SYSTEM FOR DYNAMIC COMPENSATOR: (check controllability)'); %check for controllability

%% Disturbance Observer
%d(t) = d0 1(t)

Ad_r=0;                   %(1x1)
D_r=[1;1;zeros(6,1)];     %(8x1)
Cd_r=1;                   %(1x1)

sys_prop(Ad_r,0,Cd_r, 'Properties of disturbance system Sd:'); %check for controllability

Aa_r=[A D_r*Cd_r; zeros(1,8) Ad_r];     %(9x9)
Ba_r=[B; zeros(1,2)];                   %(9x2)
Ca_r=[C zeros(8,1)];                    %(8x9)

sys_prop(Aa_r,Ba_r,Ca_r, 'Properties of AUGMENTED SYSTEM FOR DISTURBANCE OBSERVER: (check observability)'); %check for observability

Q_r = eye(9);       %same dimension as the augmented state
R_r = eye(8);       %same dimension as the augmented output
[Hat_r,S_r,egn_val_Ha_r] = lqr(Aa_r',Ca_r',Q_r,R_r);
Ha_r = Hat_r';              %(9,8)

G_r = pinv(B)*D_r;          %(1,1)

%% Properties of Open-loop system
% This function computes and checks all the main properties of a system, 
% namely the stability, the controllability and the observability.

function [s,c,o, st] = sys_prop(A,B,C, ST)
    disp (ST);
    sys = ss(A,B,C,[]);
    
    %% Open loop stability check
    s = isstable(sys); %determine if the system is stable 

    if s
        disp ('The system is stable.');
    else
        disp ('The system is unstable.');
    end

    [stab_states, unstable_states] = stabsep(sys); %subdivide the system in stable and unstable parts

    %% Controllability check
    uncont_states = length(A) - rank(ctrb(A,B)); 
    c = (uncont_states == 0); %if uncount_states=0, Kalman controllability matrix is fully rank, therefore the sys is fully controllable
    
    if c
        disp ('The system is fully controllable.');
    else
        disp ('The system is not controllable.');

        msg = sprintf (' It has %d uncontrollable states.', uncont_states);
        fprintf(msg);

        if rank(ctrb(unstable_states.A,unstable_states.B)) == length(unstable_states.A) %if the unstable states are controllable
            c = 1;
            disp (' However, the system is at least stabilizable.');
        else
            disp (' The system is not even stabilizable.');
        end
    end

    %% Obserability check
    unobs_states = length(A) - rank(obsv(A,C));
    o = (unobs_states == 0); %if unobs_states=0, Kalman observability matrix is fully rank, therefore the sys is fully observable

    if o
        disp ('The system is fully observable.');
    else
        disp ('The system is not observable.');
        msg = sprintf (' It has %d unobservable states.', unobs_states);
        fprintf(msg);

        if rank(obsv(unstable_states.A,unstable_states.C)) == length(unstable_states.A) %if the unstable states are observable
            o = 1;
            disp (' However, the system is at least detectable.');
        else
            disp (' The system is not even detectable.');
        end
    end
    
    fprintf(1, '\n');  
end