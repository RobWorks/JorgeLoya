clear all, close all, clc
%% Matrix
x=[0;0]
Pk = eye(2);
I = eye (2);
delta_t = .01;
A = [1 delta_t; 0 1];
At = A';
Ac = (A-I)/delta_t;
B = [(delta_t^2)/2; delta_t];
Bc = B/delta_t;
C = [1 0];
Ct = C';
D = zeros(size(C,1), size(Bc,2));

%% Noise
Vd = .1*eye(2);
Vn = 1;

BF = [Bc Vd 0*Bc];

t = delta_t: delta_t: 50; % Creamos una variable t para simular 50 seg
uDIST = randn(2, size(t,2)); % Generamos ruido para los estados
uNOISE = randn(size(t));

u=0*t;  % Creamos u que es la entrada de nuestro modelo.
u(100:120) = 5; % Añadimos un movimiento en nuestra entrada
u(1500:1520) = -5; % Añadimos el movimiento opuesto
u(2500:2520) = -15; % Añadimos un movimiento en nuestra entrada
u(3500:3520) = 15;
uAUG = [u; Vd*Vd*uDIST; uNOISE]; 

%% Entradas y salidas del sistema
sysC = ss(Ac,BF,C,[0 0 0 Vn]);

[y,t] = lsim(sysC, uAUG, t);

sysFullOutput = ss(Ac,BF,eye(2),zeros(2,size(BF,2)));
[xtrue,t] = lsim(sysFullOutput, uAUG, t);


%% My Kalman Filter
counter = 0;
save = zeros(2,5000);
Kk = [0.0014;
    0.0001];
for i= 0.01: delta_t: 50
    counter = counter + 1;
    %Predict    
    Xe = A*x + B*u(counter);
    % B es multiplicado por u
    Pe = A*Pk*At + .0001*eye(2);
    %Update
    Kk = (Pe*Ct)/((C*Pe*Ct) + 1000);
    x = Xe + Kk*((y(counter)) - C*Xe);
    Pk = (I-Kk*C)*Pe;
    save(:,counter) = x();
    
end 
Kf = (lqr(Ac',C',Vd,Vn))';

sysKF = ss(Ac-Kf*C, [Bc Kf], eye(2), 0*[Bc Kf]);

[xKalman,t] = lsim(sysKF,[u; y'],t);
plot(t,xtrue(:,1));
hold on;
plot(t,y);
hold on;
plot(t,save(1,:),'-',t,xKalman(:,1),'--','LineWidth',1)
