clear all, close all, clc
%% Open File
fileID = fopen('5hzprueba1.txt','r');
formatSpec = '%f';
sizeA = [5 Inf];
Data = fscanf(fileID,formatSpec,sizeA);
%% Start of script
Yaw = Data(5,:);
%% Matrix
Max = size(Data)
Pk = eye(2);
I = eye (2);
delta_t = .2;
deltat = .2;
t = 0: deltat: (Max(2)*deltat)-deltat;
A = [1 deltat; 0 1];
At = A';
Ac = (A-I)/deltat;
B = [(deltat^2)/2; deltat];
Bc = B/deltat;
C = [1 0];
Ct = C';
D = zeros(size(C,1), size(Bc,2));
wgs84 = wgs84Ellipsoid;
[xNorth,yEast,zDown] = geodetic2ned(Data(1,:),Data(2,:), 1584.849,Data(1,1),Data(2,1),1584.849,wgs84)
x = [xNorth(1);0];
Xe = [xNorth(1);0];
xint=[0;0];
Xint=[0;0];
%% Noise
Vd = 1*eye(2);
Vn = .1;
Siz = size(t);
save = zeros(2,(Siz(2)));
save2 = zeros(2,(Siz(2)));
recta = zeros((Siz(2)));
counter = 1;
save(:,counter) = xNorth(1);
for i= 1: deltat: (Max(2)*deltat)
   
    counter = counter + 1;
    %Predict    
        Xe = A*x + (B*Data(3,counter))*cos(degtorad(Yaw(counter)'));
        Xint = A*xint + (B*Data(3,counter))*cos(degtorad(Yaw(counter)'));

    xint = Xint;
    % B es multiplicado por u
    Pe = A*Pk*At + [.01 0; 0 .01];
    %Update
    Kk = (Pe*Ct)/((C*Pe*Ct) + 1);
    x = Xe + Kk*((xNorth(counter)) - C*Xe);
    Pk = (I-Kk*C)*Pe;
    save(:,counter) = x();
    save2(:,counter) = Xint();
    if (counter == 1)
        recta(counter) = -55;
    else
        recta(counter) = recta(counter-1) + 0.31055;
    end
end 
%plot(t,Yaw);
hold on;
plot(t,save(1,:),'-',t,xNorth,'--');