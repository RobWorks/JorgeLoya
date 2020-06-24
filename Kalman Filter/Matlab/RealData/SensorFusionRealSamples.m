clear all, close all, clc
%% Open File
fileID = fopen('AllMeasure1.txt','r');
formatSpec = '%f';
sizeA = [12 Inf];
Data = fscanf(fileID,formatSpec,sizeA);
%% Matrix
Max = size(Data)
x = [5.3189;0];
Xe = [5.3189;0];
Pk = eye(2);
I = eye (2);
delta_t = .3;
t = 0: .5: (Max(2)-1)/2;
A = [1 .5; 0 1];
At = A';
Ac = (A-I)/.5;
B = [(.5^2)/2; .5];
Bc = B/.5;
C = [1 0];
Ct = C';
D = zeros(size(C,1), size(Bc,2));
wgs84 = wgs84Ellipsoid;
[xNorth,yEast,zDown] = geodetic2ned(Data(10,:),Data(11,:),Data(12,:),20.607139,-103.413375,1607.800048,wgs84)

%% Noise
Vd = 1*eye(2);
Vn = .1;
Siz = size(t);
save = zeros(2,(Siz(2)));
counter = 1;

for i= 0: 1: (Max(2)-2)
    counter = counter + 1;
    %Predict    
    Xe = A*x + (B*Data(2,counter));
    % B es multiplicado por u
    Pe = A*Pk*At + 1.8409e-04*eye(2);
    %Update
    Kk = (Pe*Ct)/((C*Pe*Ct) + .0035389);
    x = Xe + Kk*((xNorth(counter)) - C*Xe);
    Pk = (I-Kk*C)*Pe;
    save(:,counter) = x();
    
end 

plot(t,save(1,:),'-',t,xNorth,'--');
xNorth(Max)
save(1,Max)
