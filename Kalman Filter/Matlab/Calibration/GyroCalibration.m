% This File it used to Calibrate Gyroscope, to calibrate magnetometer
% you have to enter a file with the next form
% GyroX     GyroY   GyroZ

fileID = fopen('GyroData.txt','r');
formatSpec = '%f';
sizeA = [3 Inf];
GyroData = fscanf(fileID,formatSpec,sizeA);
Max = size(GyroData);
GyroX = 0;
GyroY = 0;
GyroZ = 0;
for i= 1: 1: Max(2)
    GyroX = GyroX + GyroData(1,i);
    GyroY = GyroY + GyroData(2,i);
    GyroZ = GyroZ + GyroData(3,i);
end
GyroX = GyroX / (Max(2)+1)
GyroY = GyroY / (Max(2)+1)
GyroZ = GyroZ / (Max(2)+1)

