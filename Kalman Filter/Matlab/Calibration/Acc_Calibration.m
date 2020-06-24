% This File it used to Calibrate Accelerometer, to calibrate magnetometer
% you have to enter a file with the next form
% AccX     AccY   AccZ

fileID = fopen('AccData.txt','r');
formatSpec = '%f';
sizeA = [3 Inf];
AccData = fscanf(fileID,formatSpec,sizeA);
Max = size(AccData);
AccX = 0;
AccY = 0;
AccZ = 0;
for i= 1: 1: Max(2)
    AccX = AccX + AccData(1,i);
    AccY = AccY + AccData(2,i);
    AccZ = AccZ + AccData(3,i);
end
AccX = AccX / Max(2)
AccY = AccY / Max(2)
AccZ = AccZ / Max(2)