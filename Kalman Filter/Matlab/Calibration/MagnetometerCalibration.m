% This File it used to Calibrate magnetometer, to calibrate magnetometer
% you have to enter a file with the next form
% MagnetometerX     MagnetometerY   MagnetometerZ

fileID = fopen('MagData.txt','r');
formatSpec = '%f';
sizeA = [3 Inf];
MagnetometerData = fscanf(fileID,formatSpec,sizeA);
Max = size(MagnetometerData);
for i= 1: 1: Max(2)
    if i == 1
        Xout_Mag_max = MagnetometerData(1,i);
        Yout_Mag_max = MagnetometerData(2,i);
        Zout_Mag_max = MagnetometerData(3,i);
        
        Xout_Mag_min = MagnetometerData(1,i);
        Yout_Mag_min = MagnetometerData(2,i);
        Zout_Mag_min = MagnetometerData(3,i);
    end
    
    if MagnetometerData(1,i) > Xout_Mag_max
        Xout_Mag_max = MagnetometerData(1,i);
    end
    
    if MagnetometerData(2,i) > Yout_Mag_max
        Yout_Mag_max = MagnetometerData(2,i);
    end
    
    if MagnetometerData(3,i) > Zout_Mag_max
        Zout_Mag_max = MagnetometerData(3,i);
    end
    
    if MagnetometerData(1,i) < Xout_Mag_min
        Xout_Mag_min = MagnetometerData(1,i);
    end
    
    if MagnetometerData(2,i) < Yout_Mag_min
        Yout_Mag_min = MagnetometerData(2,i);
    end
    
    if MagnetometerData(3,i) < Zout_Mag_min
        Zout_Mag_min = MagnetometerData(3,i);
    end
end

Xout_Mag_Avg = (Xout_Mag_max+Xout_Mag_min)/2;
Yout_Mag_Avg = (Yout_Mag_max+Yout_Mag_min)/2;
Zout_Mag_Avg = (Zout_Mag_max+Zout_Mag_min)/2;

Offset_X = Xout_Mag_Avg
Offset_y = Yout_Mag_Avg
Offset_z = Zout_Mag_Avg

Xout_Mag_Avg = (Xout_Mag_max - Xout_Mag_min)/2;
Yout_Mag_Avg = (Yout_Mag_max - Yout_Mag_min)/2;
Zout_Mag_Avg = (Zout_Mag_max - Zout_Mag_min)/2;

Avg_delta = (Yout_Mag_Avg + Yout_Mag_Avg + Zout_Mag_Avg)/3;

ScaleX = Avg_delta/Xout_Mag_Avg
ScaleY = Avg_delta/Yout_Mag_Avg
ScaleZ = Avg_delta/Zout_Mag_Avg
