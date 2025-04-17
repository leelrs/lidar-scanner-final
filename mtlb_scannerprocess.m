clc;
clear;
clf;

%init serial comms:
s = serialport("COM5", 115200, "Timeout", 200); %set port, baud, timeout(arbitrary)
flush(s); %clr serial buffer
disp("opening: " + s.Port);
s.write("s", "string"); %send start cmd to sensor

%define tof measurement params-------------------------------------------
layerNum = 3; %how many vertical layers are we scanning
measurementNum = 32; %#of points per layer, 32 corresponds to 11.5degree steps
spinStat = 1; %ensure motor is spinning lol
stopMeasurement = false; %flag for stopping tof measurements, set to false so it starts

endLayer = 0;%minds the last scanned layer before termination (if it occurs)

%arrays to store coordinates from tof
xCoords = zeros(1, measurementNum * layerNum);
yCoords = zeros(1, measurementNum * layerNum);
zCoords = zeros(1, measurementNum * layerNum);

for j = 1:layerNum
    
    firstValid = false;
    while ~firstValid %waits for the first actual piece of tof data to come thru
        data = readline(s); %read incoming data

        if ~isempty(data)
            tempMatrix_data = translate(data); %convert raw readings to numerical
            
            if ~isnan(tempMatrix_data(1))%checks if the first value is ok
                disp("First valid measurement received for layer " + j + ": " + tempMatrix_data(1));
                firstValid = true; %swaps flag to enable further readings to be stored
    
                %calculate the 3d coordinates from the distance value and
                %the angle
                tempMatrix = calculate(tempMatrix_data(1), j, tempMatrix_data(2));
                xCoords((measurementNum*(j-1))+1) = tempMatrix(1);
                yCoords((measurementNum*(j-1))+1) = tempMatrix(2);
                zCoords((measurementNum*(j-1))+1) = tempMatrix(3);

            else
                disp("waiting: need first valid measurement " + j + ".");  
            end
        else
            disp("waiting: need first valid measurement " + j + ".");
        end
    end
%-------------main loop (measurement for current layer)----------------------------
    for i = 2:measurementNum
        data = readline(s); %read serial data
        input_data = translate(data); %convert to numerical
        distance_data = input_data(1); %pull out distance measurement
        currAngle = input_data(2); %pull out angle measurement
        spinStat = input_data(3); %pull out spin status at the time of measurement

        disp(currAngle);

        if isnan(distance_data) %safety for if the data incoming is somehow 0
            disp("warning, data invalid. not breaking out of loop");
            continue;
        end

        if isnan(spinStat)||~spinStat %if the spinstat is off, the motor is off.
            stopMeasurement = true; %so the tof will also stop measuring
        end

       %calculate 3d coordinates from dist and angle for the rest of the
       %incoming data
        tempMatrix = calculate(distance_data, j, currAngle);

        index = (j-1)*measurementNum+i;


        xCoords(index) = tempMatrix(1);
        yCoords(index) = tempMatrix(2);
        zCoords(index) = tempMatrix(3);

        endLayer = j; %update termination layer(if you were to hit stop rn, itd only print the most recent complete layer j)
    end

    currAngle = 0;

    if stopMeasurement
        break;
    end

end

layerNum = endLayer; %adjust layer count to reflect real measured layers. useful if terminated.

disp("loop exited");

%trim coord arrays to only be as big as the amount of data we've collected
%(in the case of an early stop, or generally.)
xCoords = xCoords(1:(measurementNum*layerNum));
yCoords = yCoords(1:(measurementNum*layerNum));
zCoords = zCoords(1:(measurementNum*layerNum));

%goodbye serial port data. deleted
delete(s);
clear s;

%3d plot of scanned coords
plot3(xCoords, zCoords, yCoords, 'b');

%draw lines connecting corresponding points between layers.
for j = 1:(layerNum - 1)
    for i = 1:(measurementNum-1)
        index_current = (j - 1) * measurementNum + i;
        index_next = j * measurementNum + i;

        line([xCoords(index_current), xCoords(index_next)], ...
             [zCoords(index_current), zCoords(index_next)], ...
             [yCoords(index_current), yCoords(index_next)], ...
             'Color', 'b', 'LineWidth', 1.5);
    end

    %line for visibility between first points of each layer
     line([xCoords((j-1)*measurementNum + 1), xCoords(j*measurementNum + 1)], ...
     [zCoords((j-1)*measurementNum + 1), zCoords(j*measurementNum + 1)], ...
     [yCoords((j-1)*measurementNum + 1), yCoords(j*measurementNum + 1)], ...
     'Color', 'r', 'LineWidth', 2); 
end

plot3(xCoords, zCoords, yCoords, 'b');

for j = 1:(layerNum - 1)
    for i = 1:(measurementNum-1)
        index_current = (j - 1) * measurementNum + i;
        index_next = j * measurementNum + i;

        line([xCoords(index_current), xCoords(index_next)], ...
             [zCoords(index_current), zCoords(index_next)], ...
             [yCoords(index_current), yCoords(index_next)], ...
             'Color', 'b', 'LineWidth', 1.5);
    end
end

hold off;

%for parsing and translating serial data
function translate_data = translate(data)
    incoming_string = string(data); %converts incoming data to a string 
    parsedVar = strsplit(incoming_string); %splits the string into values

    disp("INCOMING VALUES: " + parsedVar(1) + ", " + parsedVar(2)+ ", " + parsedVar(3));

    distanceVar = str2double(parsedVar(1)); %convert distance to number(duble)
    currAngle = str2double(parsedVar(2)) / (512 / 360); %convert and scale angle #
    spinStat = str2num(parsedVar(3)); %set spin stat to a number

    translate_data = [distanceVar, currAngle, spinStat];
end

%get 3d coords from distance and angle
function getDistance = calculate(distanceVar, layer, currAngle)
    x = distanceVar * sind(currAngle);
    y = distanceVar * cosd(currAngle);
    z = layer;
    
    getDistance = [x, y, z];
end