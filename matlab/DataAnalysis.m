function [arr,arrNaN,arrAng]= DataAnalysis(name)

    % this function is used to extract the bagfile of scans generated from
    % turtlebot. This function transfer the original text file to cell
    % arrays containing one scan in one cell. Another two cell arrays have
    % been created. The numArrayNaN contains points where NaN exists.
    arr = {};
    arrNaN = {};
    arrAng = {};

    fh = fopen(name);
    line = fgetl(fh);
    angle = linspace(-37,37,640); 
    % the angle, after discarding the NaN values at two ends, this is not accurate, waiting for revision.
    while (ischar(line))
        if(line(1) ~= '-')
            numArray = str2num(line);
            numArray = numArray(end:-1:1);
            %numArray = numArray(22:596);
            numArrayAng = numArray.*cosd(angle);
            numArrayNaN = isnan(numArray)*2;
            arr = [arr,numArray];
            arrNaN = [arrNaN,numArrayNaN];
            arrAng = [arrAng,numArrayAng];
        end
        line = fgetl(fh);
    end
    
    scan = 10;

% % 1. ploting
% % 2D plot
% %cellLength = length(arr);
% % this can be changed to make plot of all scans.
% figure
% for index = scan:scan
%     hold on
%     plot(arr{index},'r');
%     %plot(arrAng{index},'g');
%     plot(arrNaN{index},'.');
% end
% 
% % 2. this part convert the polar coordinate to cartesian coordinate
% numArray = arr{1};
% % the angle may not be centered
% ang = linspace(-27.32,35.32,640);
% y = cosd(ang).*numArray;
% x = sind(ang).*numArray;
% figure
% plot(x,y);
    
end

