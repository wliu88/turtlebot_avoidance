function [arrSeg, arrMin] = HT(arr,arrNaN,arrAng)

scan = 1;

% 1. ploting
% 2D plot
%cellLength = length(arr);
% this can be changed to make plot of all scans.
figure
for index = scan:scan
    hold on
    plot(arr{index},'r');
    %plot(arrAng{index},'g');
    plot(arrNaN{index},'.');
end

% 2. this part convert the polar coordinate to cartesian coordinate
numArray = arr{1};
% the angle may not be centered
ang = linspace(-27.32,35.32,640);
y = cosd(ang).*numArray;
x = sind(ang).*numArray;
figure
plot(x,y);

% 3. 
numArray = arr{scan};
numLength = length(numArray);

vote = zeros(25,18);
for index = 1:320
    theta = 28 / 320 * (320 - index);
    rho = numArray(index);
    if (~isnan(rho))
    for index2 = 1:18
       alpha = index2 * 2.5 - 22.5;
       r = round((rho * sind(theta + alpha)) / 0.1);
       if (r >= 1 && r <= 25)
          vote(r, index2) = vote(r, index2) + 1;
       end
    end
    end
end

figure
image(vote);

[a, Ic] = max(vote);
[b, Ir] = max(a);
row = Ir;
column = Ic(Ir);
alpha = row * 2.5 - 22.5;
for index = 1:320
   theta = 28 / 320 * (320 -index);
   rho = numArray(index);
   if (~isnan(rho))
       rmatch = round (rho * sind(theta + alpha) / 0.1);
       if (abs(rmatch - column) < 3);
          numArray(index) = 0;
       end
   end
end



% 4.
vote2 = zeros(25,18);
for index = 320:640
    theta = 28 / 320 * (index - 321);
    rho = numArray(index);
    if (~isnan(rho))
    for index2 = 1:18
       alpha = index2 * 2.5 - 22.5;
       r = round((rho * sind(theta + alpha)) / 0.1);
       if (r >= 1 && r <= 25)
          vote2(r, index2) = vote2(r, index2) + 1;
       end
    end
    end
end

figure
image(vote2);

[a, Ic] = max(vote2);
[b, Ir] = max(a);
row = Ir;
column = Ic(Ir);
alpha = row * 2.5 - 22.5;
for index = 320:640
   theta = 28 / 320 * (index - 321);
   rho = numArray(index);
   if (~isnan(rho))
       rmatch = round (rho * sind(theta + alpha) / 0.1);
       if (abs(rmatch - column) < 3); % here setting the threshold for deleting the wall
          numArray(index) = 0;
       end
   end
end

figure
plot(numArray);
title('after')

ang = linspace(-27.32,35.32,640);
y = cosd(ang).*numArray;
x = sind(ang).*numArray;
figure
plot(x,y);


% 2. find segments
index = 2;
beginPoint = 0;
endPoint = 0;
count = 0;
threshold = 0.5;
arrSeg = [];
arrMin = [];
min = 0;


while (index < 640)
    current = numArray(index);
    if (~isnan(current))
        if (current ~= 0)
            if (beginPoint == 0)
                beginPoint = index;
                arrSeg = [arrSeg, beginPoint];
                endPoint = index;
                min = current;
            end
            if (current < min)
                min = current;
            end
            count = 0;
            index = index + 1;
            continue;
        end
    end
    count = count + 1;
    if (count > 5)
        if (beginPoint ~= 0)
            endPoint = index - count;
            if (endPoint - beginPoint > 20)
                arrSeg = [arrSeg, endPoint];
                arrMin = [arrMin, min];
                beginPoint = 0;
                count = 0;
            else
                arrSeg(end) = [];
                count = 0;
                beginPoint = 0;
                endPoint = 0;
            end
        end
    end
    index = index + 1;
end



% % 2. find segments
% objectThreshold = 0.5;
% index = 1; 
% 
% 
% while (index < 640)
%     if (index == 220)
%         c = 0;
%     end
%     current = numArray(index);
%     if (~isnan(current))
%        if (current ~= 0)
%            index2 = index;
%            next = numArray(index2);
%            while ((abs(next - current) < objectThreshold) && (index2 < 640))
%                 index2 = index2 + 1;
%                 next = numArray(index2);
%                 count = 0;
%                 while (isnan(next) || next == 0)
%                    count = count + 1; 
%                    index2 = index2 + 1;
%                    next = numArray(index2);
%                    if (count > 5)
%                        break;
%                    end
%                 end
%                 if (count > 5)
%                     break;
%                 end
%            end
%            if (index2 - index > 20)
%               break;
%            end
%        end
%     end
%     index = index2 + 1;
% end
% 
% index
% index2
end


