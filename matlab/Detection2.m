% 2015.02.03
% Created by Weiyu Liu
% This program is used to detect objects
% The first step is to find significant jump in the raw data, including
% numerical jump, jump from NaN to real number, from real number to NaN.
% This will give us a few segments within which a consistent behavior will
% be observed. Then the second step is to determine the behaviors of these
% segments.

function [segArr, attribute, object] = Detection2(arr,arrNaN,arrAng)
%function [segArr, attributeWall, attributeVar, attributeMean, attributeCenter, attributeRadius] = Detection2(arr,arrNaN,arrAng)
scan = 1;

% 1. ploting
% 2D plot
%cellLength = length(arr);
% this can be changed to make plot of all scans.
for index = scan:scan
    hold on
    plot(arr{index},'r');
    %plot(arrAng{index},'g');
    plot(arrNaN{index},'.');
end

numArray = arr{scan};
numLength = length(numArray);


% 2. find segments
previous = 100;
current = 0;
diff = 0;
jumpThreshold = 1;
wallThreshold = 5;
segArr = [];
attributeWall = []; % 1 stands for wall
index = 1; 


while (index < numLength)
    current = numArray(index);
    if (isnan(current))
        index2 = index;
        count = 0;
        while (isnan(current) && index2 < numLength)
            count = count + 1;
            index2 = index2 + 1;
            current = numArray(index2);
        end
        if (count > wallThreshold)
            segArr = [segArr, index];
            attributeWall = [attributeWall, 1];
            index = index2 - 1;
            previous = 100;
        end     
    else
        diff = abs(current - previous);
        if (diff > jumpThreshold)
            segArr = [segArr, index];
            attributeWall = [attributeWall, 0];
        end
        previous = current;
    end
    index = index + 1;
end
segArr = [segArr, numLength]; % add the ending point of data
display(segArr);



% 3.
numSeg = length(attributeWall);

attributeVar = [];
attributeMean = [];
attributeCenter = [];
attributeRadius = [];
attributeCV = [];
attribute = [];

for segIndex = 1 : numSeg
   seg = numArray(segArr(segIndex) : segArr(segIndex + 1));
   attributeCenter = [attributeCenter, (segArr(segIndex + 1) + segArr(segIndex))/2];
   attributeRadius = [attributeRadius, (segArr(segIndex + 1) - segArr(segIndex))/2];
   if (attributeWall(segIndex)) 
       attributeVar = [attributeVar, 0];
       attributeMean = [attributeMean, 9.7];
       attributeCV = [attributeCV, 1];
   else
       attributeVar = [attributeVar, var(seg(~isnan(seg)))];
       attributeMean = [attributeMean, mean(seg(~isnan(seg)))];
       attributeCV = [attributeCV, attributeVar/attributeMean];      
   end
end

attribute.Wall = attributeWall;
attribute.Mean = attributeMean;
attribute.Var = attributeVar;
attribute.CV = attributeCV;
attribute.Radius = attributeRadius;
attribute.Center = attributeCenter;
display(attribute)

% 5.
object = [];
objectCenter = [];
objectRadius = [];
objectMean = [];
objectVar = [];
objectCV = [];
for segIndex = 1:numSeg
    if(~attribute.Wall(segIndex)) %cannot be wall
        if(attribute.Mean(segIndex) < 7) %cannot be too far
            if(attribute.CV < 0.2)
                objectCenter = [objectCenter, attribute.Center(segIndex)];
                objectRadius = [objectRadius, attribute.Radius(segIndex)];
                objectMean = [objectMean, attribute.Mean(segIndex)];
                objectVar = [objectVar, attribute.Var(segIndex)];
                objectCV = [objectCV, attribute.CV(segIndex)];
            end
        end
    end
end

object.Center = objectCenter;
object.Radius = objectRadius;
object.Mean =objectMean;
object.Var =objectVar;
object.CV = objectCV;
display(object)

end

