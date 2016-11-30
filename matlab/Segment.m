function [numNaNArr, Attribute1, varArr, spanArr,locateNaNArr] = Segment(arr,arrNaN,arrAng)
    
    % define which frame of scan to be analyzed
    scan = 1;

    % 1. plot
    % 2D plot
    %cellLength = length(arr);
    % this can be changed to make plot of all scans.
    for index = scan:scan
        hold on
        plot(arr{index},'r');
        %plot(arrAng{index},'g');
        plot(arrNaN{index},'.');
    end
    
    % 2. extract data for speficied frame
    numArray = arr{scan};
    numLength = length(numArray);
    numSeg = floor(numLength/10);
    
    % 3. locate NaN in the laser data.
    locateNaNArr = [];
    for i = 1:numLength
        if (isnan(numArray(i)))
           locateNaNArr = [locateNaNArr, i]; 
        end
    end
    display(locateNaNArr);
    
    % 4. divide laser into segments.
    
    % this array will have 64 packets, each contain the number of NaN in
    % this 10 data points packet
    numNaNArr = [];
    
    % this array will have 64 packets, each contain a sign number to
    % distinguish defferent situations after the first filter
    % sign number 3 : noise                      < 30% NaN
    %             2 : uncertatin                 30% - 70% NaN
    %             1 : too far or too close       > 70% NaN
    Attribute1 = []; 
    varArr = [];
    spanArr = [];
    for i = 1:numSeg-1
       index = (i-1)*10+1;
       seg = numArray(index:index+10); 
       numNaN = sum(isnan(seg));
       
       varSeg = -0.001;
       spanSeg = -1;
       if (numNaN < 3) 
           varSeg = var(seg(~isnan(seg)));
           spanSeg = max(seg(~isnan(seg))) - min(seg(~isnan(seg)));
       end
       spanArr = [spanArr, spanSeg];
       varArr = [varArr, varSeg*1000];
       
       numNaNArr = [numNaNArr ,ones(1,10).*numNaN];
       sign = 0;
       if (numNaN < 3)
           sign = 3;
       else if (numNaN <= 7)
               sign = 2;
           else
               sign = 1;
           end
       end
       
       if varSeg 
            Attribute1 = [Attribute1, sign];
       end
    end
    display(numNaNArr);
    display(Attribute1);
    display(numNaNArr);
    display(spanArr);
end



