function Detection(arr,arrNaN,arrAng)

    scan = 50;

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
  
%     %for 3D plot extremely slow
%     cellLength = 1;
%     dataLength = length(arr{1});
%     x = 1:dataLength;
%     for index = 1:cellLength
%         plot3(x,ones(dataLength).*index,arr{index});
%         hold on
%     end
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')

    % 2. object detection
    % this threshold is for detecting the jump between scan points.
    threshold = 1;
    objectBegin = 0;
    objectEnd = 0;
    detectionOn = false;
    detected = false;
    
    numArray = arr{scan};
    numLength = length(numArray);
    
    %(1) manipulate numArray to get rid of NaN. This gives every NaN its
    % left adjecent real value.
    numArray(1) = 2;
    for i = 1:numLength-1
       if(isnan(numArray(i+1)))
           numArray(i+1) = numArray(i);
       end
    end
    %plot(numArray,'y')
    
    %(2) This part is for detecting a segment that has jump on both ends.
    % It will not give the object itself because of the manipulation we do
    % to get rid of NaN.
    for i = 2:numLength
        diff = abs(numArray(i) - numArray(i-1));
        if (diff >= threshold)
            if (detectionOn == false && detected == false)
                objectBegin = i;
                detectionOn = true;
            else
                % here i-1 is used to avoid error
                objectEnd = i-1;
                detectionOn = false;
                detected = true;
            end
        end
    end
    
    %(3) This part will give the true edges of the object. Effect of NaN 
    % will be eliminated.
    object = zeros(1,640);
    object(objectBegin:objectEnd) = 1;
    % below is used to make NaN positions all have value of zero.
    object = object.*(~arrNaN{50}); 
    index = find(object==1);
    objectBegin = min(index);
    objectEnd = max(index);
    object = object.*numArray;
    plot(object,'c');
    
    %(4) This part will use the detected objectBegining and objectEnd to
    % calculate an average of object position.
    
    avg = sum(object)/sum(object~=0);
    avgArr = zeros(1,575);
    avgArr(objectBegin:objectEnd) = avg;
    plot(avgArr,'y');

end


