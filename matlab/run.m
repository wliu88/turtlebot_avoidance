close all
%[arr, arrNaN, arrAng] = DataAnalysis('beforeHough.txt');
%DataAnalysis('afterHough.txt');

%[arr, arrNaN, arrAng] = DataAnalysis('0.5mleftnearwall.txt');
%[arr, arrNaN, arrAng] = DataAnalysis('1.3mleftnearwall.txt');
%[arr, arrNaN, arrAng] = DataAnalysis('1.9mleftatwall.txt');
%[arr, arrNaN, arrAng] = DataAnalysis('1mleftcenter.txt');
%[arr, arrNaN, arrAng] = DataAnalysis('1mrightbetweencenterwall.txt');

%[arr, arrNaN, arrAng] = DataAnalysis('afterHough.txt');
%[arr,arrNaN,arrAng ]= DataAnalysis('4mcenter.txt');
%[arr,arrNaN,arrAng ]= DataAnalysis('7mcenter.txt');
%[arr,arrNaN,arrAng ]= DataAnalysis('4mleft.txt');
%[arr,arrNaN,arrAng ]= DataAnalysis('1mright.txt');
%[arr,arrNaN,arrAng ]= DataAnalysis('-20.txt');
%[arr,arrNaN,arrAng ]= DataAnalysis('-45.txt');
%[arr,arrNaN,arrAng ]= DataAnalysis('35.txt');
%[arr,arrNaN,arrAng ]= DataAnalysis('walkalongwallright.txt');
%[arr,arrNaN,arrAng ]= DataAnalysis('straightWalking.txt');
%[arr,arrNaN,arrAng ]= DataAnalysis('hallway_backwards.txt');
%Detection(arr,arrNaN,arrAng);
%[numNaNArr, Attribute1, varArr, spanArr,locateNaNArr] = Segment(arr,arrNaN,arrAng);
%[segArr, attribute, object] = Detection2(arr,arrNaN,arrAng);
%bw = ToBW(arr);
%[arrSeg, arrMin] = HT(arr,arrNaN,arrAng)