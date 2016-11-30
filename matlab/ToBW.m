function bw = ToBW(arr)

% 1. this part convert the polar coordinate to cartesian coordinate
numArray = arr{1};
% the angle may not be centered
ang = linspace(-27.32,35.32,640);
y = cosd(ang).*numArray;
x = sind(ang).*numArray;
plot(y,x);

% % 2. this part convert all the data point to the binary image
% bw = zeros(200,200);
% for index = 1:640
%    if (y(index) < 10 && y(index) > 0 && x(index) < 5 && x(index) > -5)
%        bw(floor(y(index)/0.05), floor((x(index) + 5)/0.05)) = 1;
%    end
% end
% 
% figure;
% imshow(bw);
% 
% [H, theta, rho] = hough(bw);
% peaks = houghpeaks(H, 2);
% lines = houghlines(bw, theta, rho, peaks);
% 
% lines(1)
% lines(2)
% 
% xs = lines(1).point1(1);
% ys = lines(1).point1(2);
% deltaX = lines(1).point2(1)-lines(1).point1(1);
% deltaY = lines(1).point2(2)-lines(1).point1(2);
% num = deltaY;
% deltaX = deltaX / num;
% deltaY = deltaY / num;
% for index = 0: num
%     x = xs + index * deltaX;
%     y = ys + index * deltaY;
%     bw(floor(y),floor(x)) = 0;
%     bw(floor(y),floor(x)+2) = 0;
%     bw(floor(y),floor(x)-2) = 0;
%     bw(floor(y),floor(x)+1) = 0;
%     bw(floor(y),floor(x)-1) = 0;
% end
% 
% xs = lines(2).point1(1);
% ys = lines(2).point1(2);
% deltaX = lines(2).point2(1)-lines(2).point1(1);
% deltaY = lines(2).point2(2)-lines(2).point1(2);
% num = deltaY;
% deltaX = deltaX / num;
% deltaY = deltaY / num;
% for index = 0: num
%     x = xs + index * deltaX;
%     y = ys + index * deltaY;
%     bw(floor(y),floor(x)) = 0;
%     bw(floor(y),floor(x)+2) = 0;
%     bw(floor(y),floor(x)-2) = 0;
%     bw(floor(y),floor(x)+1) = 0;
%     bw(floor(y),floor(x)-1) = 0;
% end
% 
% figure;
% imshow(bw);

end

