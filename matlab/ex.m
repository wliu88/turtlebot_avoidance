xs = 118;
ys = 29;
deltaX = 120-118;
deltaY = 188-29;
num = deltaY;
deltaX = deltaX / num;
deltaY = deltaY / num;
for index = 0: num
    x = xs + index * deltaX;
    y = ys + index * deltaY;
    bw(floor(y),floor(x)) = 0;
    bw(floor(y),floor(x)+2) = 0;
    bw(floor(y),floor(x)-2) = 0;
    bw(floor(y),floor(x)+1) = 0;
    bw(floor(y),floor(x)-1) = 0;
end

xs = 82;
ys = 36;
deltaX = 82-82;
deltaY = 172-36;
num = deltaY;
deltaX = deltaX / num;
deltaY = deltaY / num;
for index = 0: num
    x = xs + index * deltaX;
    y = ys + index * deltaY;
    bw(floor(y),floor(x)) = 0;
    bw(floor(y),floor(x)+2) = 0;
    bw(floor(y),floor(x)-2) = 0;
    bw(floor(y),floor(x)+1) = 0;
    bw(floor(y),floor(x)-1) = 0;
end

