vote = zeros(25,18)
for row = 1:25
    for col = 1:18
        vote(row,col) = vote2(col + (row - 1) * 18);
    end
end
figure
image(vote);