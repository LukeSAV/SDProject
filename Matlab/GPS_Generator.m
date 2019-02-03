
points = [];
samples = [];
m = 10
for i = 0:.001:2
    y = i;
    x = (y-2)^3 + 3*(y-2)^2 + 2*(y-2);
    points = [points; m*[x,y]]; %#ok<AGROW>
    
    if mod(m*i, .18) == 0
        samples = [samples; [normrnd(m*x,1), normrnd(m*y,1)]]; %#ok<AGROW>
    end
end
plot(points(:,1), points(:,2), 'b')
hold on;
plot(samples(:,1), samples(:,2), 'rx')
xlabel("y");
ylabel("x");
title("Sample GPS Datapoints");

