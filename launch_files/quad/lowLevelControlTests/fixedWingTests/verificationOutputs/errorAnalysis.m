%reads in the wrench error
wrenchError = csvread('wrenchError.csv');

%extracts the components
fx = wrenchError(1,:);
fz = wrenchError(2,:);
Mx = wrenchError(3,:);
My = wrenchError(4,:);
Mz = wrenchError(5,:);


for i = 1:5
    figure(i);
    plot(wrenchError(i,:));
    title('Component: ',i);
end