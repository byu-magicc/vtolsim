actualWrench = csvread("actualWrench.csv");
calculatedWrench = csvread('calculatedWrench.csv');

actualWrench = [actualWrench(1,:); actualWrench(3,:); actualWrench(4,:); actualWrench(5,:);actualWrench(6,:)];


error = actualWrench - calculatedWrench;

for i = 1:5
    figure(i);
    plot(actualWrench(i,:));
    hold on;
    plot(calculatedWrench(i,:));
    hold off;
    title("F_", i);
    legend('actual', 'calculated');
end



for i = 1:5
    figure(i+5);
    hold on;
    plot(error(i,:));
    hold off;
    title("Error F_", i);
    legend('actual', 'calculated');
end


