%reads in the actual wrench and the calculated wrench
actualWrench = csvread('actualWrench.csv');
calculatedWrench = csvread('calculatedWrench.csv');


%compares the fx
figure(1);
plot(actualWrench(1,:));
hold on;
plot(calculatedWrench(1,:));
hold off;
title('Fx')
legend('actual', 'calculated');



%compares the fz
figure(2);
plot(actualWrench(3,:));
hold on;
plot(calculatedWrench(2,:));
hold off;
title('Fz')
legend('actual', 'calculated');


%compares the Mx
figure(3);
plot(actualWrench(4,:));
hold on;
plot(calculatedWrench(3,:));
hold off;
title('Mx')
legend('actual', 'calculated');



%compares the My
figure(4);
plot(actualWrench(5,:));
hold on;
plot(calculatedWrench(4,:));
hold off;
title('My')
legend('actual', 'calculated');


%compares the Mz
figure(5);
plot(actualWrench(6,:));
hold on;
plot(calculatedWrench(5,:));
hold off;
title('Mz')
legend('actual', 'calculated');