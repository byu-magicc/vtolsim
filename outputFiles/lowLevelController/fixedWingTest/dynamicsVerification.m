%reads in the calculated wrench vector
calculatedWrench = csvread('calculatedWrench.csv');
actualWrench = csvread('wrenchReference.csv');
mavsimWrench = csvread('mavsimWrench.csv');


%plots the forces in the x direction
figure(1);
plot(calculatedWrench(1,:));
hold on;
plot(actualWrench(1,:));
plot(mavsimWrench(1,:));
hold off;
legend('calculated', 'actual', 'mavsim');
title("Fx");

%plots the forces in the x direction
figure(2);
plot(calculatedWrench(2,:));
hold on;
plot(actualWrench(3,:));
plot(mavsimWrench(3,:));
hold off;
legend('calculated', 'actual', 'mavsim');
title("Fz");

%plots the forces in the x direction
figure(3);
plot(calculatedWrench(3,:));
hold on;
plot(actualWrench(4,:));
plot(mavsimWrench(4,:));
hold off;
legend('calculated', 'actual', 'mavsim');
title("Mx");


%plots the forces in the x direction
figure(4);
plot(calculatedWrench(4,:));
hold on;
plot(actualWrench(5,:));
plot(mavsimWrench(5,:));
hold off;
legend('calculated', 'actual', 'mavsim');
title("My");


%plots the forces in the x direction
figure(5);
plot(calculatedWrench(5,:));
hold on;
plot(actualWrench(6,:));
plot(mavsimWrench(6,:));
hold off;
legend('calculated', 'actual', 'mavsim');
title("Mz");