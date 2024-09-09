%reads in the csv 

wrench = csvread('fixedWingWrenchOutput.csv');


%plots each one out
Fx = wrench(1,:);
Fy = wrench(2,:);
Fz = wrench(3,:);
Mx = wrench(4,:);
My = wrench(5,:);
Mz = wrench(6,:);


figure(1);
title("Forces");
plot(Fx);
hold on;
plot(Fy);
plot(Fz);
hold off;
legend('Fx', 'Fy', 'Fz');

figure(2);
title('Moments');
plot(Mx);
hold on;
plot(My);
plot(Mz);
hold off;
legend('Mx', 'My', 'Mz');
