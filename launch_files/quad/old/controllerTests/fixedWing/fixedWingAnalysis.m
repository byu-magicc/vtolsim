%reads in the csv 


desiredWrench = csvread('desiredWrench.csv');
actualWrench = csvread('actualForceTorque.csv');


%plots each one out
Fx_desired= desiredWrench(1,:);
Fy_desired = desiredWrench(2,:);
Fz_desired = desiredWrench(3,:);
Mx_desired = desiredWrench(4,:);
My_desired = desiredWrench(5,:);
Mz_desired = desiredWrench(6,:);

%saves each component
Fx_actual = actualWrench(1,:);
Fy_actual = actualWrench(2,:);
Fz_actual = actualWrench(3,:);
Mx_actual = actualWrench(4,:);
My_actual = actualWrench(5,:);
Mz_actual = actualWrench(6,:);


figure(1);
plot(Fx_desired);
hold on;
plot(Fx_actual);
hold off;
title('Fx');
legend('Desired', 'Actual');

figure(2);
plot(Fz_desired);
hold on;
plot(Fz_actual);
hold off;
title('Fz');
legend('Desired', 'Actual');

figure(3);
plot(Mx_desired);
hold on;
plot(Mx_actual);
hold off;
title('Mx');
legend('Desired', 'Actual');

figure(4);
plot(My_desired);
hold on;
plot(My_actual);
hold off;
title('My');
legend('Desired', 'Actual')


%reads in the desired and actual deltas
desiredDelta = csvread('desiredDelta.csv');
actualDelta = csvread('deltasActual.csv');

%gets the desired deltas
elevatorDesired = desiredDelta(1,:);
aileronDesired = desiredDelta(2,:);
throttleDesired = desiredDelta(3,:);

%gets the actual delta
elevatorActual = actualDelta(1,:);
aileronActual = actualDelta(2,:);
throttleActual = actualDelta(3,:);

figure(5);
plot(elevatorDesired);
hold on;
plot(elevatorActual);
hold off;
title("Elevator");
legend('Desired', 'Actual')

figure(6);
plot(aileronDesired);
hold on;
plot(aileronActual);
hold off;
title("Aileron");
legend('Desired', 'Actual')


figure(7);
plot(throttleDesired);
hold on;
plot(throttleActual);
hold off;
title("Throttle");
legend('Desired', 'Actual')

