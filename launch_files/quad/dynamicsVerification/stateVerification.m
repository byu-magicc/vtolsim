%implements the state verification of the sytems


%reats in the mavsim state
mavState = csvread('mavsimState.csv');
vtolState = csvread('vtolsimState.csv');

mavEuler = csvread('mavsimEuler.csv');
vtolEuler = csvread('vtolsimEulerAngles.csv');



%gets the positional vectors
mavPosition = mavState(1:3,:);
vtolPosition = vtolState(1:3,:);

%gets the velocity vectors
mavVelocity = mavState(4:6,:);
vtolVelocity = vtolState(4:6,:);

%gets the omegas
mavOmega = mavState(10:12,:);
vtolOmega = vtolState(10:12,:);

figure(1);
plot(mavPosition(1,:));
hold on;
plot(vtolPosition(1,:));
hold off;
title("X Position");
legend('mav', 'vtol');

figure(2);
plot(mavPosition(2,:));
hold on;
plot(vtolPosition(2,:));
hold off;
title("Y Position");
legend('mav', 'vtol');

figure(3);
plot(mavPosition(3,:));
hold on;
plot(vtolPosition(3,:));
hold off;
title("Z Position");
legend('mav', 'vtol');



figure(4);
plot(mavEuler(1,:));
hold on;
plot(vtolEuler(1,:));
hold off;
title('Phi');
legend('mav', 'vtol');

figure(5);
plot(mavEuler(2,:));
hold on;
plot(vtolEuler(2,:));
hold off;
title('Theta');
legend('mav', 'vtol');

figure(6);
plot(mavEuler(3,:));
hold on;
plot(vtolEuler(3,:));
hold off;
title('Psi');
legend('mav', 'vtol');