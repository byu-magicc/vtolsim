%reads in the csv file
deltas = csvread('sysIDDeltaOutput.csv');



figure(1);
plot(deltas(1,:));
hold on;
plot(deltas(2,:));
plot(deltas(3,:));
plot(deltas(4,:));
hold off;
title("Control surface inputs");
legend('elevator', 'aileron', 'rudder', 'forward Throttle');

rotorForces = csvread('sysIDRotorForcesOutput.csv');

figure(2);
plot(rotorForces(1,:));
hold on;
plot(rotorForces(2,:));
plot(rotorForces(3,:));
plot(rotorForces(4,:));
plot(rotorForces(5,:));
hold off;
title("rotor Forces");
legend('Forward', 'front port', 'rear port', 'rear starboard', 'front starboard');