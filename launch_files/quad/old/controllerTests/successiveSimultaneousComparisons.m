%reads in the two csvs and we will use them for comparison

simultaneousDelta = csvread("SimultaneousTest.csv");
successiveDelta = csvread("SuccessiveTest.csv");


%plots each of the eight input controls for the system.

%elevator
figure(1);
plot(simultaneousDelta(1,:));
hold on;
plot(successiveDelta(1,:));
hold off;
title("Elevator");
legend('simultaneous', 'successive');

%aileron
figure(2);
plot(simultaneousDelta(2,:));
hold on;
plot(successiveDelta(2,:));
hold off;
title("Aileron");
legend('simultaneous', 'successive');

%rudder
figure(3);
plot(simultaneousDelta(3,:));
hold on;
plot(successiveDelta(3,:));
hold off;
title("Rudder");
legend('simultaneous', 'successive');

%forward throttle
figure(4);
plot(simultaneousDelta(4,:));
hold on;
plot(successiveDelta(4,:));
hold off;
title("Forward Throttle");
legend('simultaneous', 'successive');

%Vertical Throttle 1
figure(5);
plot(simultaneousDelta(5,:));
hold on;
plot(successiveDelta(5,:));
hold off;
title("Vertical Throttle 1");
legend('simultaneous', 'successive');

%Vertical Throttle 2
figure(6);
plot(simultaneousDelta(6,:));
hold on;
plot(successiveDelta(6,:));
hold off;
title("Vertical Throttle 2");
legend('simultaneous', 'successive');

%Vertical throttle 3
figure(7);
plot(simultaneousDelta(7,:));
hold on;
plot(successiveDelta(7,:));
hold off;
title("vertical Throttle 3");
legend('simultaneous', 'successive');

%Vertical throttle 4
figure(8);
plot(simultaneousDelta(8,:));
hold on;
plot(successiveDelta(8,:));
hold off;
title("vertical Throttle 4");
legend('simultaneous', 'successive');