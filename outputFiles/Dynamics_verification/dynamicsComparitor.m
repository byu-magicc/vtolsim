mavsimState = csvread('mavsimState.csv');
vtolsimState = csvread('vtolsimState.csv');


%compares the north position
figure(1);
mavsimNorth = mavsimState(1,:);
vtolsimNorth = vtolsimState(1,:);
plot(mavsimNorth);
hold on;
plot(vtolsimNorth);
hold off;
title("North Position");
legend('mavsim', 'vtolsim');


%compares the altitude position
figure(2);
mavsimAltitude = mavsimState(2,:);
vtolsimAltitude = vtolsimState(2,:);
plot(mavsimAltitude);
hold on;
plot(vtolsimAltitude);
hold off;
title("Altitude Position");
legend('mavsim', 'vtolsim');


