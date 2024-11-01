mavsimDeltas = csvread('mavsimDeltas.csv');
mavsimWrench = csvread('mavsimWrench.csv');

vtolsimDeltas = csvread('vtolsimDeltas.csv');
vtolsimWrench = csvread('vtolsimWrench.csv');


%goes through and plots both of the wrenches
for i = 1:6
    figure(i);
    plot(mavsimWrench(i,:));
    hold on;
    plot(vtolsimWrench(i,:));
    hold off;
    legend('mavsim', 'vtolsim');
    title('wrench', i);
end

offset = 6;

for i = (1+offset):(4+offset)
    figure(i);
    plot(mavsimDeltas(i-offset,:));
    hold on;
    plot(vtolsimDeltas(i-offset,:));
    hold off;
    legend('mavsim', 'vtolsim');
    title('delta', i-offset);
end
