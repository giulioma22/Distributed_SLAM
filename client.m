clear all;
clc;

data = 1:50;
% plot(data);

t = tcpip('localhost', 30000, 'NetworkRole', 'client');
fopen(t);

fwrite(t, data)

% pause(0.01);

data_read = fread(t);
plot(data_read);