clear all;
clc;

%% CLIENT 2

data = 1:50;
% plot(data);

t = tcpip('localhost', 30000, 'NetworkRole', 'client');
fopen(t);

fwrite(t, data)

print = "Sent"

clear t;

% pause(0.01);

% data_read = fread(t);
% plot(data_read);