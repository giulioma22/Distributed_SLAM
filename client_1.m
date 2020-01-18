clear all;
clc;

%% CLIENT 1

data = 10:60;
% plot(data);

t = tcpip('localhost', 30000, 'NetworkRole', 'client');
fopen(t);

fwrite(t, data)

print = "Sent"

clear t;

% pause(0.01);

% data_read = fread(t);
% plot(data_read);