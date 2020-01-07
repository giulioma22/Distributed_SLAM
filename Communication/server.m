clear all;
clc;

t = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server');

fopen(t);

data_read = fread(t, t.BytesAvailable);
plot(data_read);

data = 60:110;
fwrite(t, data)