
%% SERVER

while (1)
    
    clear all;
    clc;
    
    t = tcpip('0.0.0.0', 30000, 'NetworkRole', 'server', 'Timeout', 60);
    fopen(t);
        
    data_read = fread(t, t.BytesAvailable);
    plot(data_read);
    hold on
    drawnow
    

    % Send data if needed
    
%         data = 60:110;
%         fwrite(t, data)

end