clear all;
clc;

%% NEW SERVER

% No inital request
flag_1 = -1;

 while (1)
 
     %% Talk to CLIENT 1
     
    % Send flag to client 1
    data_flag_1 = tcpip('0.0.0.0', 11110, 'NetworkRole', 'server');
    fopen(data_flag_1);
    pause(0.2);
    fwrite(data_flag_1, flag_1, 'int8')
    fprintf("Server -->> Client 1 (%d)\n", flag_1);
    fclose(data_flag_1);
 
    % Get answer from client 1
    data_rec_1 = tcpip('0.0.0.0', 11111, 'NetworkRole', 'server');
    fopen(data_rec_1);
    while (data_rec_1.BytesAvailable == 0)
    end
    answer_1 = fread(data_rec_1, data_rec_1.BytesAvailable, 'int8');
    fprintf("Server <<-- Client 1 (%d)\n\n", answer_1);
    fclose(data_rec_1);
   
    % Update flag_2 w/ last message received
    flag_2 = answer_1;
    
    %% Talk to CLIENT 2
 
    % Send flag to client 2
    data_flag_2 = tcpip('0.0.0.0', 22220, 'NetworkRole', 'server');
    fopen(data_flag_2);
    pause(0.2);
    fwrite(data_flag_2, flag_2, 'int8')
    fprintf("Server -->> Client 2 (%d)\n", flag_2);
    fclose(data_flag_2);
 
    % Get answer from client 2
    data_rec_2 = tcpip('0.0.0.0', 22222, 'NetworkRole', 'server');
    fopen(data_rec_2);
    while (data_rec_2.BytesAvailable == 0)
    end
    answer_2 = fread(data_rec_2, data_rec_2.BytesAvailable, 'int8');
    fprintf("Server <<-- Client 2 (%d)\n\n", answer_2);
    fclose(data_rec_2);

    % Update flag_1 w/ last message received
    flag_1 = answer_2;
    
 end