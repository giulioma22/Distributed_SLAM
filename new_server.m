clear all;
clc;

%% NEW SERVER

flag_1 = -1;

 while (1)
 
    % Send flag to client 1
    data_flag_1 = tcpip('0.0.0.0', 11111, 'NetworkRole', 'server');
    fopen(data_flag_1);
    fwrite(data_flag_1, flag_1, 'int8')
    fprintf("Server - To client 1 = %d\n", flag_1);
    fclose(data_flag_1);
 
    % Get answer from client 1
    data_rec_1 = tcpip('0.0.0.0', 11111, 'NetworkRole', 'server');
    fopen(data_rec_1);
    while (data_rec_1.BytesAvailable == 0)
    end
    answer_1 = fread(data_rec_1, data_rec_1.BytesAvailable, 'int8')
    fclose(data_rec_1);
   
    flag_2 = answer_1;
    
 %%
 
    % Send flag to client 2
    data_flag_2 = tcpip('0.0.0.0', 22222, 'NetworkRole', 'server');
    fopen(data_flag_2);
    fwrite(data_flag_2, flag_2, 'int8')
    fprintf("Server - To client 2 = %d\n", flag_2);
    fclose(data_flag_2);
 
    % Get answer from client 2
    data_rec_2 = tcpip('0.0.0.0', 22222, 'NetworkRole', 'server');
    fopen(data_rec_2);
    while (data_rec_2.BytesAvailable == 0)
    end
    answer_2 = fread(data_rec_2, data_rec_2.BytesAvailable, 'int8')
    fclose(data_rec_2);

    flag_1 = answer_2;
    
 end