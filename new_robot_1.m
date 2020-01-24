clear all;
clc;

%% NEW ROBOT 1

isCellOccupied = 3;

table = [4];

waiting_for_answer = true;

while (waiting_for_answer == true)
    
    waiting_for_answer = false;
    if (isCellOccupied > 1)
       waiting_for_answer = true; 
    end
    
    %% Receive server flag
    server_call = tcpip('localhost', 11111, 'NetworkRole', 'client');
    fopen(server_call);
    while (server_call.BytesAvailable == 0)
    end
    server_flag = fread(server_call, server_call.BytesAvailable, 'int8');
    fclose(server_call);
    
    %% Answer depending on server flag
    
    % Free to make request
    if (server_flag == -1)          
        fprintf("No server request\n");
        message = isCellOccupied;
        % No request to make
        if (isCellOccupied == -1)
            fprintf("No request\n");
        % Make request
        else
            fprintf("Request sent (%d)\n", message);
        end
        
    % Answer to your previous request
    elseif (server_flag == 0 || server_flag == 1)       
        fprintf("Answer to previous request\n");
        message = -1;
        waiting_for_answer = false;
        fprintf("Thank you (%d)\n", message);
        
    % Forwarded request from other client
    else                            
        fprintf("Request from other client (%d)\n", server_flag);
        message = int8(ismember(server_flag, table));
        fprintf("Answer to client 2 (%d)\n", message);
    end
    
    data_send = tcpip('localhost', 11111, 'NetworkRole', 'client');
    fopen(data_send);
    fwrite(data_send, message, 'int8');
    fclose(data_send);
    
    pause(2);

end