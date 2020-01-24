clear all;
clc;

%% NEW ROBOT 2

SLAM_completed = false;

while (~SLAM_completed)
    
    isCellOccupied = 3;

    table = [2];

    waiting_for_answer = true;

    while (waiting_for_answer == true)

        % Exit loop iff no pending response
        waiting_for_answer = false;
        if (isCellOccupied > 1)
           waiting_for_answer = true; 
        end

        %% Receive server flag
        server_call = tcpip('localhost', 22220, 'NetworkRole', 'client');
        fopen(server_call);
        while (server_call.BytesAvailable == 0)
        end
        server_flag = fread(server_call, server_call.BytesAvailable, 'int8');
        fclose(server_call);

        %% Answer depending on server flag

        if (server_flag == -1)          % Free to make request
            fprintf("No server request (-1)\n");
            message = isCellOccupied;
            % No request to make
            if (isCellOccupied == -1)
                fprintf("No request (-1)\n");
            % Make request
            else
                fprintf("Request sent (%d)\n", message);
            end
        elseif (server_flag == 0 || server_flag == 1)       % Answer to your previous request
            fprintf("Answer to previous request (%d)\n", server_flag);
            message = -1;
            waiting_for_answer = false;
            fprintf("No new request (%d)\n", message);
        else                            % Forwarded request from other client
            fprintf("Request from other client: cell = %d\n", server_flag);
            message = int8(ismember(server_flag, table));
            fprintf("Answer to client 1 (%d)\n", message);
        end

        data_send = tcpip('localhost', 22222, 'NetworkRole', 'client');
        fopen(data_send);
        fwrite(data_send, message, 'int8');
        fclose(data_send);

    %     pause(1);

    end

end