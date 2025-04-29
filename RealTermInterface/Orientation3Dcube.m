function [] = Orientation3Dcube()
    close all;

    ComPortNumber = 3; 
    baudrate = 115200;

    obj = OpenCOMport(num2str(ComPortNumber), baudrate);
    if(obj.Hrealterm.PortOpen == 0)
        error('Connection could not be established');
    end

    % Cube vertex definitions
    cubDim = 3;
    vertex_matrix = [-cubDim -cubDim -cubDim;
                      cubDim -cubDim -cubDim;
                      cubDim  cubDim -cubDim;
                     -cubDim  cubDim -cubDim;
                     -cubDim -cubDim  cubDim;
                      cubDim -cubDim  cubDim;
                      cubDim  cubDim  cubDim;
                     -cubDim  cubDim  cubDim];                
    
    faces = [1 2 6 5;
             2 3 7 6;
             3 4 8 7;
             4 1 5 8;
             1 2 3 4;
             5 6 7 8];

    stopstreaming = 0;

    h = figure('Name','Cube representing orientation of sensor','Position', ...
            [10  150  1200  872]);     
  
    p = patch('vertices',vertex_matrix,'faces',faces,...
              'FaceVertexCData',hsv(6),'FaceColor','flat');   
    axis square;
    title(gca,'Cube representing sensor orientation');
    view(-135, 30);
    
    AX_LIM = 35;
    axis([-AX_LIM AX_LIM -AX_LIM AX_LIM -AX_LIM AX_LIM]);
    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');

    textHandle = text(5, -75, sprintf('Roll: %.1f, Pitch %.1f Yaw %.1f', 0, 0, 0),'FontSize',15);

    uicontrol('Style', 'pushbutton', 'String', 'Stop',...
              'Position', [220 20 50 20],...
              'Callback', {@stop});
    
    set(h, 'CloseRequestFcn', @closeFigureCallback);

    NUM_DATA_ELEMENTS_PER_ROW = 7;

    while(~stopstreaming)                                       
        [obj, SensorData] = GetSerialData(obj);                
        SensorData = cell2mat(SensorData(1:end,:));                 
        
        if(size(SensorData,2) >= NUM_DATA_ELEMENTS_PER_ROW)                              
            TimeStamp = SensorData(:,1);
            AccData = SensorData(:,2:4);  
            GyroData = SensorData(:,5:7);

            % Moyenne pour réduire le bruit
            Acc = mean(AccData, 1); 
            ax = Acc(1);
            ay = Acc(2);
            az = Acc(3);
            
            % Calcul des angles en radians
            roll  = atan2(ay, az);
            pitch = atan2(-ax, sqrt(ay^2 + az^2));
            yaw   = 0; % simplifié, faute de magnétomètre
            
            % Conversion en degrés
            roll  = rad2deg(roll);
            pitch = rad2deg(pitch);


            % Mise à jour du texte affiché
            set(textHandle, 'String', sprintf('Roll: %.1f, Pitch %.1f Yaw %.1f', roll, pitch, yaw));

            % Création du quaternion (ZYX convention: Yaw, Pitch, Roll)
            A_q_B = quaternion([yaw, pitch, roll], 'eulerd', 'ZYX', 'frame');

            % Rotation de chaque point
            vertex_matrix_rot = rotatepoint(A_q_B, vertex_matrix);

            set(p, 'vertices', vertex_matrix_rot);
            drawnow;         
        end    
    end    

    CloseCOMport(obj);

    function stop(~,~) 
        stopstreaming = 1;
    end

    function closeFigureCallback(~, ~)
        stopstreaming = 1;
        delete(gcf)
    end
end
