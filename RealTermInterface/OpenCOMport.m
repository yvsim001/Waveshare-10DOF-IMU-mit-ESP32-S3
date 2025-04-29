% based on code from shimmer
function object = OpenCOMport(ComPort, baudrate)
    object = initialiserealterm(ComPort, baudrate);   % Define and open realterm server
    try
        object.Hrealterm.PortOpen = true;                        % Open the COM Port
    catch
        fprintf(strcat('Warning: opencomport - Unable to open Com Port ',ComPort,'.\n'));
    end

    if(object.Hrealterm.PortOpen~=0)                          % TRUE if the COM Port opened OK
        invoke(object.Hrealterm,'startcapture');              % Enable realtime buffer
        object.FilePointer = 0;                               % Set FilePointer to start of file
        object.isOpen=1;
        object.SerialDataOverflow = [];                                           % Feedback buffer used by the framepackets method
        object.nFramePacketErrors = 0;                                            % Packet error count used by the framepackets method
        object.nBytesDataPacket = 0;
        object.DATA_PACKET_START_BYTE = 0;
        object.OverflowTmp = [];
    else
        CloseCOMport(object);                                   % TRUE if COM Port didnt open close realterm server
        object.isOpen=0;
    end


function object = initialiserealterm(ComPort, baudrate)
    object.Hrealterm = actxserver('realterm.realtermintf');    % Start Realterm as a server
    object.Hrealterm.baud = baudrate;
    object.Hrealterm.Port = ComPort;               % Assign the Shimmer Com Port number to the realterm server
    object.Hrealterm.caption = strcat('Matlab Realterm Server COM',ComPort);   % Assign a title to the realterm server window
    object.Hrealterm.windowstate = 1;                          % Minimise realterm server window
    realtermBufferDirectory = strcat(pwd,'\realtermBuffer');        % Define directory for realtermBuffer

    if ~(exist(realtermBufferDirectory,'dir'))                    % if realtermBuffer directory does not exist then create it
        mkdir(realtermBufferDirectory);
    end

    object.Hrealterm.CaptureFile=strcat(realtermBufferDirectory,'\matlab_data_COM',ComPort,'.dat');    % define realterm buffer file name
    object.isInitialised = true;
end % function initialiserealterm
end