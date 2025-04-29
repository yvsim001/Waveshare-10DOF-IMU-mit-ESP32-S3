% based on code from shimmer
function [object, SensorData] = GetSerialData(object)

    persistent LastSize;
    SensorData=[];
    bufferedData=[];
    LastEOF = 0;
    if(object.FilePointer==0)
        LastSize = 0;
    end
    
    fileInfo = dir( object.Hrealterm.CaptureFile);
    fileSize = fileInfo.bytes;
    
    if(fileSize ~= LastSize)        
        if (object.isOpen == 1)                        % TRUE if the Sensor is in a Streaming state
            fileId = fopen(object.Hrealterm.CaptureFile, 'r');        % Open file with read only permission
            if (fileId ~= -1)                                              % TRUE if file was opened successfully
                didFileOpen = true;                                        % Set isFileOpen to 1 to indicate that the file was opened
                              % Set file pointer to value stored from previous fread

                status =  fseek(fileId,object.FilePointer,'bof');                 % Set file pointer to value stored from previous fread

                if(status == -1)
                    error('status invalid');
                end
                SerialData=fread(fileId, inf, '*char');             % Read data from the realterm data buffer
                LengthNewPacket = length(SerialData);
                PointerPos = object.FilePointer+LengthNewPacket;  % Update FilePointer value to position of last value read

               % display(['LengthPacket = ' num2str(LengthNewPacket) ' LastSize = ' num2str(LastSize)]);
                if(LengthNewPacket ~= 0)
                    if(object.FilePointer==0)
                        %first reading --> skip first line as it may be
                        %imcomplete
                        bufferedData = textscan(SerialData,'%f %f %f %f %f %f %f','delimiter','\n','whitespace','','HeaderLines',1);
                    else
                        bufferedData = textscan(SerialData,'%f %f %f %f %f %f %f','delimiter','\n','whitespace','');                  
                    end

                    if(size(cell2mat(bufferedData(1,1)),1)> 2)                                       
                        for(i=1:7)
                            SensorData{1,i} = bufferedData{1,i}(1:( size(bufferedData{1},1)-1),1);
                        end
                        for(i=1:LengthNewPacket)
                            %display(num2str(LengthNewPacket));
                            if(strcmp(SerialData(LengthNewPacket-i),sprintf('\n')))
                                LastEOF = PointerPos-i;
                                break;
                            end
                        end

                        if(LastEOF ~= 0)
                            object.FilePointer = LastEOF ;
                        else
                            object.FilePointer = object.FilePointer + PointerPos;
                        end
                    else
                        %display(['length ' num2str(LengthNewPacket) ' pointer: ' num2str(object.FilePointer)]);
                    end
                else
                    display('empty');
                end
                fclose(fileId);
            else
                didFileOpen = false;    % Set isFileOpen to 0 to indicate that the file failed to open
                fprintf(strcat('Warning: readdatabuffer - Cannot open realterm capture file for COM',object.ComPort,'.\n'));
            end

        else
             fprintf(strcat('Warning: COM ',object.ComPort,' not connected'));
        end
        
        LastSize = fileSize;
    end
end
