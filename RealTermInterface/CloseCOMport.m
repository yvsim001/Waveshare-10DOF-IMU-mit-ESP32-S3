% based on code from shimmer
function isOpen = CloseCOMport(object)
    object.Hrealterm.PortOpen=0;                            % Close the COM Port

    if(object.Hrealterm.PortOpen~=0)                        % TRUE if the COM Port is still open
        isOpen=1;
        fprintf(strcat('Warning: closecomport - Unable to close COM',object.ComPort,'.\n'));
    else
        isOpen=0;
        closerealterm(object);
    end
    
    
    function isClosed = closerealterm(object)

        invoke(object.Hrealterm,'stopcapture');
        isClosed = true;
        try                                                           % Try to close realterm server
            invoke(object.Hrealterm,'close'); delete(object.Hrealterm);
        catch
            isClosed = false;
            fprintf(strcat('Warning: closerealterm - Unable to close realterm for COM',object.ComPort,'.'))
        end; % try

    end % function closerealterm
end % function closecomport