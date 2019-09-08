
function out = example_mti_receive_data()
warning off;


%% receive data
accValues = [];
accTimestamps = [];
gyrValues=[];
gyrTimestamps=[];

orientation=[];
relative_orientation=[];
counter_r = 1;
timestamp=[];
q1 = [1 0 0 0];
q2 = [1 0 0 0];
t1 =[];
t2 = [];
acc1 = [];
acc2 = [];
gyr1=[];
gyr2=[];
%% Launch activex server
try
    switch computer
        case 'PCWIN'
            h = actxserver('xsensdeviceapi_com32.IXsensDeviceApi');
        case 'PCWIN64'
            h = actxserver('xsensdeviceapi_com64.IXsensDeviceApi');
        otherwise
            error('XDA:os','Unsupported OS');
    end
catch e
    fprintf('\nPlease reinstall MT SDK or check manual,\n Xsens Device Api is not found.')
    rethrow(e);
end
fprintf('\nActiveXsens server - activated \n');

version = h.xdaVersion;
fprintf('Using XDA version: %.0f.%.0f.%.0f\n',version{1:3})
if length(version)>3
    fprintf('XDA build: %.0f%s\n',version{4:5})
end

%% Scan for devices
fprintf('Scanning for devices... \n')
ports = h.XsScanner_scanPorts(0,100,true,true);

% Find an MTi device
numPorts = size(ports,1);
if(numPorts >1)
    fprintf('Found %d devices... \n', numPorts)
end

if ~numPorts
    fprintf('No MTi device found. Aborting. \n');
    return
end

for port = 1:numPorts
    if (h.XsDeviceId_isMti(ports{port,1}) || h.XsDeviceId_isMtig(ports{port,1}))
        mtPort = ports(port,:);
        %break
    end
    %end
    
    
    
    deviceId(port) = mtPort{1};
    portName{port} = mtPort{3};
    baudrate(port) = mtPort{4};
    
    fprintf('Found a device with: \n');
    fprintf(' Device ID: %s \n', h.XsDeviceId_toString(deviceId(port)));
    fprintf(' Baudrate: %d \n', baudrate(port));
    fprintf(' Port name: %s \n', portName{port});
    
    %% Open port
    fprintf('Opening port... \n')
    if ~h.XsControl_openPort(portName{port}, baudrate(port), 0, true)
        fprintf('Could not open port. Aborting. \n');
        return
    end
    
    % Get the device object
    device(port) = h.XsControl_device(deviceId(port));
    fprintf('Device: %s, with ID: %s opened. \n', h.XsDevice_productCode(device(port)), h.XsDeviceId_toString(h.XsDevice_deviceId(device(port))));
    
    %% Register eventhandler
    h.registerevent({'onLiveDataAvailable',@eventhandlerXsens});
    h.setCallbackOption(h.XsComCallbackOptions_XSC_LivePacket, h.XsComCallbackOptions_XSC_None);
    % show events using h.events and h.eventlisteners too see which are registerd;
    
    %% Put device into configuration mode
    fprintf('Putting device into configuration mode... \n')
    if ~h.XsDevice_gotoConfig(device(port))
        fprintf('Could not put device into configuration mode. Aborting. \n');
        return
    end
    
    %% Configure the device
    fprintf('Configuring the device... \n')
    
%     if (h.XsDeviceId_isImu(deviceId(port)))
%         outputConfig = {h.XsDataIdentifier_XDI_PacketCounter,0;
%             h.XsDataIdentifier_XDI_SampleTimeFine,0;
%             h.XsDataIdentifier_XDI_DeltaV,0;
%             h.XsDataIdentifier_XDI_DeltaQ,0;
%             h.XsDataIdentifier_XDI_MagneticField,0};
%     elseif (h.XsDeviceId_isVru(deviceId(port)) || h.XsDeviceId_isAhrs(deviceId(port)))
%         outputConfig = {h.XsDataIdentifier_XDI_PacketCounter,0;
%             h.XsDataIdentifier_XDI_SampleTimeFine,0;
%             h.XsDataIdentifier_XDI_Quaternion,0};
%     elseif (h.XsDeviceId_isGnss(deviceId))
%         outputConfig = {h.XsDataIdentifier_XDI_PacketCounter,0;
%             h.XsDataIdentifier_XDI_SampleTimeFine,0;
%             h.XsDataIdentifier_XDI_Quaternion,0;
%             h.XsDataIdentifier_XDI_LatLon,0;
%             h.XsDataIdentifier_XDI_AltitudeEllipsoid,0;
%             h.XsDataIdentifier_XDI_VelocityXYZ,0};
%     else
%         fprintf('Unknown device while configuring. Aborting. \n');
%         return
%     end
    
    outputConfig = {
                 h.XsDataIdentifier_XDI_PacketCounter,0;
        %         h.XsDataIdentifier_XDI_SampleTimeFine,0;
                 h.XsDataIdentifier_XDI_DeltaV,0;
                 h.XsDataIdentifier_XDI_DeltaQ,0;
        %         h.XsDataIdentifier_XDI_MagneticField,0
        h.XsDataIdentifier_XDI_Acceleration,0;
        h.XsDataIdentifier_XDI_RateOfTurn,0;
        h.XsDataIdentifier_XDI_SampleTimeFine,0;
        h.XsDataIdentifier_XDI_Quaternion,0;
        h.XsDataIdentifier_XDI_MagneticField,0
        };
    
    
%     outputConfig = {
%         h.XsDataIdentifier_XDI_PacketCounter,0;
% h.XsDataIdentifier_XDI_SampleTimeFine,0;
% h.XsDataIdentifier_XDI_RateOfTurn,0;
% h.XsDataIdentifier_XDI_Acceleration,0;
% h.XsDataIdentifier_XDI_MagneticField,0};
    
    
    if ~h.XsDevice_setOutputConfiguration(device(port),outputConfig)
        fprintf('Could not configure the device. Aborting. \n');
        return
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Create a log file
    %
    % fprintf('Creating a log file... \n')
    % filename = [cd '\logfile.mtb'];
    % if h.XsDevice_createLogFile(device,filename) ~= 0 % XRV_OK
    %     fprintf('Failed to create a log file. Aborting. \n');
    %     return
    % end
    % fprintf('Created a log file: %s \n',filename);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    %% Put device into measurement mode
    fprintf('Putting device into measurement mode... \n')
    if ~h.XsDevice_gotoMeasurement(device(port))
        fprintf('Could not put device into measurement mode. Aborting. \n');
        return
    end
    
    %% Start recording
    fprintf('Starting recording... \n')
    if ~h.XsDevice_startRecording(device(port))
        fprintf('Failed to start recording. Aborting. \n');
        return
    end
    
end %%port num

counter = ones(1,numPorts);

%% Record data
fprintf('Recording data for 10 seconds. \n');
tic;
while toc < 20
    pause(.01);
end

for port = 1:numPorts
    %% Stop recording
    fprintf('\nStopping recording... \n');
    if ~h.XsDevice_stopRecording(device(port))
        fprintf('Failed to stop recording. Aborting. \n');
        return
    end
    
    % %% Close log file
    % fprintf('Closing log file... \n');
    % if ~h.XsDevice_closeLogFile(device(port))
    %     fprintf('Failed to close log file. Aborting. \n');
    %     return
    % end
    
    %% Close port and XsControl object
    fprintf('Closing port and XsControl object... \n');
    h.XsControl_closePort(portName{port});
 
end
   h.XsControl_close();
%% Release COM-object
    fprintf('Releasing COM-object... \n');
    delete(h);
    clear h;
    
    out.counter = counter;
    out.orientation = orientation;
    out.relativeOrientation = relative_orientation;
    out.timestamp = timestamp;
    out.imu1.acc = acc1;
    out.imu1.gyr = gyr1;
    out.imu1.t = t1;
    out.imu2.acc = acc2;
    out.imu2.gyr = gyr2;
    out.imu2.t = t2;
    
    fprintf('Successful exit. \n');

%% Event handler
    function eventhandlerXsens(varargin)
        % only action when new datapacket arrived
        dataPacket = varargin{3}{2};
        %%guardo quale imu è
        sensor_i = find(deviceId == h.XsDataPacket_deviceId(dataPacket));
        
        
        
                 if h.XsDataPacket_containsCalibratedData(dataPacket)
                     acc = cell2mat(h.XsDataPacket_calibratedAcceleration(dataPacket));
                    % fprintf('\rAcc X: %.2f, Acc Y: %.2f, Acc Z: %.2f', acc(1), acc(2), acc(3));
        
                     gyr = cell2mat(h.XsDataPacket_calibratedGyroscopeData(dataPacket));
        %             fprintf(' |Gyr X: %.2f, Gyr Y: %.2f, Gyr Z: %.2f', gyr(1), gyr(2), gyr(3));
        %
        %             mag = cell2mat(h.XsDataPacket_calibratedMagneticField(dataPacket));
        %             fprintf(' |Mag X: %.2f, Mag Y: %.2f, Mag Z: %.2f', mag(1), mag(2), mag(3));
                 end
                 
        if h.XsDataPacket_containsSampleTimeFine(dataPacket)
           t= h.XsDataPacket_sampleTimeFine(dataPacket);
        end
        
             if(sensor_i == 1)

                 acc1 = [acc1 acc];
                 gyr1 = [gyr1 gyr];
                 t1 = [t1 t];
            end
            if(sensor_i == 2)
                 acc2 = [acc2 acc];
                 gyr2 = [gyr2 gyr];
                 t2 = [t2 t];
             end
                 
        if h.XsDataPacket_containsOrientation(dataPacket)
            quaternion = cell2mat(h.XsDataPacket_orientationQuaternion(dataPacket,h.XsDataIdentifier_XDI_CoordSysEnu));
            %fprintf('\rq0: %.2f, q1: %.2f, q2: %.2f, q3: %.2f', quaternion(1), quaternion(2), quaternion(3), quaternion(4));
                      
            orientation(sensor_i, counter(sensor_i),:) = quaternion;
            
            if(sensor_i == 1)
                q1 = squeeze(orientation(1, counter(1),:))';
            end
            if(sensor_i == 2)
                q2 = squeeze(orientation(2, counter(2),:))';
            end
            %r= [ 0 0 0]';
            relative_orientation(counter_r,:) = quatmultiply(q1, quatinv(q2));
           % r = quat2euler( (quatmultiply(q1, quatinv(q2))) ).*180/pi
            counter(sensor_i) = counter(sensor_i) +1;

          %  euler = cell2mat(h.XsDataPacket_orientationEuler(dataPacket,h.XsDataIdentifier_XDI_CoordSysEnu));
          %  fprintf(' |Roll: %.2f, Pitch: %.2f, Yaw: %.2f', euler(1), euler(2), euler(3));

        end
        
            
         
        
        %         if h.XsDataPacket_containsLatitudeLongitude(dataPacket)
        %             latlon = cell2mat(h.XsDataPacket_latitudeLongitude(dataPacket));
        %             fprintf(' |Lat: %7.2f, Lon: %7.2f', latlon(1), latlon(2));
        %         end
        %
        %         if h.XsDataPacket_containsAltitude(dataPacket)
        %             alt = h.XsDataPacket_altitude(dataPacket);
        %             fprintf(' |Alt: %7.2f', alt);
        %         end
        %
        %         if h.XsDataPacket_containsVelocity(dataPacket)
        %             vel = cell2mat(h.XsDataPacket_velocity(dataPacket,h.XsDataIdentifier_XDI_CoordSysEnu));
        %             fprintf(' |E: %.2f, N: %.2f, U: %.2f', vel(1), vel(2), vel(3));
        %         end
        
        h.dataPacketHandled(varargin{3}{1}, dataPacket);
        counter_r = counter_r+1;

    end
end
