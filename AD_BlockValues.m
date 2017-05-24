function block_AD_BlockValues(block)
    mpath = mfilename('fullpath');
    mdir = fileparts(mpath);
    iodll = '\maiv2dll\MAI_v2_NET20_DLL.dll';
    asm = NET.addAssembly( [mdir iodll] );

    setup(block);
    tic;
%endfunction

function setup(block)
    
    % Register original number of ports based on settings in Mask Dialog
    values = get_param(block.BlockHandle,'MaskValues');
    
    % see mask properties
    % values{1} = Device
    % values{2} = Channellist
    % values{3} = Samplerate
    % values{4} = Blocksize

    block.NumInputPorts  = 4;
    block.SetPreCompInpPortInfoToDynamic;
    
    block.NumOutputPorts = 16;
    block.SetPreCompOutPortInfoToDynamic;
    
    % Setup port properties to be inherited or dynamic
    for i = 1:block.NumInputPorts
        block.InputPort(i).DatatypeID = 0;
        block.InputPort(i).Complexity  = 'Real';
        block.InputPort(i).SamplingMode = 'Sample';
        block.InputPort(i).Dimensions = 1;
    end
    
    prmBlocksize = block.DialogPrm(4).Data;
    for i = 1:block.NumOutputPorts
        block.OutputPort(i).DatatypeID  = 0; % double
        block.OutputPort(i).Complexity  = 'Real';
        block.OutputPort(i).SamplingMode = 'Sample';
        block.OutputPort(i).Dimensions = [prmBlocksize 1]; 
    end
    
    block.SampleTimes = [0 1];
    
    block.NumDialogPrms = 4;
    block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable','Nontunable'};
    
    % Specify if Accelerator should use TLC or call back into 
    % MATLAB file
    block.SetAccelRunOnTLC(false);
    block.SetSimViewingDevice(false);% no TLC required

    % Allow multi dimensional signal support. 
    block.AllowSignalsWithMoreThan2D = false;
  
    % Specify the block simStateCompliance. The allowed values are:
    %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
    %    'DefaultSimState', < Same SimState as a built-in block
    %    'HasNoSimState',   < No SimState
    %    'CustomSimState',  < Has GetSimState and SetSimState methods
    %    'DisallowSimState' < Errors out when saving or restoring the SimState
    block.SimStateCompliance = 'DefaultSimState';    
    
    % -----------------------------------------------------------------
    % The MATLAB S-function uses an internal registry for all
    % block methods. You should register all relevant methods
    % (optional and required) as illustrated below. You may choose
    % any suitable name for the methods and implement these methods
    % as local functions within the same file.
    % -----------------------------------------------------------------
   
    % -----------------------------------------------------------------
    % Register methods called at run-time
    % -----------------------------------------------------------------
    block.RegBlockMethod('CheckParameters',         @DAQ_CheckParameters);
    block.RegBlockMethod('ProcessParameters',       @DAQ_ProcessParameters);
    block.RegBlockMethod('Start',                   @DAQ_Start);
    block.RegBlockMethod('SimStatusChange',         @DAQ_SimStatusChange);
    block.RegBlockMethod('GetSimState',             @DAQ_GetSimState);
    block.RegBlockMethod('SetSimState',             @DAQ_SetSimState);
    %block.RegBlockMethod('SetInputPortSampleTime',  @DAQ_SetInputPortSampleTime);
    %block.RegBlockMethod('SetOutputPortSampleTime', @DAQ_SetOutputPortSampleTime);
    block.RegBlockMethod('SetInputPortSamplingMode',@DAQ_SetInputPortSamplingMode);
    block.RegBlockMethod('Outputs',                 @DAQ_Outputs);
    block.RegBlockMethod('Terminate',               @DAQ_Terminate);
    %block.RegBlockMethod('WriteRTW',                @DAQ_WriteRTW);
    block.RegBlockMethod('PostPropagationSetup',    @DAQ_DoPostPropSetup);
    %block.RegBlockMethod('Update',                  @DAQ_Update); 

%endfunction

function DAQ_CheckParameters(block)
    prmDevice = block.DialogPrm(1).Data;
    prmChannellist = block.DialogPrm(2).Data;
    prmSamplerate = block.DialogPrm(3).Data;
    prmBlocksize = block.DialogPrm(4).Data;
    
    if (prmDevice < 0) || (prmDevice > 16)
        error('invalid device id, it should be in range [0..15]');
    end
    
    objDevice = Goldammer.MAI.GetDeviceByIndex(prmDevice); 
    if isempty(objDevice)
        %if (prmChannel < 0) || (prmChannel > 32)
        %    error('invalid channel id, it should be in range [0..31]');
        %end
    else
        %if (prmChannel < 0) || (prmChannel >= objDevice.NumberOfADChannels)  
        %    error('invalid channel id, it should be in range [0..%d]',(objDevice.NumberOfADChannels-1));
        %end
    end
%end DAQ_CheckParameters

function DAQ_ProcessParameters(block)
    block.AutoUpdateRuntimePrms;
%end DAQ_ProcessParameters

function DAQ_DoPostPropSetup(block)
    %% Setup Dwork
    block.NumDworks = 1;
    block.Dwork(1).Name = 'x0'; 
    block.Dwork(1).Dimensions      = 1;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
%end DAQ_DoPostPropSetup

import Goldammer.* 

function DAQ_Start(block)
    prmDevice = block.DialogPrm(1).Data;
    prmChannellist = block.DialogPrm(2).Data;
    prmSamplerate = block.DialogPrm(3).Data;
    prmBlocksize = block.DialogPrm(4).Data;
    objDevice = Goldammer.MAI.GetDeviceByIndex(prmDevice); 
    if isempty(objDevice)
        error('invalid device id');
    end 
    objDevice.StopMeasure();
 	objDevice.ClearAllChannelLists();    
    
    % Setup port properties to be inherited or dynamic   
    for i = 1:numel(prmChannellist)
        idx = prmChannellist(i);
        channel = objDevice.GetADChannel( idx );
        if ~isempty(channel)
            channel.CreateMeasurementChannel( Goldammer.OversamplingMode.Disabled, Goldammer.GainFactor.Disabled, true);    
        end
    end
    
    objDevice.ADChannels.SetSampleRate(prmSamplerate);
    objDevice.ConfigMeasure();    
    objDevice.StartMeasure();

%end DAQ_Start

function DAQ_SimStatusChange(block)
  if s == 0
    disp('Pause in simulation.');
  elseif s == 1
    disp('Resume simulation.');
  end
%end DAQ_SimStatusChang

function DAQ_GetSimState(block)
%end DAQ_GetSimState

function DAQ_SetSimState(block)
%end DAQ_SetSimState


function DAQ_SetInputPortSamplingMode(block, idx, mode)
    block.InputPort(idx).SamplingMode = mode;
    for i = 1:block.NumOutputPorts
        block.OutputPort(i).SamplingMode = 'Sample';
    end
%end DAQ_SetInputPortSamplingMode

function DAQ_SetInputPortSampleTime(block, idx, time)
    %fprintf('in %d: %d,%d\n',idx,time(1),time(2));
    block.InputPort(idx).SampleTime = time;
%end DAQ_SetInputPortSampleTim

function DAQ_SetOutputPortSampleTime(block, idx, time)
    %fprintf('out %d: %d,%d\n',idx,time(1),time(2));
    block.OutputPort(idx).SampleTime = time;
%end DAQ_SetOutputPortSampleTime

function DAQ_Outputs(block)
    prmDevice = block.DialogPrm(1).Data;
    prmChannellist = block.DialogPrm(2).Data;
    prmSamplerate = block.DialogPrm(3).Data;
    prmBlocksize = block.DialogPrm(4).Data;
    
    objDevice = Goldammer.MAI.GetDeviceByIndex(prmDevice); 
    if ~isempty(objDevice)
        %objChannel  = objDevice.GetADChannel(prmChannel);
        %if ~isempty(objChannel)
        %    value  = objChannel.ReadSingleVoltage(Goldammer.OversamplingMode.Disabled, Goldammer.GainFactor.Disabled, true);
        %    block.OutputPort(1).Data = transpose(value);
        %end
        NofVal = objDevice.ADChannels.GetNumberOfValues();
        if NofVal >= prmBlocksize
            d1 = NET.createArray('System.Double',prmBlocksize);
            T0 = int64(34);
            dT = 3;
            for c = 1:objDevice.ADChannels.MeasurementChannelsCount
                nov = uint32(prmBlocksize);
                [u1,d2,nov,T0,dT] = objDevice.ADChannels.GetMeasurementChannel(c-1).ReadData([],d1,nov,false);
         
                block.OutputPort(c).Data = transpose( double(d2));
            end        
        end
    end
%end DAQ_Outputs

function DAQ_Terminate(block)
%end DAQ_Terminate

function DAQ_WriteRTW(block)
%end DAQ_WriteRTW

function DAQ_Update(block)
%end DAQ_WriteRTW

