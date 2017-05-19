function block_AD_SingleValue(block)
    % Parameters:
    % 1: DeviceId
    % 2: ChannelId
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
    % values{2} = Channel
    % values{3} = Oversampling
    % values{4} = Gain
    % values{5} = Polarity

    block.NumInputPorts  = 0;
    block.NumOutputPorts = 1;
    
    % Setup port properties to be inherited or dynamic
    
    for i = 1:block.NumOutputPorts
        block.OutputPort(i).DatatypeID  = 0; % double
        block.OutputPort(i).Complexity  = 'Real';
        block.OutputPort(i).SamplingMode = 'Sample';
        block.OutputPort(i).Dimensions = 1;
    end
    
    block.SampleTimes = [0 1];
    
    block.NumDialogPrms = 5;
    block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable','Nontunable','Nontunable'};
    
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
    prmChannel = block.DialogPrm(2).Data;
    prmOversampling = block.DialogPrm(3).Data;
    prmGain = block.DialogPrm(4).Data;
    prmPolarity = block.DialogPrm(5).Data;    
    
    if (prmDevice < 0) || (prmDevice > 16)
        error('invalid device id, it should be in range [0..15]');
    end
    
    objDevice = Goldammer.MAI.GetDeviceByIndex(prmDevice); 
    if isempty(objDevice)
        if (prmChannel < 0) || (prmChannel > 32)
            error('invalid channel id, it should be in range [0..31]');
        end
    else
        if (prmChannel < 0) || (prmChannel >= objDevice.NumberOfADChannels)  
            error('invalid channel id, it should be in range [0..%d]',(objDevice.NumberOfADChannels-1));
        end
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
    objDevice = Goldammer.MAI.GetDeviceByIndex(prmDevice); 
    if isempty(objDevice)
        error('invalid device id');
    end    
%end DAQ_Start

function DAQ_SimStatusChange(block)
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
    prmChannel = block.DialogPrm(2).Data;
    prmOversampling = block.DialogPrm(3).Data;
    prmGain = block.DialogPrm(4).Data;
    prmPolarity = block.DialogPrm(5).Data;
    
    objDevice = Goldammer.MAI.GetDeviceByIndex(prmDevice); 
    if ~isempty(objDevice)
        objChannel  = objDevice.GetADChannel(prmChannel);
        if ~isempty(objChannel)
            value  = objChannel.ReadSingleVoltage(Goldammer.OversamplingMode.Disabled, Goldammer.GainFactor.Disabled, true);
            block.OutputPort(1).Data = transpose(value);
        end
    end
%end DAQ_Outputs

function DAQ_Terminate(block)
%end DAQ_Terminate

function DAQ_WriteRTW(block)
%end DAQ_WriteRTW

function DAQ_Update(block)
%end DAQ_WriteRTW

