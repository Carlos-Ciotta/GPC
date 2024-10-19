function gpc_sfunction_srestricao(block)
    setup(block);
end

function setup(block)
    % Register number of ports
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 1;

    % Setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Override input port properties
    block.InputPort(1).Dimensions        = 1;
    block.InputPort(1).DatatypeID        = 0;  % double
    block.InputPort(1).Complexity        = 'Real';
    block.InputPort(1).DirectFeedthrough = true;
    
    block.InputPort(2).Dimensions        = 1;
    block.InputPort(2).DatatypeID        = 0;  % double
    block.InputPort(2).Complexity        = 'Real';
    block.InputPort(2).DirectFeedthrough = true;

    % Override output port properties
    block.OutputPort(1).Dimensions       = 1;
    block.OutputPort(1).DatatypeID       = 0; % double
    block.OutputPort(1).Complexity       = 'Real';

    % Register parameters
    block.NumDialogPrms     = 0;

    % Register sample times
    block.SampleTimes = [0.1 0];

    % Specify the block simStateCompliance
    block.SimStateCompliance = 'DefaultSimState';

    % Register methods
    block.RegBlockMethod('PostPropagationSetup', @DoPostPropagationSetup);
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Update',               @Update);
end

function DoPostPropagationSetup(block)
block.NumDworks = 4;

    block.Dwork(1).Name            = 'u'; 
    block.Dwork(1).Dimensions      = 1;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    block.Dwork(2).Name            = 'past_u'; 
    block.Dwork(2).Dimensions      = 1; % Adjust size as necessary
    block.Dwork(2).DatatypeID      = 0;
    block.Dwork(2).Complexity      = 'Real';
    block.Dwork(2).UsedAsDiscState = true;

    block.Dwork(3).Name            = 'past_y'; 
    block.Dwork(3).Dimensions      = 2; % Adjust size as necessary
    block.Dwork(3).DatatypeID      = 0;
    block.Dwork(3).Complexity      = 'Real';
    block.Dwork(3).UsedAsDiscState = true;

    block.Dwork(4).Name            = 'past_inc_u'; 
    block.Dwork(4).Dimensions      = 3; % Adjust size as necessary
    block.Dwork(4).DatatypeID      = 0;
    block.Dwork(4).Complexity      = 'Real';
    block.Dwork(4).UsedAsDiscState = true;
end

function InitializeConditions(block)
    % Initialize Dwork vectors
    block.Dwork(1).Data = 0;
    block.Dwork(2).Data = zeros(1, 1);
    block.Dwork(3).Data = zeros(2, 1);
    block.Dwork(4).Data = zeros(3, 1);

    
end

function Outputs(block)
    % Calls GPC init to process variables
    [G, K1, F] = inicializacao_GPC(0,[0.0004423 0.0004423], [1 -0.9994],3,15,.005);
    
    block.OutputPort(1).Data = controle(K1, G, F, block);
end

function u = controle(K1, G, F, block)
    % Calculate free state

    free = F * block.Dwork(3).Data; %matrix 3x1
    y_pred = free + G * block.Dwork(4).Data;
    
    % Calculate control increment
    
    inc_u = K1 * (block.InputPort(1).Data - y_pred);

    % Calculate control action
    u = inc_u + block.Dwork(2).Data(1);
    
    % Refresh variables
    block.Dwork(2).Data = u;
    aux_u2 = block.Dwork(4).Data(1:2);
    block.Dwork(4).Data = [inc_u; aux_u2(1); aux_u2(2)];
end

function Update(block)
    % Refresh input
    block.Dwork(3).Data = [block.InputPort(2).Data; block.Dwork(3).Data(1:end-1)];
end
