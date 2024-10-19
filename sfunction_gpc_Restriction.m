function gpc_sfunction_restrito(block)
    setup(block);
end

function setup(block)
    % Register number of ports
    block.NumInputPorts  = 2;
    block.NumOutputPorts = 1;

    % Setup port properties to be inherited or dynamic
    block.SetPreCompInpPortInfoToDynamic;
    block.SetPreCompOutPortInfoToDynamic;

    % Input port properties
    block.InputPort(1).Dimensions        = 1;
    block.InputPort(1).DatatypeID        = 0;  % double
    block.InputPort(1).Complexity        = 'Real';
    block.InputPort(1).DirectFeedthrough = true;
    
    block.InputPort(2).Dimensions        = 1;
    block.InputPort(2).DatatypeID        = 0;  % double
    block.InputPort(2).Complexity        = 'Real';
    block.InputPort(2).DirectFeedthrough = true;

    % Output port properties
    block.OutputPort(1).Dimensions       = 1;
    block.OutputPort(1).DatatypeID       = 0; % double
    block.OutputPort(1).Complexity       = 'Real';

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
% create dwork vectors

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
    % Using 'inicializacao_GPC to process data into project variables and
    % matrixes
    
    reference = block.Dwork(1).Data;
    [G, ~, F, Qd, Ql] = inicializacao_GPC(0,[0.0004423 0.0004423], [1 -0.9994],3,15,.005);
    
    block.OutputPort(1).Data = controle(G, F, Qd, Ql, reference,  block);
end

function u = controle(G, F, Qd, Ql, block)
    
    % Calculate free state
    free = F * block.Dwork(3).Data; %matrix 3x1
    
    %aplying restriction
    Triang=tril(ones(Nu));  
    T=ones(Nu,1);
    ub=10; %Maximum control sign
    u_max=ub*ones(Nu,1);  
    u_min=0;
    a=[Triang; -Triang];
    b=[u_max-T*block.Dwork(2).Data; T*block.Dwork(2).Data-u_min];
             
    H=(G'*Qd*G+Ql);
    Fo=(free-reference)'*Qd*G;
    %Calculo do Controle
    options = optimset('LargeScale','off');
    [x,~,~] = quadprog(H,Fo,a,b,[],[],[],[],[],options);
    inc_u=x(1);

    % Calculate new control action
    u = inc_u + block.Dwork(2).Data(1);
    
    % Refresh variables
    block.Dwork(2).Data = u;
    aux_u2 = block.Dwork(4).Data(1:2);
    block.Dwork(4).Data = [inc_u; aux_u2(1); aux_u2(2)];

end

function Update(block)
    % Refressh input values
    block.Dwork(3).Data = [block.InputPort(2).Data; block.Dwork(3).Data(1:end-1)];
end
