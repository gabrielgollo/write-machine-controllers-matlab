%%  Inicio
    % Clear window and workspace;
    clc; clear;
    % Organize graph windows
    close all;
    set(0,'DefaultFigureWindowStyle','docked');
    
    % Systems
    s = tf('s');
    
 % Planta (numerador/denominador)
    num = 0.774;
    den = [1 0.51 0.774];
    
    % Configuracoes
        delay=0.13*10;
        G = tf(num, den); % Planta do sistema
        H=1; % Feedback
        Du = 1; % Controlador unitário
        
    % Calculo da frequencia de amostragem do sistema
        omega_n = sqrt(0.774);
        zeta = 0.29;
        Ts = 1 / (10 * zeta * omega_n); 
        z = tf('z');
        
    % Calculo do a0
        %G(z) = (0.05514 * z + 0.05157) / ((z)^2 - 1.712 * z + 0.8188 );
        Ky = 0.999157303; % Ky = G(1)
        
        % Erro em estado estacionario
        choosen_steady_state_error = 0.1;
        a0 = (1-choosen_steady_state_error)/(Ky*choosen_steady_state_error);
           
    % Planta do sistema discretizada e sem atraso
        Gz = c2d(G, Ts, 'zoh');
        
    % Planta do sistema em malha fechada
        G_MF = feedback(series(G, Du), H, -1);
        %step(G_MF);
        G_MF_Z = feedback(series(Gz, Du), H, -1);
    
    % Avaliação da planta com controlador unitário
        %SP=1;
        %[y, t]= step(SP*G_MF_Z); grid on;
        %stepError=abs(SP - y(end))*100; %get the steady state error of step
        %utils.PRINT_SYSTEM_FEATURES(stepinfo(G_MF_Z), stepError, 1);
        
        utils.plotSystemPerfomances(G_MF_Z, 2, Ts, 1, 'Plant')
        
    %% 4- Root Locus
    close all;
    % Esse foi o primeiro controlador que ajustei no Root Locus
    % Para tentar estabilizar o sistema do jeito que eu queria.
        % firstControllerToTestRootLocus = 8.2238*(z^2-1.636*z+0.6849)/((z-1)*(z+0.5142));
        % utils.plotSystemPerfomances(Gz, firstControllerToTestRootLocus, Ts, 18, 'RootLocus')
        
    % C1 - Controlador Proporcional
        gain = 0.19614;
        utils.plotSystemPerfomances(Gz, gain, Ts, 13, 'C1 - Proporcional');
        
    %% C2 - Compensador de atraso
    close all;
        rootlocus_leadlag_compensator = 0.3508*(z-0.829)/(z-0.9363);
        utils.plotSystemPerfomances(Gz, rootlocus_leadlag_compensator, Ts, 13, 'C2 - Phase Lag RL Compensator');
        
    %% C3 - Compensador de atraso com integrador
    close all;
        compensador =  0.013587*(z+0.02448)/(z-0.7832);
        integrador = 1/(z-1);
        
        CompensadorTop = series(compensador, integrador);

        utils.plotSystemPerfomances(Gz, CompensadorTop, Ts, 13, 'C3 - Phase Lag RL  with Integrator');
    
    %% C4 - Phase Lead Compensator - extras
    close all;
        %Dz=controllers.PhaseLeadCompensator(Gz, Ts, 1.44, 40, a0);
        %utils.plotSystemPerfomances(Gz, Dz, Ts, 3, 'Phase Lead Compensator')
        %margin(Dz)
        
    % C4 - Phase Lag Compensator - Usado
        Dz1=controllers.PhaseLagCompensator(Gz, Ts, 1.10, a0);
        utils.plotSystemPerfomances(Gz, Dz1, Ts, 13, 'C4 - Phase Lag Compensator')
    
    %% C5 - Controlador PI
    close all;

        PI_controller = controllers.doPIController(Gz, Ts, 1.19, 23);
        utils.plotSystemPerfomances(Gz, PI_controller, Ts, 23, 'C5 - Controlador PI')
    
    %% C extra - Controlador PD
    %close all;

     %   PD_controller = controllers.doPIController(Gz, Ts, 1.40, 2);
      %  utils.plotSystemPerfomances(Gz, PD_controller, Ts, 23, 'Controlador PD')
    %% C6 - Controlador PID
    close all;
        
        PID_controller = controllers.doPID(Gz, Ts, 1.2, 72);
        utils.plotSystemPerfomances(Gz, PID_controller, Ts, 23, 'C6 - Controlador PID')
      
    %% C7 - Controlador PID - sintonia automatica
    close all;
        %sisotool(Gz);
        PID_auto =( 4.6863*(z^2 - 1.402*z + 0.4936) ) / ((z-1)*(z-0.09976));
        utils.plotSystemPerfomances(Gz, PID_auto, Ts, 33, 'C7 - Controlador PID Sintonia Auto')
    
    %% Sintese direta 1
    % Dead Beat
        DEAD_BEAT_CONTROLLER = controllers.doDeadBeat(Gz, Ts);
        utils.plotSystemPerfomances(Gz, DEAD_BEAT_CONTROLLER, Ts, 33, 'C8 - Controlador Dead Beat')
    
    
    %% Sintese direta 2
    % Metodo Dahlin
    close all;
        DAHLIN_CONTROLLER = controllers.doDahlin(Gz, Ts, 2);
        utils.plotSystemPerfomances(Gz, DAHLIN_CONTROLLER, Ts, 43, 'C9 - Controlador Dahlin')
    
    
    
    
    
    
    
    