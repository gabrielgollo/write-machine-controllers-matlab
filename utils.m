
classdef utils
    methods(Static)
        function plotSystemPerfomances(Gz, Dz, T, figNum, strTitle)
            % plant with controller
            DzGz = series(Dz,Gz);
            closeLoopSys = feedback(DzGz, 1, -1);
            closeLoopPlant = feedback(Gz, 1, -1);
            
            % Esforço de controle e valores min/max
                MF_U = feedback(Dz,Gz);
                t_f = 10; %tempo final
                [y,t] = step(MF_U,t_f); %função ao degrau da malha fechada R ===> U
                u_min = min(y); %valor mínimo da ação de controle 
                u_max = max(y); %valor máximo da ação de controle 
                u_inf = sum((y-mean(y)).^2)*T;%cálculo de esforço de controle
                
            currIndex = figNum;
            % 1- Step Response with zero pertubation
                fprintf("Características da Resposta ao Degrau do Sistema %s\n", strTitle);
                SP=1; %input value, if you put 1 then is the same as step(sys)
                %get the response of the system to a step with amplitude SP
                    figure(currIndex);
                    [y,t] = step(SP*closeLoopSys);
                    step(closeLoopSys, closeLoopPlant);  grid on; hold on;
                    legend('DzGz', 'Plant');
                str1=strcat('Step Response with Zero perturbation -- ', strTitle);
                title(str1);
                
                stepError=abs(SP - y(end))*100; %get the steady state error of step

            % 2- Ramp Response with zero pertubation
                currIndex = currIndex + 1; figure(currIndex);
                input = t;
                y_ramp = lsim(closeLoopSys, input, t);
                plot(t,t); hold on;
                plot(t, y_ramp); hold on; grid on;
                
                legend('Entrada em rampa', 'Resposta a rampa')
                
                str2=strcat('Ramp Response -- ', strTitle);
                title(str2);
                
                
                rampError = abs(input(end) - y_ramp(end))*100; %get the steady state error of ramp

            % 3- Step Response With nullable reference
                currIndex = currIndex + 1; figure(currIndex);
                %newSys = feedback(1, sys);
                newSys = Gz/(1+Gz*Dz);
                STEP_pertubation = stepinfo(newSys)
                [y,t] = step(newSys);
                pertubationError = abs(0 - y(end))*100;
                
                step(newSys); grid on; hold on;
                
                str3=strcat('Step Response for pertubation with Zero reference -- ', strTitle);
                title(str3);
                
            % 4- Bode Plot for the System
                currIndex = currIndex + 1; figure(currIndex);
                bode(Gz, DzGz); grid on;
                legend('Gz', 'DzGz')
                str4=strcat('Bode diagram -- ', strTitle);
                title(str4);

            % 5- Bode for close loop system
                closeLoopUnitGain = feedback(Gz, 1, -1);
                
                currIndex = currIndex + 1; figure(currIndex);
                bode(closeLoopSys, closeLoopUnitGain, zpk(Dz)); grid on;
                legend('DzGz close loop', 'Gz close loop', 'Dz')
                
                str5=strcat('Bode diagram for close loop -- ', strTitle);
                title(str5);
            
            % 7- Nichols Chart of the System
                currIndex = currIndex + 1; figure(currIndex);
                nichols(closeLoopSys); grid on;
                title(strcat('Nichols Chart -- ', strTitle));
            
            % 8- pzMap
                currIndex = currIndex + 1; figure(currIndex);
                pzmap(zpk(Dz)); grid on;
                title(strcat('Pole-Zero Map -- ', strTitle));
                
            % 9- controller
                currIndex = currIndex + 1; figure(currIndex);
                [Gm,Pm] = margin(DzGz);
                % Print the Features of the system
                STEP = stepinfo(closeLoopSys);
                utils.PRINT_SYSTEM_FEATURES(STEP, stepError, rampError, pertubationError,...
                    u_min, u_max, u_inf, Gm, Pm,strTitle, STEP_pertubation, bandwidth(closeLoopSys, -3));
                margin(DzGz);
                %title(strcat('Close Loop Bode Diagram -- ', strTitle));
                
            %10 - bench controller
                %currIndex = currIndex + 1; figure(currIndex);
                %margin(zpk(Dz));
                %title(strcat('Only Controller Bode Diagram -- ', strTitle));
        end
        
        function PRINT_SYSTEM_FEATURES(STEP_INFO, stepError, rampError, pertubationError,...
                u_min, u_max, u_inf, Gm, Pm, strTitle, STEP_pertubation, bw)
            
            fprintf("---- %s ----\n", strTitle);
            fprintf("Rise Time: %f s\n", STEP_INFO.RiseTime);
            % fprintf("Transient Time: %s\n", STEP_INFO.TransientTime);
            fprintf("Settling Time: %f s\n", STEP_INFO.SettlingTime);
            fprintf("Min Settling Time: %f s\n", STEP_INFO.SettlingMin);
            fprintf("Max Settling Time: %f s\n", STEP_INFO.SettlingMax);
            fprintf("Overshoot: %f%%\n", STEP_INFO.Overshoot);
            fprintf("Undershoot: %f%%\n", STEP_INFO.Undershoot);
            fprintf("System Peak: %f \n", STEP_INFO.Peak);
            fprintf("Sys Peak Time: %f s\n", STEP_INFO.PeakTime);
            fprintf("Steady state error of STEP: %f%%\n", stepError);
            fprintf("Steady state error of RAMP: %f%%\n ", rampError);
            fprintf("Ação de controle(saturação) min<U<max: %f<U<%f\n", u_min, u_max);
            fprintf("Esforço de controle para resposta ao degrau: %f\n", u_inf);
            fprintf("\n\n\n");
            
            
            % TODO: Implement a way to convert all data to csv or excel
            % Write all data to a txt file
            fileID = fopen('results.txt','a+');
                fprintf(fileID, "---- %s ----\n", strTitle);
                fprintf(fileID,"%f%%\n", stepError); %Steady state error of STEP
                fprintf(fileID,"%f%%\n ", rampError); %Steady state error of RAMP
                fprintf(fileID, "%f s\n", STEP_INFO.RiseTime); %Rise Time
                fprintf(fileID,"%f s\n", STEP_INFO.SettlingTime); %SettlingTime: 
                fprintf(fileID,"%f %%\n", STEP_INFO.Overshoot); %Overshoot: 
                fprintf(fileID,"%f dB\n", 20*log10(Gm));
                fprintf(fileID,"%f º\n", Pm);
                fprintf(fileID, "%f rad/s \n", bw); % banda de passagem MF em -3db
                fprintf(fileID, "%f\n", pertubationError); % Steady state error for pertubation
                fprintf(fileID, "%f s\n", STEP_pertubation.SettlingTime); %SettlingTime for pertubation
                fprintf(fileID,"%f ; %f\n", u_min, u_max);
                fprintf(fileID,"%f\n", u_inf);
                fprintf(fileID,"%f dB\n", STEP_INFO.Peak);
                
                fprintf(fileID,"\n\n\n");
            fclose(fileID);
            
        end
    end
end
