classdef controllers
    methods(Static)
        function Dz=PhaseLeadCompensator(Gz, T, ww1, Pm, a0)
            Gw = d2c(Gz, 'tustin');
            [magGww1,phaseGww1] = bode(Gw, ww1);
            thetad = 180+Pm-phaseGww1; 
            theta = deg2rad(thetad);
            if(phaseGww1 < 180 + Pm)
               error('Restrição 1 não atingida: ∠𝐺 𝑗𝜔𝑤1 < 180° + 𝜙�') 
            end
            
            if(magGww1 < 1/a0)
               error('Restrição 2 não atingida: |𝐺 𝑗𝜔𝑤1| < 1/𝑎0') 
            end
            
            if(cos(theta) > a0 * magGww1)
                error('Restrição 3 não atingida: |𝐺 𝑗𝜔𝑤1| < 1/𝑎0') 
            end
                   
            a1 = (1-a0*magGww1*cos(theta))/...
                (ww1*magGww1*sin(theta));
            b1 = (cos(theta)-a0*magGww1)/...
                (ww1*sin(theta));
            ww0 = a0/a1; wwp = 1/b1;
            s = tf('s');
            Dw = a0*(s/ww0+1)/(s/wwp+1);
            Dz = c2d(Dw, T, 'tustin');
            return
        end
        function Dz=PhaseLagCompensator(Gz, T, ww1, a0)
            [magGjww1,~]=bode(Gz,ww1);
            ww0 = 0.1*ww1; 
            wwp = ww0/(a0*magGjww1);
            Kd = a0*(wwp*(ww0+2/T))/(ww0*(wwp+2/T));
            z0 = (2/T-ww0)/(2/T+ww0); 
            zp = (2/T-wwp)/(2/T+wwp);
            Dz = Kd*tf([1 -z0],[1 -zp],T);
            return
        end
        function Dz=doPIController(Gz, T, ww1, Pm)
            [magGww1, phaseGww1] = bode(Gz,ww1);
            thetad = 180+Pm-phaseGww1; thetar = thetad*pi/180;
            KD = 0;
            KP = cos(thetar)/magGww1;
            KI = -ww1*sin(thetar)/magGww1;
            % ww0=KI/KP;
            Dz = KP+(KI*T/2)*tf([1 1],[1 -1],T)+(KD/T)*tf([1 -1],[1 0],T);
            return
        end
        function Dz=doPDController(Gz, T, ww1, Pm)
            [magGww1, phaseGww1] = bode(Gz,ww1);
            thetad = 180+Pm-phaseGww1; thetar = thetad*pi/180;
            KD = 0;
            % Equation (8-63)
            KP = cos(thetar)/magGww1;
            KI = -ww1*sin(thetar)/magGww1;
            % ww0=KI/KP;
            % Equation (8-52)
            Dz = KP+(KI*T/2)*tf([1 1],[1 -1],T)+(KD/T)*tf([1 -1],[1 0],T);
            return
        end
        function Dz=doPID(Gz, T, ww1, Pm)
            [magGww1, phaseGww1] = bode(Gz,ww1);
            thetad = 180+Pm-phaseGww1; thetar = thetad*pi/180;
            KI = 1;
            KD = (sin(thetar)/magGww1/ww1 + KI/ww1^2)*(1 + ww1^2/(2/T)^2);
            KP = cos(thetar)/magGww1 - KD*ww1^2/(2/T+ww1^2*T/2);
            Dz = KP + (KI*T/2)*tf([1 1],[1 -1],T) + (KD/T)*tf([1 -1],[1 0],T);
            return
        end
        function Dz=doDeadBeat(Gz, T)
            z = tf('z', T);
            Dz = 1 / (Gz*(z-1));
            return
        end
        function Dz=doDahlin(Gz, T, n)
            z = tf('z', T);
            tau=T*n;
            Dz = (1 / (Gz)) * (1-exp(-T/tau))/(z^(n+1)-exp(-T/tau)*z^n-(1-exp(-T/tau)));
            return
        end
    end
end