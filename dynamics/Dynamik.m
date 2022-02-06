
close all;
clear all;

addpath('../inverseKinematic/')
addpath('../directKinematic/')


% Simulation des Roboters ausfuehren
simulation = true;
% Integrationsverfahren
numIntVerfahren='Adams Bashforth'; % 'Euler': Euler vorwaerts
% Reglerverfahren
regVerfahren = "PD";    % P oder PD Regler
Kp = 40; % für Euler 30, für Adams Bashforth 30
Kd = 0.001;

% Lade Sollbahn im Arbeitsraum in Workspace
load( 'Gelenkwinkelbahn.mat' );

% Erzeuge Roboterstruktur
rob = erstelle_roboter();
 
% Datenmatrix fuer Viewer
V = zeros(3,4,6,length(T));

%% 2. Inverse Dynamik

% Variable zur Speicherung der berechneten Gelenkmomente - Inverse Dynamik
Tau_id = zeros( size(Q) );

% Berechne Gelenkmomentenverlauf fuer gegebene Bahn
disp('Inverse Dynamik')
for i=1:length(T)
    % Setze Zeit und Sollvorgabe für Gelenkwinkel 
    rob.zeit = T(i);
    
    rob.q = Q(:,i);
    rob.dot_q = dot_Q(:,i);
    rob.ddot_q = ddot_Q(:,i);

    % Berechne Antriebsmomente mit aktueller Soll-Roboterkonfiguration
    rob = berechne_id(rob);
    
    %Gelenkmomente zur spaeteren Analyse speichern
    Tau_id(:,i) = rob.tau_id;
    
    % Speichere Vektoren B0_r_i und Transformationsmatrizen A_i0 fuer Visualisierung im Viewer
    for l = 1:6
        V( :, 1, l, i ) = rob.kl(l).B0_r_i;
        V( :, 2:4, l, i ) = rob.kl(l).A_i0;
    end
end

% Speichere die Gelenkwinkel fuer den Viewer
write_data(T,V,6,'trajectory_Dynamik_Soll.csv');

%Drehmomentenverlauf plotten
figure();
plot( T, Tau_id(1,:), ...
      T, Tau_id(2,:), ...
      T, Tau_id(3,:), ...
      T, Tau_id(4,:), ...
      T, Tau_id(5,:), ...
      T, Tau_id(6,:) );
  
h=legend( '$\tau_1$','$\tau_2$','$\tau_3$','$\tau_4$','$\tau_5$','$\tau_6$');
h.Interpreter='latex';
xlabel( 't in [s]','Interpreter','latex');
ylabel( '$\tau$ in [Nm]','Interpreter','latex');
title('\bf{Drehmomentenverlauf}','Interpreter','latex')

%Winkelgeschwindigkeit plotten
figure();
plot( T, dot_Q(1,:), ...
      T, dot_Q(2,:), ...
      T, dot_Q(3,:), ...
      T, dot_Q(4,:), ...
      T, dot_Q(5,:), ...
      T, dot_Q(6,:) );

h=legend( '$\dot q_1$','$\dot q_2$','$\dot q_3$','$\dot q_4$','$\dot q_5$','$\dot q_6$' );
h.Interpreter='latex';
xlabel( 't in [s]','Interpreter','latex');
ylabel( '$\dot{q}$ in [rad/s]','Interpreter','latex');
title('\bf{Gelenkwinkelgeschwindigkeiten}','Interpreter','latex')

%% 3. Simulation - Direkte Dynamik

if simulation
    %Setze Roboter auf Anfangsposition
    rob.q = Q(:,1);
    rob.dot_q = dot_Q(:,1);

    % Variablen zur Speicherung der berechneten Bahn - Vortwaertsdynamik
    Q_vd = zeros( size(Q) );
    dot_Q_vd = zeros( size(Q) );
    ddot_Q_vd = zeros( size(Q) );

    % Variable zur Speicherung der Lagerkräfte/-momente
    Q_Lager = zeros([6 size(Q)]);
    % Datenmatrix fuer Viewer - Istbahn
    V_vd = zeros(3,4,rob.N_Q,length(T));

    %Berechne Bahn aus Drehmomenten der inversen Dynamik
    disp('Direkte Dynamik')
    for j=1:length(T)
        
        %Setze die aktuelle Zeit
        rob.zeit = T(j);

        % Regelung
        tau_reg = berechne_regler(rob, Q(:,j), dot_Q(:,j), Kp, Kd, regVerfahren);
        % Setze Antriebsmoment aus Inverser Dynamik
        rob.tau_antrieb = Tau_id(:,j) + tau_reg;

        % Speichere simulierte Gelenkwinkel fuer Zeitschritt j
        Q_vd(:,j) = rob.q;
        dot_Q_vd(:,j) = rob.dot_q;
        
        % Simulation des Roboters
        % (berechne Zustand zur naechsten Zeit j+1)
        %--------------------
         if j>=3
            dot_q_1 = dot_Q_vd(:,j-1);
            dot_q_2 = dot_Q_vd(:,j-2);
            ddot_q_1 = ddot_Q_vd(:,j-1);
            ddot_q_2 = ddot_Q_vd(:,j-2);
       else
            dot_q_1 = zeros(rob.N_Q,1);
            dot_q_2 = zeros(rob.N_Q,1);
            ddot_q_1 = zeros(rob.N_Q,1);
            ddot_q_2 = zeros(rob.N_Q,1);
        end

        rob = berechne_simulationszeitschritt(rob, numIntVerfahren, dot_q_1, dot_q_2, ddot_q_1, ddot_q_2, j);
        rob = berechne_lagerkraefte(rob);
       
        % Speichere Beschleunigung fuer Zeitschritt j
        ddot_Q_vd(:,j) = rob.ddot_q;

        % Speicher Lagerkräfte
        for i = 1:rob.N_Q
            Q_Lager(i,:,j) = [rob.kl(i).Bi_F_iv; rob.kl(i).Bi_M_iv];
        end

        % Visualisierung: Vektoren B0_r_i und Transformationsmatrizen A_i0 fuer Viewer speichern
        for l = 1:rob.N_Q
            V_vd(:,1,l,j) = rob.kl(l).B0_r_i;
            V_vd(:,2:4,l,j) = rob.kl(l).A_i0;
        end

    end

  % Speichere die Gelenkwinkel fuer den Viewer
  write_data(T,V_vd,6,'trajectory_Dynamik_Ist.csv');

  %Differenzen der Winkel zur Ueberpruefung plotten
  e_Q = Q - Q_vd;
  figure();
  plot( T, e_Q(1,:), ...
        T, e_Q(2,:), ...
        T, e_Q(3,:), ...
        T, e_Q(4,:), ...
        T, e_Q(5,:), ...
        T, e_Q(6,:) );

  h=legend( '$e_{q_1}$','$e_{q_2}$','$e_{q_3}$','$e_{q_4}$','$e_{q_5}$','$e_{q_6}$','Location','northwest');
  h.Interpreter='latex';
  title('\bf{Differenzen der Gelenkwinkel}','Interpreter','latex')
  xlabel( 't in [s]','Interpreter','latex');
  ylabel( '$e_q$ in [rad]','Interpreter','latex');
  
  
  %Gelenkwinkel Soll und Ist plotten
  figure();
  plot( T, Q(1,:), '--',...
        T, Q(2,:), '--',...
        T, Q(3,:), '--',...
        T, Q(4,:), '--',...
        T, Q(5,:), '--',...
        T, Q(6,:), '--');
  hold on
  % restart color order
  ax = gca;
  ax.ColorOrderIndex = 1;
  plot(   T, Q_vd(1,:), ...
          T, Q_vd(2,:), ...
          T, Q_vd(3,:), ...
          T, Q_vd(4,:), ...
          T, Q_vd(5,:), ...
          T, Q_vd(6,:) );
  hold off
  h=legend( '${q_1}$','${q_2}$','${q_3}$','${q_4}$','${q_5}$','${q_6}$','${q_{1,vd}}$','${q_{2,vd}}$','${q_{3,vd}}$','${q_{4,vd}}$','${q_{5,vd}}$','${q_{6,vd}}$','Location','northwest');
  h.Interpreter='latex';
  title('\bf{Gelenkwinkel Soll und Ist}','Interpreter','latex')
  xlabel( 't in [s]','Interpreter','latex');
  ylabel( '$q$ in [rad]','Interpreter','latex');

end
