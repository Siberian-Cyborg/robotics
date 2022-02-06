clear all;
close all;
clc;

 addpath(...
    '..\inverseKinematic\', ...
    '..\directKinematic\');

%% 2. Auswahl des Planungsverfahrens
% -------------------------------------------------------------------------
flag_traj = 'kubischeSpline' ; % Trajektorienplanungsverfahren
                        % 'p2p_kub': p2p kubische Polynome
                        % 'p2p_quint': p2p quintische Polynome
                        % 'kubischeSpline': kubische Splines
                        % 'paraBlends': parabolic blends
                        

%% 3. Trajektorienparameter initialisieren
% -------------------------------------------------------------------------
%
% Skizze:
%     ----------    -----------    0.3
%         |    |    |    |    |
%         |    |    |    |    |
%         |    |    |    |    |
%         |    ------    |    |    0.2
%
%                   ^ x
%               y   |
%              <----
%
% Gegebende Stuetzpunkte:
W_stuetz = [0.20, 0.20, -0.2;
            0.30, 0.20, -0.2;
            0.30, 0.30, -0.2;
            0.30, 0.10, -0.2;
            0.20, 0.10, -0.2;
            0.20, 0.00, -0.2;
            0.30, 0.00, -0.2;
            0.30,-0.10, -0.2;
            0.20,-0.10, -0.2;
            0.30,-0.10, -0.2;
            0.30,-0.20, -0.2;
            0.20,-0.20, -0.2 ]';
% Gesamtdauer der Trajektorie in [s]:
T_ges   = 6.05;      % Gilt bei Methoden mit Zeitvorgabe.
delta_T = 0.01;
% Roboter:
N_Q = 6;
% Plots an:
Flag = 1;

%% 4. Berechnung der Bewegungen
% -------------------------------------------------------------------------
    % Polynome dritter Ordnung
    if strcmp(flag_traj,'p2p_kub') == true
        [ W, dot_W, ddot_W, T ] = p2p_kubisch( W_stuetz, T_ges, delta_T );
        W_basiert = 1;
    % Polynome fuenfter Ordnung
    elseif strcmp(flag_traj,'p2p_quint') == true
        [ W, dot_W, ddot_W, T ] = p2p_quintisch( W_stuetz, T_ges, delta_T );
        W_basiert = 1;
    % Kubische Splines
    elseif strcmp(flag_traj,'kubischeSpline') == true
        [ W, dot_W, ddot_W, T ] = kubischer_spline( W_stuetz, T_ges, delta_T );
        W_basiert = 1;
    % Via-Point-Verfahren mit Ueberschleifen
    elseif strcmp(flag_traj,'paraBlends') == true
        [ Q, dot_Q, ddot_Q, T, Q_I, T_I ] = parabolic_blends( W_stuetz, delta_T );
        W_basiert = 0;
    end

%% 5. Ausgabe fuer den Viewer
% -------------------------------------------------------------------------
rob = erstelle_roboter();
% IK, falls noetig
if W_basiert
  [Q] = berechne_gelenkwinkel(W);
end

if ~W_basiert
  W = zeros(3,length(T));
  dot_W = zeros(size(W));
  ddot_W = zeros(size(W));
end
V = zeros(3,4,6,length(T));

% DK für Transformationsmatrizen und Arbeitsraumplots
for t = 1:length(T)
  rob.q = Q(:,t);
  rob.zeit = T(t);
  rob = berechne_dk_positionen_vektorkette( rob );
  for l = 1:6
    V( :, 1, l, t ) = rob.kl(l).B0_r_i;
    V( :, 2:4, l, t ) = rob.kl(l).A_i0;
  end
  if ~W_basiert
    W(:,t) = rob.w;
  end
end

% Speichern der Gelenkwinkel fuer Viewer
write_data( T, V, 6, 'trajectory_D.csv' );

%% 6. Plots zur Ueberpruefung
% -------------------------------------------------------------------------
if Flag == true
    figure();
    if W_basiert
      subplot(3,1,1);
      plot( T, W,'-');
      legend('x(t)', 'y(t)', 'z(t)' );
      title('\bf{Position}','Interpreter','latex')
      subplot(3,1,2);
      plot( T, dot_W,'-');
      title('\bf{Geschwindigkeit}','Interpreter','latex')
      subplot(3,1,3);
      plot( T, ddot_W,'-');
      title('\bf{Beschleunigung}','Interpreter','latex')
    else
      subplot(3,1,1);
      plot( T, Q,'-');
      hold on
      set(gca, 'ColorOrderIndex', 1)
      plot( T_I, Q_I,'.');
      title('\bf{Gelenkwinkelposition} $q$','Interpreter','latex')
      subplot(3,1,2);
      plot( T, dot_Q,'-');
      title('\bf{Gelenkwinkelgeschwindigkeit} $\dot{q}$','Interpreter','latex')
      subplot(3,1,3);
      for g = 1:N_Q
        stairs( T, ddot_Q(g,:) )
        hold on
      end
      ylim(1.1*max(max(ddot_Q))*[-1,1])
      title('\bf{Gelenkwinkelbeschleunigung} $\ddot{q}$','Interpreter','latex')
    end
    %3D-Plot der Bahn
    figure();
    plot3( W(1,:), W(2,:), W(3,:) );
    hold on
    plot3( W_stuetz(1,:), W_stuetz(2,:), W_stuetz(3,:), '.');
    title('\bf{Bahn im Arbeitsraum}','Interpreter','latex' );
    xlabel('x')
    ylabel('y')
    zlabel('z')
    grid on
    axis square
    axis equal
    view([-20 -5 15]);
    lim = max(max(abs(W)));
    xlim([-2*lim,2*lim])
    ylim([-2*lim,2*lim])
    zlim([-2*lim,2*lim])
end

