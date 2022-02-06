function [Q_traj, dot_Q_traj, ddot_Q_traj, T, Q_stuetz, T_stuetz] = parabolic_blends( W_stuetz, delta_T )
  % Erzeugt aus N_I Stuetzpunkten in W_stuetz eine Trajektorie mit
  % stueckweise konstanten Bescheunigungen unter Einhaltung der maximalen
  % Gelenkwinkelgeschwindigkeiten und -beschleunigungen.
  %
  % Q_traj         := Gelenkwinkeltrajektorie auf Positionsebene
  % dot_Q_traj     := Gelenkwinkeltrajektorie auf Geschwindigkeitsebene
  % ddot_Q_traj    := Gelenkwinkeltrajektorie auf Beschleunigungsebene
  % T_stuetz       := Zeitpunkte bei Erreichen der Stuetzpunkte
  % Q_stuetz       := Gelenkwinkelkonfigurationen der Stuetzstellen
  %
  % W_stuetz       := Stuetzpunkte
  % delta_T        := Taktzeit

  %% 1. Preprocessing
  % Gegebene Aktuatorgrenzen
  dot_Q_max = 2.4 * [ 1, 1, 0.8, 1, 1, 1 ]; % [rad/s]
  ddot_Q_max = 32 * [ 1, 1, 1, 1, 1, 1 ]; % [rad/s^2]
  % Berechnung der Gelenkwinkel der Stuetzpunkte
  Q_stuetz = berechne_gelenkwinkel(W_stuetz);
  % Anzahl der Stuetzpunkte
  N_I = length(Q_stuetz(1,:));
  % Anzahl der Freiheitsgrade
  N_Q = length(Q_stuetz(:,1));
  
  %% 2. Initialisierung der Gelenkwinkelbewegungen und der Zeiten
  dot_Q = zeros(N_Q,N_I-1);
  ddot_Q = zeros(N_Q,N_I);
  t_b = zeros(N_Q,N_I);
  t_g = zeros(N_Q,N_I-1);


  % Erzeuge Trajektorie 

  
  %% Schleife ueber alle Gelenkwinkel
  for g = 1:N_Q
    %% A. Berechnung der Geschwindigkeiten in den Zeitintervallen
    dot_Q(g,:) = sign(Q_stuetz(g,2:end)-Q_stuetz(g,1:end-1))*dot_Q_max(g);
    %% B. Berechnung der Beschleunigungen in den Zeitintervallen
    ddot_Q(g,1) = sign(dot_Q(g,1)) * ddot_Q_max(g);
    ddot_Q(g,end) = sign(-dot_Q(g,end))*ddot_Q_max(g);
    for i = 2:N_I-1
        ddot_Q(g,i) = sign(dot_Q(g,i)-dot_Q(g,i-1)) * ddot_Q_max(g);
    %% C. Berechnung der Beschleunigungszeiten
        t_b(g,i) = (dot_Q(g,i)-dot_Q(g,i-1))/ddot_Q(g,i);
    end
    t_b(g,1) = dot_Q(g,1)/ddot_Q(g,1);
    t_b(g,end) = -dot_Q(g,end)/ddot_Q(g,end);
    t_b(isinf(t_b)|isnan(t_b))=0;
    
    %% D. Berechnung der benoetigten Gesamtzeiten fuer die Zeitintervalle
    t_g(g,:) = (Q_stuetz(g,2:end)-Q_stuetz(g,1:end-1))./dot_Q(g,:);
    t_g(g,1) = (Q_stuetz(g,2)-Q_stuetz(g,1))/dot_Q(g,1) + 1/2*t_b(1);
    t_g(g,end) = (Q_stuetz(g,end)-Q_stuetz(g,end-1))/dot_Q(g,end) + 1/2*t_b(end);
  end
  t_g(isinf(t_g)|isnan(t_g))=0;


  %% 4. Bestimmung der laengsten Zeitdauer zur Synchronisation
  t_max = max(t_g,[],1) ;
  if min(t_max) <= 1e-3
    error('Zwei aufeinanderfolgende Stuetzpunkte sind zu nahe beieinander.')
  end

  %% 5. Initialisierungen
  % Re-Initialisierung der Gelenkwinkelgeschwindigkeiten und Beschl.zeiten
  dot_Q         = zeros(N_Q,N_I-1);
  t_b           = zeros(N_Q,N_I);
  % Initialisierung der Schaltzeiten 
  t_const2acc       = zeros(N_Q,N_I); % Schalten auf Beschleunigungsphase.
  t_acc2const       = zeros(N_Q,N_I); % Schalten auf konst. Geschwinigikeit.
  T_stuetz       = zeros(1,N_I-1); % Zeitpunkte der Stuetzstellen
  % Berechnung der Stuetzstellenzeitpunkte
  T_stuetz = [0 cumsum(t_max)];

  %% Schleife ueber alle Gelenkwinkel
  for g = 1:N_Q
    %% A. Neuberechnung der Geschwindigkeiten in den Zeitintervallen
    dot_Q(g,:) = (Q_stuetz(g,2:end)-Q_stuetz(g,1:end-1))./t_max;
    dot_Q(g,1) = ddot_Q(g,1)*t_max(1) - ddot_Q(g,1)*sqrt(t_max(1)^2-2/ddot_Q(g,1).*(Q_stuetz(g,2)-Q_stuetz(g,1)));
    dot_Q(g,end) = -ddot_Q(g,end)*t_max(end) + ddot_Q(g,end)*sqrt(t_max(end)^2+2/ddot_Q(g,end)*(Q_stuetz(g,end)-Q_stuetz(g,end-1)));

    dot_Q(isinf(dot_Q)|isnan(dot_Q))=0;
    %% B. Neuberechnung der Bescheunigungen
    % Aus den obigen Anpassungen der Gelenkwinkelgeschw. kann es sich
    % ergeben, dass die Beschleunigungen in den naechsten Zeitintervallen
    % angepasst werden muessen.
    ddot_Q(g,1) = sign(dot_Q(g,1)) * ddot_Q_max(g);
    ddot_Q(g,end) = sign(-dot_Q(g,end))*ddot_Q_max(g);
    for i = 2:N_I-1
        ddot_Q(g,i) = sign(dot_Q(g,i)-dot_Q(g,i-1)) * ddot_Q_max(g);
    %% C. Neuberechnung der Bescheunigungszeiten
        t_b(g,i) = (dot_Q(g,i)-dot_Q(g,i-1))/ddot_Q(g,i);
    end
    t_b(g,1) = dot_Q(g,1)/ddot_Q(g,1);
    t_b(g,end) = -dot_Q(g,end)/ddot_Q(g,end);

    % Catch any inf or NaN
    t_b(isinf(t_b)|isnan(t_b))=0;

    %% D. Berechnung der Schaltzeiten
    t_const2acc(g,1) = 0;
    t_acc2const(g,1) = t_b(g,1);
    for i=2:N_I-1
      t_const2acc(g,i) = T_stuetz(i)-0.5*t_b(g,i);
      t_acc2const(g,i) = T_stuetz(i)+0.5*t_b(g,i);
    end
    t_const2acc(g,N_I) = T_stuetz(N_I)-t_b(g,N_I);
    t_acc2const(g,N_I) = T_stuetz(N_I);
  end

  %% 7. Abbruch, falls gegebene Stuetzpunkte nicht erreicht werden koennen
  for g = 1:N_Q
    for j = 1:N_I-1
      if t_acc2const(g,j) > t_const2acc(g,j+1)
        error('Diese Trajektorie kann mit den gegebenen Bedingungen nicht abgefahren werden! Bitte aendern Sie Punkte oder Maximalgeschwindigkeit/-beschleunigung.');
      end
    end
  end

  %% 8. Berechnnung der Steuertrajektorien
  % Gesamtzeit der Trajektorie
  t_end   = max(t_acc2const(:,N_I));
  T       = 0:delta_T:t_end;
  % Initialisierung der finalen Trajektorie
  Q_traj       = zeros(N_Q,length(T));
  dot_Q_traj   = zeros(size(Q_traj));
  ddot_Q_traj  = zeros(size(Q_traj));
  % Die Werte fuer die Beschleunigungen und Geschwindigkeiten werden
  % interpoliert:
  % Intialisierung der Vektoren mit bekannten Beschleunigungen und
  % Geschwindigkeiten:
  t_ddot_q_I = zeros(N_Q,N_I*4 -2);
  ddot_q_I   = zeros(N_Q,N_I*4 -2);
  t_dot_q_I  = zeros(N_Q,N_I*2);
  dot_q_I    = zeros(N_Q,N_I*2);
  % Wir nehmen an, dass der fruehest moegliche Zeitpunkt zum Aufschalten der
  % ersten Beschleunigung nach dem ersten Zeitschritt ist.
  t_ddot_q_I(:,1:4:end)  = 0.001+t_const2acc;
  % Um die Ansteuerungszeit der errechnenten Beschleunigungszeiten und
  % Zeiten mit konstanter Geschwindigkeit beizubehalten wird das Delta mit
  % in die naechste Ansteuerung genommen. Ausserdem erfolgt ein
  % Rueckschalten wieder erst mit Verzoegerung.
  t_ddot_q_I(:,2:4:end)  = 0.002+t_acc2const;
  t_ddot_q_I(:,3:4:end)  = 0.003+t_acc2const(:,1:end-1);
  t_ddot_q_I(:,4:4:end)  = t_const2acc(:,2:end);
  % Bekannte Beschleunigungen bei aktiver Beschl.:
  ddot_q_I(:,1:4:end)    = ddot_Q;
  ddot_q_I(:,2:4:end)    = ddot_Q;
  % Bekannte Beschleunigungen bei Bewegung mit konst. Geschw.:
  ddot_q_I(:,3:4:end)    = zeros(N_Q,N_I-1);
  ddot_q_I(:,4:4:end)    = zeros(N_Q,N_I-1);
  % Zeiten bei Beweg. mit konst. Geschw.:
  t_dot_q_I(:,1:2:end)   = t_const2acc;
  t_dot_q_I(:,2:2:end)   = 0.001+t_acc2const;
  % Bekannten Geschwindigkeiten bei Beweg. mit konst. Geschw.:
  dot_q_I(:,1)           = zeros(N_Q,1);
  dot_q_I(:,2:2:end-1)   = dot_Q;
  dot_q_I(:,3:2:end-1)   = dot_Q;
  dot_q_I(:,end)         = zeros(N_Q,1);
  % Interpolation der Werte und Integration der Position:
  for g=1:N_Q
    ddot_Q_traj(g,:) = interp1(t_ddot_q_I(g,:),ddot_q_I(g,:),T,'previous');
    dot_Q_traj(g,:)  = interp1(t_dot_q_I(g,:),dot_q_I(g,:),T);
    Q_traj(g,:)      = trapz_int(t_dot_q_I(g,:),dot_q_I(g,:),T,Q_stuetz(g,1));
  end
  ddot_Q_traj(:,1) = zeros(N_Q,1);
end


%% Funktion zur Integration und Interpolation
function Q = trapz_int( t_dot_Q_stuetz, dot_Q_stuetz, T, Q_0 )
  Q = zeros(1,length(T));
  Q(:,1) = Q_0;
  dot_Q  = dot_Q_stuetz(1);
  for t=2:length(T)
    dot_Q_last  = dot_Q;
    dot_Q       = interp1(t_dot_Q_stuetz,dot_Q_stuetz,T(t));
    Q(t)        = Q(t-1) + (dot_Q_last+dot_Q)/2*(T(t)-T(t-1));
  end
end
