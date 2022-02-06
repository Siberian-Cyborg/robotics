function rob = erstelle_roboterstruktur(n_Elemente)
    % Erzeugt eine (leere) Datenstruktur mit allen notwenigen Variablen

    %% --- Allgemeine Daten/Konstanten ------------------------------------
    rob.N_Q = n_Elemente;   % Anzahl der Freiheitsgrade (Dimension von q)
    rob.zeit = 0;           % Aktuelle Simulationszeit
    rob.dt = 0;             % Zeitschrittweite
    rob.B0_g = zeros(3,1);  % Erdbeschleunigung

    %% --- Geometriedaten -------------------------------------------------
    % Laengen, Breiten und Dicken der einzelnen Koerper
    rob.l = zeros(1,6);
    rob.b = zeros(1,6);
    rob.d = zeros(1,6);

    %% --- Eigentliche RD-Variablen ---------------------------------------
    % Vektor der generalisierten Koordinaten, Geschwindigkeiten und Beschleunigungen
    rob.q = zeros(rob.N_Q,1);
    rob.dot_q = zeros(rob.N_Q,1);
    rob.ddot_q = zeros(rob.N_Q,1);

    % Arbeitsraum-Koordinaten (Endeffektor-Position): Istwert, Sollwert und Ableitungen
    rob.w = zeros(3,1);
    rob.dot_w = zeros(3,1);
    rob.w_d = zeros(3,1);
    rob.dot_w_d = zeros(3,1);

    % Vektor der generalisierten (Antriebs-)Kraefte
    rob.tau_id = zeros(rob.N_Q,1);
    rob.tau_antrieb = zeros(rob.N_Q,1);

    % Massenmatrix und h Vektor fuer Dynamik
    rob.M = zeros(rob.N_Q);
    rob.h = zeros(rob.N_Q,1);

    % Vektor zum TCP, relativ zum koerperfesten Koordinatensystem des letzten Koerpers
    rob.BN_r_N_tcp = zeros(3,1);

    % Jacobi-Matrix der Endeffektor-Position
    rob.Jw = zeros(3,rob.N_Q);

    % Variablen der einzelnen Koerper anlegen
    for i = 1:rob.N_Q
        % Vorgaengerkoerper
        rob.kl(i).vorgaenger = 0;

        % Freiheitsgrad, der vom Koerper belegt wird (d.h. hier theta=q(fhg_no)
        rob.kl(i).fhg_no = 0;

        % DH-Parameter (alpha, a und d sind Konstanten fuer rein rotatorische Gelenke)
        rob.kl(i).alpha = 0.0;
        rob.kl(i).a = 0;
        rob.kl(i).d = 0;

        % Masse des i-ten Koerpers
        rob.kl(i).m = 0;

        % Vektoren im inertialen Koordinatensystem
        rob.kl(i).B0_r_i = zeros(3,1);          % Position des Ursprungs B0-System (inertial)

        % Vektoren im i-ten Koordinatensystem
        rob.kl(i).Bi_r_s = zeros(3,1);          % Schwerpunkt relativ zum Koerper Kosy-Ursprung
        rob.kl(i).Bi_dot_r_s = zeros(3,1);      % Absolutgeschwindigkeit des Schwerpunkts
        rob.kl(i).Bi_ddot_r_s = zeros(3,1);     % Absolutbeschleunigung des Schwerpunkts
        rob.kl(i).Bi_r_i = zeros(3,1);          % Position des Ursprungs des i-ten Koerpers im KOS des i-ten Koerpers
        rob.kl(i).Bi_dot_r_i = zeros(3,1);      % Absolutgeschwindigkeit des Ursprungs des i-ten Koerpers im KOS des i-ten Koerpers
        rob.kl(i).Bi_ddot_r_i = zeros(3,1);     % Absolutbeschleunigung des Ursprungs des i-ten Koerpers im KOS des i-ten Koerpers

        rob.kl(i).Bi_omega = zeros(3,1);        % absolute Winkelgeschwindigkeit des i-ten Koerpers im i-ten KOS
        rob.kl(i).Bi_dot_omega = zeros(3,1);    % absolute Winkelbeschleunigung des i-ten Koerpers im i-ten KOS
        rob.kl(i).Bi_omega_rel = zeros(3,1);    % relativer Anteil
        rob.kl(i).Bi_dot_omega_rel = zeros(3,1);% Ableitung des relativen Anteils

        rob.kl(i).Bi_g = zeros(3,1);            % Erdbeschleunigung im System des i-ten Koerpers

        % Vektoren im Vorgaenger-Koordinatensystem
        rob.kl(i).Bv_r_vi = zeros(3,1);         % Verschiebungsvektor vom Vorgaenger zum Koerper i im KOS des Vorgaengers (Bv)

        % Drehmatrizen
        rob.kl(i).A_iv = eye(3);                % Drehmatrix vom Vorgaenger zum i-ten Koerper
        rob.kl(i).A_i0 = eye(3);                % Drehmatrix vom Inertialsystem (=B0) zum i-ten Koerper

        % DH-Matrizen
        rob.kl(i).D_vi = eye(4);                % Homogene Transformationsmatrix vom i-ten Koerper zum Vorgaenger
        rob.kl(i).D_0i = eye(4);                % Homogene Transformationsmatrix vom i-ten Koerper ins Inertialsystem

        % Jacobi-Matrizen
        rob.kl(i).Bi_Jr = zeros(3,length(rob.q));   % Jacobi-Matrix der Rotation, dargestellt im Bi-KOS (d(Bi_omega)/d(dot(q))
        rob.kl(i).Bi_Jt_o = zeros(3,length(rob.q)); % Jacobi-Matrix der Translation, dargestellt im Bi-KOS

        % Traegheitstensoren
        rob.kl(i).I_g = zeros(3);               % Traegheitstensor bezueglich Schwerpunkts des KOSY
        rob.kl(i).I_g_Steiner = zeros(3);       % Steiner Anteil aus Offset
        rob.kl(i).I_o = zeros(3);               % Traegheitstensor bezueglich Ursprung des KOSY
    end
end
