function rob=berechne_bgl(rob)
%Berechnung Bewegungsgleichungen des Systems (M, h und ddot_q)
%
% Alle notwendigen Funktionen werden von hier aus aufgerufen
%

%% 1. Berechnung des h-Vektors:  


rob.ddot_q = zeros(1,rob.N_Q);
rob = berechne_id(rob);
rob.h = rob.tau_id;

%% 2. Berechnung der Massenmatrix
rob.M=zeros(rob.N_Q,rob.N_Q);

% Beitraege aller Koerper addieren
for i=1:length(rob.kl)
    %Anteil dieses Koerpers
    M_i = [rob.kl(i).m*eye(3) , rob.kl(i).m * tilde(rob.kl(i).Bi_r_s)' ; rob.kl(i).m * tilde(rob.kl(i).Bi_r_s), rob.kl(i).I_o];
    J_i = [rob.kl(i).Bi_Jt_o; rob.kl(i).Bi_Jr];
    dM = J_i' * M_i * J_i;

    %Anteil zur Gesamt-Massenmatrix addieren 
    rob.M=rob.M+dM;
end

%Die aktuellen Beschleunigungen berechnen
%Hier werden auch die Antriebsmomente der Regelung tau_reg beruecksichtigt

rob.ddot_q= rob.M\(rob.tau_antrieb - rob.h); 

end
