function rob = berechne_lagerkraefte(rob)
    % Inverse Dynamik fuer Roboter rob berechnen
% Die Ergebnisse werden wiederum in der Struktur rob.gespeichert
%
% Die Lagerkräfte werden rückwärts berechnet. Es gibt 5 eingeschränkte
% Freiheitsgrade pro Gelenk
%1. Mit Null initialisieren
rob.Q_lager=zeros(rob.N_Q,rob.N_Q);

%2. Kinematik berechnen
rob=berechne_dk_positionen(rob);
rob=berechne_dk_geschwindigkeiten(rob);
rob=berechne_dk_beschleunigungen(rob);
rob=berechne_dk_jacobis(rob);

%3. Berechnung fuer alle Koerper: Lagerkräfte
for i=length(rob.kl):-1:1
    nach = i + 1;
    %Absolutbeschleunigung des Schwerpunkts: 
        rob.kl(i).Bi_r_i_s = rob.kl(i).Bi_r_s; % -rob.kl(i).Bi_r_i;
        rob.kl(i).Bi_ddot_r_s = rob.kl(i).Bi_ddot_r_i + ...
            (tilde(rob.kl(i).Bi_dot_omega) + tilde(rob.kl(i).Bi_omega)*tilde(rob.kl(i).Bi_omega))*...
            rob.kl(i).Bi_r_s;
    
    %Impulsaenderung - Schwerkraft
        if nach == rob.N_Q + 1
            rob.kl(i).Bi_F_iv = rob.kl(i).m * rob.kl(i).Bi_ddot_r_s - rob.kl(i).m * rob.kl(i).A_i0 * rob.B0_g;
        else
            rob.kl(i).Bi_F_iv = rob.kl(i).m * rob.kl(i).Bi_ddot_r_s - rob.kl(i).m * rob.kl(i).A_i0 * rob.B0_g + rob.kl(nach).A_iv' * rob.kl(nach).Bi_F_iv;
        end
    %Drallaenderung - Moment der Schwerkraft
        if nach == rob.N_Q + 1
            rob.kl(i).Bi_M_iv = rob.kl(i).I_o * rob.kl(i).Bi_dot_omega + tilde(rob.kl(i).Bi_omega) * rob.kl(i).I_o * rob.kl(i).Bi_omega ...
            + rob.kl(i).m * tilde(rob.kl(i).Bi_r_s) * rob.kl(i).Bi_ddot_r_i ...
            - rob.kl(i).m * tilde(rob.kl(i).Bi_r_s) * rob.kl(i).A_i0 * rob.B0_g; % Anteil des vorherigen Gelenks
        else
            rob.kl(i).Bi_M_iv = rob.kl(i).I_o * rob.kl(i).Bi_dot_omega + tilde(rob.kl(i).Bi_omega) * rob.kl(i).I_o * rob.kl(i).Bi_omega ...
            + rob.kl(i).m * tilde(rob.kl(i).Bi_r_s) * rob.kl(i).Bi_ddot_r_i ...
            - rob.kl(i).m * tilde(rob.kl(i).Bi_r_s) * rob.kl(i).A_i0 * rob.B0_g ...
            + rob.kl(nach).A_iv' * rob.kl(nach).Bi_M_iv + tilde(rob.kl(nach).Bv_r_vi)*rob.kl(nach).A_iv'*rob.kl(i).Bi_F_iv; % Anteil des vorherigen Gelenks
        end

end