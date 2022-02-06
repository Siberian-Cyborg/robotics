function rob = berechne_dk_positionen(rob)
    % Berechnung der Positions-Groessen der direkten Kinematik (Lage und Orientierung)
    % ueber die Bildung einer Vektorkette


    % Berechnung fuer alle Koerper
    for i = 1:rob.N_Q
        % Index des Vorgaengers merken
        vor = rob.kl(i).vorgaenger;

        % Relativkinematik: Position und Orientierung relativ zum Vorgaenger
        % ------------------------------------------------------------------
        % Verschiebungsvektor vom Vorgaenger zum Koerper i im KOS des Vorgaengers (Bv)
        rob.kl(i).Bv_r_vi = [                      rob.kl(i).a;...
                             -sin(rob.kl(i).alpha)*rob.kl(i).d;...
                              cos(rob.kl(i).alpha)*rob.kl(i).d];

        % Drehmatrix vom Vorgaenger zum i-ten Koerper
        rob.kl(i).A_iv = Az(rob.q(i))*Ax(rob.kl(i).alpha);

        % Absolute Position und Orientierung
        % ----------------------------------
        % Drehmatrix vom B0-KOS ins Bi-KOS:
        if vor == -1
            rob.kl(i).A_i0 = rob.kl(i).A_iv;
        else
            rob.kl(i).A_i0 = rob.kl(i).A_iv*rob.kl(vor).A_i0;
        end

        % Position des Ursprungs des i-ten Koerpers im Bi-KOS:
        if vor == -1
            rob.kl(i).Bi_r_i = rob.kl(i).A_iv*rob.kl(i).Bv_r_vi;
        else
            rob.kl(i).Bi_r_i = rob.kl(i).A_iv*(rob.kl(vor).Bi_r_i+rob.kl(i).Bv_r_vi);
        end


        % Position des Ursprungs im B0-System (fuer Animation benoetigt):
        rob.kl(i).B0_r_i = rob.kl(i).A_i0'*rob.kl(i).Bi_r_i;
    end

    % Position des TCP im B0-System berechnen
    rob.w = rob.kl(rob.N_Q).B0_r_i+rob.kl(rob.N_Q).A_i0'*rob.BN_r_N_tcp;

end