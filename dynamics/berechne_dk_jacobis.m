function rob = berechne_dk_jacobis(rob)
    % Berechnung der Jacobi-Matrizen der Rotation, Translation und Arbeitsraumkoordinaten
    % Hinweis: vor dem Aufruf dieser Funktion muss berechne_dk_positionen ausgefuehrt werden, um
    % Zugriff auf aktuellen Drehmatrizen zu haben. Dies ist bereits in Inverse_Kinematik.m
    % implementiert.


    % Berechnung fuer alle Koerper
    for i=1:rob.N_Q
        % Index des Vorgaengers merken
        vor = rob.kl(i).vorgaenger;
        
        Bi_J_R_rel = zeros(3,rob.N_Q);
        Bi_J_R_rel(3,i) = 1;
        
        if vor == -1
            % Jacobi-Matrix der Rotation des ersten Körpers
        % -----------------------------------------
            rob.kl(i).Bi_Jr =  Bi_J_R_rel;
        % Jacobi-Matrix der Translation des ersten Körpers
        % --------------------------------------------
            rob.kl(i).Bi_Jt_o = zeros(3,rob.N_Q);
        else
            % Jacobi-Matrix der Rotation des Koerpers i
        % -----------------------------------------
            rob.kl(i).Bi_Jr = rob.kl(i).A_iv * rob.kl(vor).Bi_Jr +  Bi_J_R_rel;
            
        % Jacobi-Matrix der Translation des Koerpers i
        % --------------------------------------------     
            rob.kl(i).Bi_Jt_o = rob.kl(i).A_iv * rob.kl(vor).Bi_Jt_o + rob.kl(i).A_iv * tilde(rob.kl(i).Bv_r_vi)' * rob.kl(vor).Bi_Jr;
        end
        
    end

    % Jacobi-Matrizen fuer TCP
    % ------------------------
    % Jacobi-Matrix der Rotation des TCP dargestellt im B0-KOS
    B0_Jr = rob.kl(rob.N_Q).A_i0' * rob.kl(rob.N_Q).Bi_Jr;

    % Jacobi-Matrix der Translation des TCP dargestellt im B0-KOS
    B0_Jt_o = rob.kl(rob.N_Q).A_i0' * (rob.kl(rob.N_Q).Bi_Jt_o +  tilde(rob.BN_r_N_tcp)'  * rob.kl(rob.N_Q).Bi_Jr);

    % Jacobi-Matrix der Arbeitsraum-Koordinaten
    % -----------------------------------------
%      rob.Jw = B0_Jt_o;


    rob.Jw = [B0_Jt_o; B0_Jr]; 
end