function rob = berechne_dk_positionen_vektorkette(rob)
    % Calculation of position and orientation of direct kinematic using
    % a vector chain


    % for every body
    for i = 1:rob.N_Q
        % Index of predecessor 
        vor = rob.kl(i).vorgaenger;

        % Relativ kinematic: Position und Orientation relativ to
        % predecessor
        % ------------------------------------------------------------------
        % Translational vector from predecessor to i-th body in coordinate system
        % of predecessor
        rob.kl(i).Bv_r_vi = [                      rob.kl(i).a;...
                             -sin(rob.kl(i).alpha)*rob.kl(i).d;...
                              cos(rob.kl(i).alpha)*rob.kl(i).d];

        % Rotational matrix from predecessor to current body
        rob.kl(i).A_iv = Az(rob.q(i))*Ax(rob.kl(i).alpha);

        % Absolute Position und Orientation
        % ----------------------------------
        % Rotational matrix from B0 to Bi-System
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



        rob.kl(i).B0_r_i = rob.kl(i).A_i0'*rob.kl(i).Bi_r_i;
    end

     % Position of tool center point in B0-System
    rob.w = rob.kl(rob.N_Q).B0_r_i+rob.kl(rob.N_Q).A_i0'*rob.BN_r_N_tcp;

end
