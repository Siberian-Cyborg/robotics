function rob = berechne_dk_geschwindigkeiten(rob)
    % Calculate velocities for each body


    % ---------------------------------------------------------------------
    %for each body
    for i = 1:rob.N_Q
        % index of predecessor body
        vor = rob.kl(i).vorgaenger;
        first = vor == -1;
        % Relativ rotational velocity
        % ----------------------------------------
        rob.kl(i).Bi_omega_rel = [0;0;rob.dot_q(i)];

        % absolute rotational velocity
        % ----------------------------------------
        if first
            rob.kl(i).Bi_omega = rob.kl(i).Bi_omega_rel;
        else
            rob.kl(i).Bi_omega = rob.kl(i).A_iv * rob.kl(vor).Bi_omega + rob.kl(i).Bi_omega_rel;
        end

        % Absolut translational velocity
        % ----------------------------------------------
        if first
            rob.kl(i).Bi_dot_r_i = 0;
        else
            rob.kl(i).Bi_dot_r_i = rob.kl(i).A_iv * (rob.kl(vor).Bi_dot_r_i + cross(rob.kl(vor).Bi_omega,rob.kl(i).Bv_r_vi));
        end
    end

    % Velocity of tool center point (TCP) in B0-System
    rob.dot_w = rob.kl(rob.N_Q).A_i0' * (rob.kl(rob.N_Q).Bi_dot_r_i + cross(rob.kl(rob.N_Q).Bi_omega, rob.BN_r_N_tcp));

end
