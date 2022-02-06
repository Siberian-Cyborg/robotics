function rob = berechne_dk_positionen_dh_trafo(rob)
	% Calculation of position and orientation of direct kinematic using
    % denavit hartenberg convention


    % for every body
    for i = 1:rob.N_Q
        % index of predecessor body
        vor = rob.kl(i).vorgaenger;

        % Relativ kinematic: Position und Orientation relativ to
        % predecessor
        % ------------------------------------------------------------------
        % Homogeneous Transformation from i-th body to predecessor

        rob.kl(i).D_vi = dh_trafo(rob.kl(i).alpha, rob.kl(i).a, rob.kl(i).d, rob.q(i));

        % Absolute Position and Orientation
        % ----------------------------------
        % Homogeneous Transformation from i to 0-System
        if vor == -1
            rob.kl(i).D_0i = rob.kl(i).D_vi;
        else
            rob.kl(i).D_0i = rob.kl(vor).D_0i * rob.kl(i).D_vi;
        end
        % vector from inertial system to i-th body given in B0-System
        rob.kl(i).B0_r_i = rob.kl(i).D_0i(1:3, end);

        % Rotational matrix from B0 to Bi-System
        rob.kl(i).A_i0 = rob.kl(i).D_0i(1:3, 1:3)';
    end
    % Position of tool center point in B0-System
    rob.w = rob.kl(rob.N_Q).A_i0 * rob.BN_r_N_tcp + rob.kl(rob.N_Q).B0_r_i;

end
