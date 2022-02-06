function rob = berechne_simulationszeitschritt(rob, numIntVerfahren, dot_q_1, dot_q_2, ddot_q_1, ddot_q_2, j)

    if strcmp(numIntVerfahren,'Euler') == true
        rob = berechne_bgl(rob);
        rob.dot_q = rob.dot_q + rob.ddot_q * rob.dt; 
        rob.q = rob.q + rob.dot_q * rob.dt;      
    elseif strcmp(numIntVerfahren,'Adams Bashforth') == true
        rob = berechne_bgl(rob);
        if j>=3
            rob.q = rob.q +  rob.dt* (23/12 * rob.dot_q - 16/12 * dot_q_1 + 5/12 * dot_q_2);
            rob.dot_q = rob.dot_q + rob.dt* (23/12 * rob.ddot_q - 16/12 * ddot_q_1 + 5/12 * ddot_q_2);
        else
            rob.q = rob.q + rob.dot_q * rob.dt;
            rob.dot_q = rob.dot_q + rob.ddot_q * rob.dt;
        end
    else
        warning('Integrationsverfahren nicht gefunden!')
        return;
    end
end
