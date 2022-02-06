function rob = berechne_ik_xxx(rob,Option)
    % Berechnung der Inversen Kinematik
    % Option ...'VA' Variante A
    %        ...'VB' Bariante B

    % Berechnung der Gelenkwinkelgeschwindigkeit...
    if strcmp(Option,'VA') == true
    dot_q_new = rob.Jw\rob.dot_w_d;

    elseif strcmp(Option,'VB') == true
    dot_q_new = rob.Jw' * rob.dot_w_d;
        
    else
        % Ungueltige Option
        error('Ungueltige Option gewaehlt!')
    end


    % Berechnung der Gelenkwinkelbeschleunigung aus Differenzenquotient
    rob.ddot_q = (dot_q_new-rob.dot_q)/rob.dt;

    % Uebernehmen der berechneten Gelenkwinkelgeschwindigkeit
    rob.dot_q = dot_q_new;

    % Gelenkwinkel ueber explizites Euler-Verfahren berechnen
    rob.q = rob.q+rob.dot_q*rob.dt;
end
