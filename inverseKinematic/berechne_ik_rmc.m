function rob = berechne_ik_rmc(rob,W,K,Option)
    % Berechnung der Inversen Kinematik ueber Resolved Motion Rate Control
    % W     ...positiv definite Wichtungsmatrix
    % K     ...Driftkompensationsmatrix
    % Option...dem Verfahren uebergebene Optionen

    % Berechnung der mit W gewichteten Pseudoinversen der Arbeitsraum-Jacobimatrix
     Jw_pseudo = W\rob.Jw'/(rob.Jw/W*rob.Jw');

    % Berechnung der Gelenkwinkelgeschwindigkeit ueber Resolved Motion Rate Control
    if strcmp(Option,'drift') == true
        % Aufgabe 2.1 RMC - ohne Driftkompensation
        dot_q_new = Jw_pseudo*rob.dot_w_d;

    elseif strcmp(Option,'driftcomp') == true
        % Aufgabe 2.2 RMC - mit Driftkompensation
        w_dot_d_eff = K*(rob.w_d-rob.w) + rob.dot_w_d;
        dot_q_new = Jw_pseudo*w_dot_d_eff;
        
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
