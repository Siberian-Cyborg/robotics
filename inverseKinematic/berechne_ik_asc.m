function rob = berechne_ik_asc(rob,W,K,alpha,Option)
    % Berechnung der Inversen Kinematik ueber Automatic Supervisory Control
    % W     ...positiv definite Wichtungsmatrix
    % K     ...Driftkompensationsmatrix
    % alpha ...Wichtungsfaktor der Guetefunktion
    % Option...dem Verfahren uebergebene Optionen

    % Berechnung des Gradienten der Guetefunktion
    if strcmp(Option,'comfort') == true
        [~,grad_H] = H_comfort(rob.q);
    elseif strcmp(Option,'limit') == true
        [~,grad_H] = H_limit(rob.q);
    elseif strcmp(Option,'both') == true
        [~,grad_H_comfort] = H_comfort(rob.q);
        [~,grad_H_limit] = H_limit(rob.q);
        grad_H = grad_H_comfort+grad_H_limit;
    else
        % Ungueltige Option
        error('Ungueltige Option gewaehlt!')
    end

    Jw_pseudo = W\rob.Jw'/(rob.Jw/W*rob.Jw');
    dot_w_d_eff = rob.dot_w_d + K*(rob.w_d-rob.w);
    N_w = eye(rob.N_Q,rob.N_Q) - Jw_pseudo * rob.Jw;

    dot_q_new = Jw_pseudo * dot_w_d_eff - alpha*N_w/W*grad_H;

    % Berechnung der Gelenkwinkelbeschleunigung aus Differenzenquotient
    rob.ddot_q = (dot_q_new-rob.dot_q)/rob.dt;

    % Uebernehmen der berechneten Gelenkwinkelgeschwindigkeit
    rob.dot_q = dot_q_new;

    % Gelenkwinkel ueber explizites Euler-Verfahren berechnen
    rob.q = rob.q+rob.dot_q*rob.dt;
end
