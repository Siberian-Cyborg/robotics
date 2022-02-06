function [H,grad_H] = H_limit(q)
    % Berechnung der Guetefunktion sowie dessen Gradienten
    % in Abhaengigkeit von der aktuellen Konfiguration (Gelenkwinkel)
    % zur Vermeidung von Gelenkwinkelbeschraenkungen

    % Definition der Gelenkwinkelbeschraenkungen
    q_max = [    pi/2.0; ...
                     pi; ...
                 pi/2.0; ...
                 pi/2.0; ...
             3.0/4.0*pi; ...
                     pi];
    q_min = -q_max;

    % Straffunktion
    H = 1/((q - q_max)'*(q - q_max)) + 1/((q - q_min)'*(q - q_min));

    % Gradient der Straffunktion
    grad_H = -2*H./(q-q_max) - 2*H./(q-q_min);

end

