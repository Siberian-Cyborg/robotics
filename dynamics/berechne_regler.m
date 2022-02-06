function tau_reg = berechne_regler(rob, qd, dot_qd, Kp, Kd, verfahren)
% Die Funktion bietet für die direkte Dynamik zwei Reglerverfahren an:
%       1. P-Regler auf Positionsebene
%       2. PD-Regler auf Positionsebene

if strcmp(verfahren, "P")
    tau_reg = Kp*(qd - rob.q);
elseif strcmp(verfahren, "PD")
    tau_reg = Kp*(qd - rob.q) + Kd*(dot_qd - rob.dot_q);
else
    warning("Ungültiges Reglerverfahren!")
    tau_reg = zeros(rob.N_Q,1);
end