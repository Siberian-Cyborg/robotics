function [ W, dot_W, ddot_W, T ] = p2p_kubisch( W_stuetz, T_ges, delta_T )
% Erzeugt aus N_I Stuetzpunkten in W_stuetz je mit Anfangs- und Endpunkt
% N_I-1 Trajektorien je in Form eines kubischen Polynoms.
%
% W         := Trajektorie auf Positionsebene
% dot_W     := Trajektorie auf Geschwindigkeitsebene
% ddot_W    := Trajektorie auf Beschleunigungsebene
% T         := Zeitvektor der Trajektorie
%
% W_stuetz  := Stuetzpunkte
% T_ges     := Dauer der Bewegung
% delta_T   := Taktzeit

%% 1. Preprocessing
% Anzahl der Stuetzpunkte der zu planenden Trajektorie
N_I       = size( W_stuetz,2 );
% Anzahl der Freiheitsgrade der Stuetzpunkte der Trajektorie
N_Q       = size( W_stuetz,1 );
% Zeitintervall fuer jedes Teilstueck der Trajektorie
T_I       = 0:delta_T:(T_ges/(N_I-1));
% Anzahl der Zeitpunkte eines Teilstuecks
N_T_I     = length(T_I);          

%% 2. Initialisierung der Teiltrajektorien und der Gesamttrajektorie
% Initialisierung der Trajektorien der Teilstuecke
W_I       = zeros( N_Q, N_T_I );
dot_W_I   = zeros(size(W_I));
ddot_W_I  = zeros(size(W_I));
% Initialisierung der Bahnparameter der Teilstuecke (die Verwendung des
% Bahnparameters ist nicht zwingend erforderlich)
S_I       = zeros( N_Q, N_T_I );
dot_S_I   = zeros(size(S_I));
ddot_S_I  = zeros(size(S_I));
% Initialisierung der Gesamttrajektorie
W         = [];
dot_W     = [];
ddot_W    = [];
T = [];
counter = 0;


% Berechnung der Trajektorie

for j = 1:(N_I-1) %durch alle Stützpunkte
    for i = 1:N_T_I  %durch alle Zeitschritte pro Teilstück
        counter = counter + 1;
        W_i = -2 * (W_stuetz(:, j+1)-W_stuetz(:,j))/(T_I(end)^3) * T_I(i)^3  + 3 * (W_stuetz(:,j+1)-W_stuetz(:,j))/(T_I(end)^2) * T_I(i)^2 + W_stuetz(:,j);
        dot_W_i = -6 * (W_stuetz(:,j+1)-W_stuetz(:,j))/(T_I(end)^3) * T_I(i)^2  + 6 * (W_stuetz(:,j+1)-W_stuetz(:,j))/(T_I(end)^2) * T_I(i);
        ddot_W_i = -12 * (W_stuetz(:,j+1)-W_stuetz(:,j))/(T_I(end)^3) * T_I(i)  + 6 * (W_stuetz(:,j+1)-W_stuetz(:,j))/(T_I(end)^2);

        W_I(:,i) = W_i;
        dot_W_I(:,i) = dot_W_i;
        ddot_W_I(:,i) = ddot_W_i;
        T = [T,counter*delta_T];
    end
    
    W = [W, W_I];
    dot_W = [dot_W, dot_W_I];
    ddot_W = [ddot_W, ddot_W_I];
end


end % function

