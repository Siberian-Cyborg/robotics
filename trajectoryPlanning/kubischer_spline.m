function [ W, dot_W, ddot_W, T ] = kubischer_spline( W_stuetz, T_ges, delta_T )
% Erzeugt aus N_I Stuetzpunkten in W_stuetz eine Trajektorie in Form
% kubischer Splines.
%
% W         := Trajektorie auf Positionsebene
% dot_W     := Trajektorie auf Geschwindigkeitsebene
% ddot_W    := Trajektorie auf Beschleunigungsebene
% T         := Zeitvektor der Trajektorie
%
% W_stuetz  := Stuetzpunkte
% T_ges     := Dauer der Bewegung/Interpolation
% delta_T   := Taktzeit

%% 1. Preprocessing
% Anzahl der Stuetzpunkte der zu planenden Trajektorie
N_I       = size( W_stuetz,2 );
% Anzahl der Freiheitsgrade der Stuetzpunkte der Trajektorie
N_Q       = size( W_stuetz,1 );
% Anzahl der Zeitpunkte der Trajektorie
N_T       = T_ges / delta_T + 1;          
% Vektor der Zeitintervalle zwischen den Splines
h         = T_ges / ( N_I - 1 ) * ones( 1, N_I - 1 );

%% 2. Initialisierung der der Gesamttrajektorie
W       = zeros( N_Q, N_T );
dot_W   = zeros( N_Q, N_T );
ddot_W  = zeros( N_Q, N_T );
T       = delta_T * (0:N_T-1);

%% 3. Erzeuge Gesamtrajektorie 
for i = 1:N_Q
    [W(i, :), dot_W(i, :), ddot_W(i, :)] = kubischer_spline_skalar( W_stuetz(i, :), h, delta_T );
end

end

function [w, dot_w, ddot_w] = kubischer_spline_skalar( p, h, delta_T )%
%
%% A. Gleichungssystem A* ddot_p = r aufstellen
% Initialisierung des Gleichungssystems zum Loesen von ddot_p
     N_I = length(p);
     A = zeros(N_I);
     r = zeros(N_I,1);
% Initialisierung der Spline Koeffizienten
     a = zeros(N_I-1,1);
     b = zeros(N_I-1,1);
     c = zeros(N_I-1,1);
     d = zeros(N_I-1,1);
%
%% B. Berechnung der A-Matrix und des r-Vektors
%     (...)
%
     A(1,1) = 2 * h(1);
     A(1,2) = h(1);
     A(N_I,N_I-1) = h(N_I-1);
     A(N_I,N_I) = 2*h(N_I-1);
     r(1) = 6*(p(2)-p(1))/h(1);
     r(N_I) = -6*(p(N_I)-p(N_I-1))/h(N_I-1);
for i = 2:N_I-1
    A(i,i-1) = h(i-1);
    A(i,i) = 2*(h(i) + h(i-1));
    A(i,i+1) = h(i);
    r(i) = -6*(p(i) - p(i-1))/h(i-1) + 6*(p(i+1) - p(i))/h(i);
end
%% C. Aufloesen des Gleichungssystems
%     (...)
%
     ddot_p = A\r;
%% D. Berechnung der Spline Koeffizienten mittels der Wegpunkte(ableitungen)
     for i = 1:N_I-1
         a(i) = (ddot_p(i+1) - ddot_p(i))/(6*h(i));
         b(i) = ddot_p(i)/2;
         c(i) = (p(i+1) - p(i))/h(i) - h(i)*(ddot_p(i+1) + 2*ddot_p(i))/6;
         d(i) = p(i);
     end
%% E.Initialisierung der Trajektorie
     N_T_I = floor(sum(h)/delta_T) + 1;
     w = zeros(1,N_T_I);
     dot_w = zeros(1,N_T_I);
     ddot_w = zeros(1,N_T_I);
%
%% F. Schleife ueber alle Splines zur Berechnung der Trajektorie
%     (...)
%
for i = 1:N_I-1
    T_I = 0:delta_T:h(i);
    n = length(T_I);
    n0 = (i-1)*(n-1) + 1; %Anfang jedes Zeitabschnitts
    n1 = i*(n-1) + 1; %Ende jedes Zeitabschnitts
    w(n0:n1) = a(i)*T_I.^3 + b(i)*T_I.^2 + c(i)*T_I + d(i);
    dot_w(n0:n1) = 3*a(i)*T_I.^2 + 2*b(i)*T_I + c(i);
    ddot_w(n0:n1) = 6*a(i)*T_I + 2*b(i);
end

end
