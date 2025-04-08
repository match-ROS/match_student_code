import numpy as np

class savgol_filter():
    def __init__(self, window_size, poly_order):
        self.window_size = window_size
        self.poly_order = poly_order
        
    def savgol_coefficients_one_sided(self):
        """
        Berechnet Savitzky-Golay-Koeffizienten für ein *einseitiges* Fenster
        (d.h. kausale Sicht nach hinten) und Polynomapproximation 0. Ordnung (Glättung).
        Der gefilterte Wert bezieht sich auf den letzten Punkt im Fenster.

        Parameters
        ----------
        window_size : int
            Anzahl der Datenpunkte im Fenster (>= poly_order + 1).
        poly_order : int
            Grad des zu approximierenden Polynoms (>= 0).

        Returns
        -------
        coeffs : np.ndarray
            Array der Länge window_size mit den Faltungsgewichten
            c[i], so dass gefilterter Wert = sum_{i=0..W-1}(c[i]*y[i]).
            Dabei entspricht i=0 dem ältesten Datenpunkt,
            i=window_size-1 dem aktuellsten.
        """
        if self.window_size <= self.poly_order:
            raise ValueError("window_size muss größer als poly_order sein.")

        # Indizes i = 0..(W-1)
        i = np.arange(self.window_size)

        # G-Matrix: Zeile i enthält [1, i, i^2, ..., i^poly_order]
        G = np.vstack([i**k for k in range(self.poly_order+1)]).T  # shape = (window_size, poly_order+1)

        # desired-Vektor (dient zur Extraktion des Polynomauswerts bei i=window_size-1)
        # => wir wollen P(W-1). Dies lässt sich "abfragen", indem in desired an Position (W-1) eine 1 steht
        # und sonst 0 (delta). Dann a = pinv(G)*desired => a_k => Polynomkoeffs,
        # P(W-1) = sum_{i=0..W-1} c_i y(i) => c = ...
        desired = np.zeros(self.window_size)
        desired[-1] = 1.0  # 1 an der Stelle W-1

        # pseudo-inverse: a = (G^+) * desired
        # wir wollen aber c: c = G^+^T e, je nach Konvention. Einfacher Weg:
        # c = G^+ * desired gibt uns a, und c_i = sum_k a_k*(i^k)? Besser: wir nutzen G^+ direkt:
        # 
        # Standard Savitzky-Golay: c = row( (G^T G)^-1 G^T ) , so dass sum_i c[i]*y(i) = P(W-1).
        # Kurz: wir machen pinv(G) * desired => a => werten Polynom an i=0..(W-1) aus?
        # 
        # Besser: c = pinv(G).T @ desired. Schauen wir uns an:
        # 
        # Wir wollen c[i] = Gewichte an y(i). 
        # c[i] = partial derivative of P(W-1) w.r.t. y(i). 
        # 
        # Durch Test: 
        # A = pinv(G) # shape=(p+1, W)
        # => a = A @ y
        # => P(W-1) = [ (W-1)^0, (W-1)^1, ..., (W-1)^p ]^T * a 
        # 
        # wir konstruieren "desired" so, dass wir =1 an W-1 => a = A @ desired 
        # => a[k] => Polynomkoeff, so dass P(W-1)=1. 
        # => Dann c = G @ a ??? 
        # 
        # Evtl. einfacher: Wir lösen c direkt: c = M^T, wir machen es methodisch:
        # 
        # c = G * ( (G^T G)^-1 G^T ) e
        #   =  I * e (??)
        # 
        # In der Praxis: Wir können's so machen:
        #   A = np.linalg.pinv(G) (p+1 x W)
        #   a = A @ desired        (p+1)
        #   c = G @ a              (W)
        # => c ist dann das, was wir wollen: c[i] = Wert, der mit y[i] multipliziert wird
        # 
        # => P(W-1) = sum_i c[i]* y[i].
        # 
        A = np.linalg.pinv(G)
        a = A @ desired  # Polynomkoeffizienten
        c = G @ a        # Dies ergibt ein Vektor c[i] (Länge W)

        return c

    def filter(self, data):
        """
        Wendet ein einseitiges (kausales) Savitzky-Golay-Filter
        auf ein 1D-Array 'data' an. Der gefilterte Wert an Index t nutzt
        (window_size) Punkte: [t-window_size+1 .. t].

        Parameters
        ----------
        data : array_like
            1D-Eingangsdaten (z.B. numpy.ndarray).
        window_size : int
            Größe des einseitigen Fensters.
        poly_order : int
            Grad des Polynoms.

        Returns
        -------
        filtered_data : np.ndarray
            Gefiltertes Signal derselben Länge wie 'data'.
            Die ersten (window_size-1) Werte werden nur rudimentär
            gefiltert (da nicht genügend Vorlaufwerte da sind),
            oder bei kurzer Zeitreihe sollte man sie ggf. verwerfen.
        """
        n = len(data)
        data = np.asarray(data, dtype=float)

        # Filterkoeffizienten einmalig berechnen
        coeffs = self.savgol_coefficients_one_sided()
        
        # Ausgabearray
        filtered_data = np.zeros_like(data)

        # Um Randprobleme zu vermeiden, könnte man Padding vornehmen.
        # Hier machen wir es einfach: Für t < window_size-1 haben wir kein volles Fenster.
        # => Minimale Schleife: man kann z.B. "0-Padding" oder "extend" machen.
        # Wir entscheiden uns hier für extend mit dem erstverfügbaren Wert.

        # Erzeuge gepaddetes Array:
        pad_length = self.window_size - 1
        # vorn um pad_length erweitern (damit Index t immer mind. window_size-1 zur Verfügung hat)
        padded = np.concatenate([np.full(pad_length, data[0]), data])

        # Jetzt Faltung "manuell" oder in einer Schleife:
        for t in range(n):
            # Index im gepaddeten Array: offset um +t 
            segment = padded[t : t+self.window_size]  # window_size lang
            filtered_data[t] = np.sum(segment * coeffs)

        return filtered_data
