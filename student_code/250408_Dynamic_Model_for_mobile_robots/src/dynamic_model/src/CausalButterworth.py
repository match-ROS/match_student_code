import numpy as np
from scipy.signal import butter

class CausalButterworth:
    """
    Implements a causal (forward-only) Butterworth filter as an IIR filter.
    
    Usage:
        # 1) Create a filter instance:
        bfilt = CausalButterworth(order=2, cutoff_hz=10.0, fs=100.0, btype='low')
        
        # 2a) Filter an entire signal at once:
        filtered_signal = bfilt.filter_signal(raw_signal)
        
        # 2b) Real-time filtering (sample-by-sample):
        #     (Assume you have a loop that gets new_data continuously)
        for sample in incoming_data:
            y = bfilt.process_sample(sample)
            # y is the new filtered output
    """
    
    def __init__(self, order=2, cutoff_hz=1.0, fs=100.0, btype='low'):
        """
        Parameters
        ----------
        order : int
            Filter order (e.g., 2, 4, etc.)
        cutoff_hz : float
            Cutoff frequency in Hz.
        fs : float
            Sampling rate in Hz.
        btype : str
            Filter type ('low', 'high', 'bandpass', 'bandstop').
            
        Notes
        -----
        - By default, this initializes a low-pass filter of specified order 
          with a single cutoff frequency (cutoff_hz).
        - If you need a bandpass or bandstop, pass a 2-element list as cutoff_hz
          like cutoff_hz = [low_cut, high_cut].
        """
        self.order = order
        self.fs = fs
        self.btype = btype
        
        # Normalize frequency (Nyquist = fs/2)
        # -> For bandpass/bandstop, cutoff_hz can be [low, high]
        nyq = 0.5 * fs
        if isinstance(cutoff_hz, (list, tuple, np.ndarray)):
            Wn = [c / nyq for c in cutoff_hz]
        else:
            Wn = cutoff_hz / nyq
        
        # Use scipy to get IIR coefficients (b, a)
        b, a = butter(order, Wn, btype=btype, analog=False, output='ba')
        
        # Make sure a[0] is 1.0 for simpler difference equation
        # (scipy usually provides that, but let's normalize if needed)
        if a[0] != 1.0:
            b = b / a[0]
            a = a / a[0]
        
        self.b = b
        self.a = a
        
        # Filter state for real-time sample-by-sample processing
        # We'll store up to max(len(a), len(b)) - 1 past samples.
        # If filter is order=N, then b and a each have length N+1.
        self.numerator_order   = len(b) - 1  # same as 'order', but let's keep it explicit
        self.denominator_order = len(a) - 1
        max_order = max(self.numerator_order, self.denominator_order)
        
        # Past inputs (x) and outputs (y) for the difference equation
        self.x_history = np.zeros(max_order)  # past x
        self.y_history = np.zeros(max_order)  # past y
        
    def reset_state(self):
        """
        Reset the filter's past input/output memory.
        Call this if you want to restart filtering.
        """
        self.x_history[:] = 0.0
        self.y_history[:] = 0.0

    def process_sample(self, x_new):
        """
        Process a single new sample in real time (causal).
        Returns the filtered output y_new.
        
        The difference equation is:
            y[n] = b0*x[n] + b1*x[n-1] + ... - a1*y[n-1] - ...
        
        We do not apply any forward-backward pass, so it's purely causal 
        (no zero-phase filtering).
        """
        # b[0]*x[n] + b[1]*x[n-1] + ... + b[M]*x[n-M]
        # - a[1]*y[n-1] - ... - a[M]*y[n-M]
        
        # length of b, a => M+1
        # push the new input into x_history, shift old samples
        # x_history[0] is the most recent in this scheme or the oldest?
        # We'll store them so that x_history[0] is x[n-1], x_history[1] = x[n-2], etc.
        
        # form the sum
        y_new = self.b[0] * x_new
        
        # numerator part
        for i in range(self.numerator_order):
            y_new += self.b[i+1] * self.x_history[i]
        # denominator part
        for i in range(self.denominator_order):
            y_new -= self.a[i+1] * self.y_history[i]
        
        # shift the history
        # x[n] becomes x_history[0], the old x_history[0] -> x_history[1], etc.
        self.x_history = np.roll(self.x_history, 1)
        self.y_history = np.roll(self.y_history, 1)
        
        # store the new input and output at the front
        self.x_history[0] = x_new
        self.y_history[0] = y_new
        
        return y_new
    
    def filter_signal(self, x):
        """
        Apply the filter to an entire 1D numpy array x in a causal, forward-only pass.
        Returns the filtered array y of the same shape.
        """
        x = np.asarray(x, dtype=float)
        y = np.zeros_like(x)
        
        # Reset state so consecutive calls produce the same result
        # from the start for each new signal.
        self.reset_state()
        
        for n in range(len(x)):
            y[n] = self.process_sample(x[n])
        
        return y
