function Hd = iir_60hz
%UNTITLED Returns a discrete-time filter object.

% MATLAB Code
% Generated by MATLAB(R) 9.13 and Signal Processing Toolbox 9.1.
% Generated on: 01-Nov-2022 14:36:58

% Chebyshev Type II Bandstop filter designed using FDESIGN.BANDSTOP.

% All frequency values are in Hz.
Fs = 360;  % Sampling Frequency
% 
% Fpass1 = 59;          % First Passband Frequency
% Fstop1 = 59.6;        % First Stopband Frequency
% Fstop2 = 60.5;        % Second Stopband Frequency
% Fpass2 = 61;          % Second Passband Frequency

Fpass1 = 55;          % First Passband Frequency
Fstop1 = 57;        % First Stopband Frequency
Fstop2 = 63;        % Second Stopband Frequency
Fpass2 = 65;          % Second Passband Frequency
Apass1 = 0.5;         % First Passband Ripple (dB)
Astop  = 60;          % Stopband Attenuation (dB)
Apass2 = 1;           % Second Passband Ripple (dB)
match  = 'stopband';  % Band to match exactly

% Construct an FDESIGN object and call its CHEBY2 method.
h  = fdesign.bandstop(Fpass1, Fstop1, Fstop2, Fpass2, Apass1, Astop, ...
                      Apass2, Fs);
Hd = design(h, 'cheby2', 'MatchExactly', match);

[b,a]=sos2tf(Hd.sosMatrix, Hd.ScaleValues);

fileID = fopen("a_coef_60.bin", 'wb');
fwrite(fileID, a, 'double');
fclose(fileID);

fileID = fopen("b_coef_60.bin", 'wb');
fwrite(fileID, b, 'double');
fclose(fileID);

freqz(b, a, 'whole', 360);

% [EOF]