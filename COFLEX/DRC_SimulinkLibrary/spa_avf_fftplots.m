function spa_avf_fftplots(u, y, dt, varargin)

avgWindow = 10;

if isequal(length(u), length(y))
    determineTF = true;
    U = u;
    Y = y;
    DT = dt;
else
    determineTF = false;
    U = u;
    DT = y;
end

if ~determineTF
    if exist('dt', 'var')
        avgWindow = dt;   
    end 
else
    if ~isempty(varargin{1})
        avgWindow = varargin{1};
    end
end

if determineTF
    [G, w, ~, mSuu, mSyy] = spa_avf(U, Y, DT, avgWindow, [], [], 'hamming');
    
    figure(100)
    [magG, phaseG] =  magphase(G);
    mbode(magG, phaseG, w)
    title('Transfer function estimate $G = Y/U$', 'Interpreter', 'latex')
    
    figure(101)
    mbode(abs(mSyy), angle(mSyy), w)
    title('Spectrum of output signal Y', 'Interpreter', 'latex')
else
    [~, w, ~, mSuu, ~] = spa_avf(U, U, DT, avgWindow, [], [], 'hamming');
    G = [];
end

figure(102)
[magmSuu, phasemSuu] =  magphase(mSuu);
mbode(magmSuu, phasemSuu, w)
title('Spectrum of input signal U', 'Interpreter', 'latex')
