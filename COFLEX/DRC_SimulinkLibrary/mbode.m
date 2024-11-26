function mbode(mag, phase, omega, varargin)
% mbode(mag, phase, omega, varargin) - Manual bode plot function
% for plotting Bode plots using frequency response data (mag, phase). 
% The providedFrequency data should be a one-dimensional vector with
% the same length as mag, phase.
%
% The argument varargin can take the following parameters (in this
% order):
% - PlotTitle (string), e.g., 'MassSpringDamper system'
% - LineStyle (string), e.g., '--'
% - LineWidth (float/integer), e.g., 2
% - LineColor (string), e.g., 'k' or 'yellow'
% - Hz or rad/s (string) --> 'rads' / 'Hz'
% - Plot phase (logical) --> true/false
%
% S.P. Mulders (Sebastiaan)
% Delft Center for Systems and Control (DCSC)
% The Netherlands, 2021

linestyle = '-';
linewidth = 1;
linecolor = 'k';
labelx = 'Frequency [rad/s]';
plotPhase = false;

nvarargin = length(varargin);
switch nvarargin
    case 0
        % Do nothing
    otherwise
        if ~isempty(varargin{1})
            plottitle = varargin{1};
        end
        if ~isempty(varargin{2})
            linestyle = varargin{2};
        end
        if ~isempty(varargin{3})
            linewidth = varargin{3};
        end
        if ~isempty(varargin{4})
            linecolor = varargin{4};
        end
        if strcmpi(varargin{5}, 'Hz')
            omega = omega/(2*pi);
            labelx = 'Frequency [Hz]';
        end
        if varargin{6}
            plotPhase = true;
        end
end

mag = squeeze(mag);
phase = squeeze(phase);
% phase = wrapTo360(phase);

if plotPhase
    ax1 = subplot(211);
end
semilogx(omega, mag, 'LineStyle', linestyle, 'LineWidth', linewidth, 'Color', linecolor)
hold on
xlim([omega(1), omega(end)])

if nvarargin > 0
    if ~isempty(varargin{1})
        title(plottitle, 'Interpreter', 'latex', 'FontSize', 12)
    end
end
ylabel('Magnitude [dB]', 'Interpreter', 'latex', 'FontSize', 11)
grid on
set(gca,'TickLabelInterpreter','latex');
if plotPhase
    ax2 = subplot(212);
    semilogx(omega, phase, 'LineStyle', linestyle, 'LineWidth', linewidth, 'Color', linecolor) %mod(phase, -180)
    hold on
    xlim([omega(1), omega(end)])
    ylabel('Phase [deg]', 'Interpreter', 'latex', 'FontSize', 11)
    grid on
    set(gca,'TickLabelInterpreter','latex');
    linkaxes([ax1 ax2], 'x') 
end
xlabel(labelx, 'Interpreter', 'latex', 'FontSize', 11)
end

