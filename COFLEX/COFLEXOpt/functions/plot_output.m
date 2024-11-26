function plot_output(output, windspeed_vector, turbine)

rpm2rads = pi/30;
rads2rpm = 1/rpm2rads;

set(groot, 'DefaultAxesFontName', 'Times New Roman');
set(groot, 'DefaultTextFontName', 'Times New Roman');

% Create a new figure
figure;

% First subplot (INPUTS)
subplot(4, 1, 1);
title('INPUTS')

yyaxis left;
plot(windspeed_vector, output.omega_values / (2 * pi) * 60, '-');
ylabel('\omega (RPM)');
ylim([0.95 * turbine.wr_min * rads2rpm, 1.05 * turbine.wr_max * rads2rpm]);
yyaxis right;
plot(windspeed_vector, rad2deg(output.pitch_values), '-');
ylabel('\beta (deg)');
ylim([0.95 * rad2deg(turbine.pitch_min), 1.05 * rad2deg(turbine.pitch_max)]);
xlim([turbine.ws_in turbine.ws_out]);
grid on;

% Second subplot (OUTPUTS)
subplot(4, 1, 2);
title('OUTPUTS')

yyaxis left;
plot(windspeed_vector, output.P_values * turbine.etag / 10^3, '-');
ylabel('Gen. Power (MW)');
ylim([0, 1.05 * turbine.P_rated / 10^6]);
yyaxis right;
plot(windspeed_vector, output.T_values / 10^3, '-');
ylabel('Thrust (MN)');
xlim([turbine.ws_in turbine.ws_out]);
grid on;

% Third subplot
subplot(4, 1, 3);

yyaxis left;
plot(windspeed_vector, (output.omega_values ./ windspeed_vector') * turbine.R, '-');
ylabel('TSR (-)');
yyaxis right;
plot(windspeed_vector, output.GenQ_values / 10^3, '-');
ylabel('Gen. Torque (MNm)');
ylim([0, 1.05 * turbine.Tg_max / 10^6]);
xlim([turbine.ws_in turbine.ws_out]);
grid on;

% Fourth subplot
subplot(4, 1, 4);

plot(windspeed_vector, output.OOPTip_values, '-');
ylabel('OOP Tip Disp. (m)');
xlabel('V (m/s)');
xlim([turbine.ws_in turbine.ws_out]);
grid on;


formatSpec = 'Set points and outputs';
str = sprintf(formatSpec);
sgt = sgtitle(str,'Color','black');
sgt.FontWeight = 'bold';
sgt.FontSize = 13;

end
