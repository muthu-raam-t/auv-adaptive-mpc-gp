function force_white_figure(figs)
%FORCE_WHITE_FIGURE Make figure(s) render with a white background and black
%   axes, text, grid and legend, regardless of MATLAB's dark theme.
%
%   force_white_figure(fig)   % one figure handle (or a vector of handles)
%   force_white_figure        % current figure
%
%   Call it AFTER the plotting commands of a figure. It sets the light theme
%   (R2025a and newer; silently skipped on older releases) and then
%   explicitly overrides every colour that the dark theme would otherwise
%   leave dark/light, so exported PNGs are always black-on-white.

if nargin < 1, figs = gcf; end

ink  = [0 0 0];          % axis lines, tick labels, titles
grid_ = [0 0 0];         % grid colour (alpha keeps it faint)

for fig = figs(:)'
    try, fig.Theme = 'light'; catch, end       % R2025a+ only
    set(fig, 'Color', 'white');

    % ---- axes (covers every subplot) ------------------------------------
    for ax = findall(fig, 'Type', 'axes')'
        set(ax, 'Color', 'white', ...
                'XColor', ink, 'YColor', ink, ...
                'GridColor', grid_, 'GridAlpha', 0.15, ...
                'MinorGridColor', grid_, 'MinorGridAlpha', 0.10);
        try, set(ax, 'ZColor', ink); catch, end
        set(ax.Title,  'Color', ink);
        set(ax.XLabel, 'Color', ink);
        set(ax.YLabel, 'Color', ink);
        try, set(ax.ZLabel, 'Color', ink); catch, end
    end

    % ---- legends ----------------------------------------------------------
    for lg = findall(fig, 'Type', 'legend')'
        set(lg, 'TextColor', ink, 'Color', 'white', 'EdgeColor', [0.15 0.15 0.15]);
    end

    % ---- colour bars, if any ---------------------------------------------
    for cb = findall(fig, 'Type', 'colorbar')'
        set(cb, 'Color', ink);
    end
end

drawnow;
end
