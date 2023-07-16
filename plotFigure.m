function plotFigure(varargin)
numInputs = nargin;
YData1 = 0;
if numInputs == 7
    yTitle = varargin{1};
    XData1 = varargin{2};
    YData1 = varargin{3};
    YData2 = varargin{4};
    YData3 = varargin{5};
    YData4 = varargin{6};
    ax = varargin{7};
elseif numInputs == 6
    yTitle = varargin{1};
    XData1 = varargin{2};
    YData2 = varargin{3};
    YData3 = varargin{4};
    YData4 = varargin{5};
    ax = varargin{6};
end

% Create axes for the specified subplot index

% Create line for each data series
if YData1 ~= 0
    line(XData1, YData1, 'DisplayName', 'Reference', 'Tag', 'DisplayLine1', 'Parent', ax, 'LineWidth', 1, 'Color', [0 0 0]);
end
line(XData1, YData2, 'DisplayName', 'DTLANNC', 'Tag', 'DisplayLine2', 'Parent', ax, 'LineWidth', 1, 'Color', [1 0 0]);
line(XData1, YData3, 'DisplayName', 'TLMBC', 'Tag', 'DisplayLine3', 'Parent', ax, 'LineWidth', 1, 'Color', [0.466666666666667 0.674509803921569 0.188235294117647]);
line(XData1, YData4, 'DisplayName', 'NNSMC', 'Tag', 'DisplayLine4', 'Parent', ax, 'LineWidth', 1, 'Color', [0 0.447058823529412 0.741176470588235]);

% Set axis labels
xlabel(ax, 'Time (s)', 'FontName', 'Times New Roman');
ylabel(ax, yTitle, 'FontName', 'Times New Roman');

% Set remaining axis properties
set(ax, 'ClippingStyle', 'rectangle', 'FontName', 'Times New Roman', 'GridAlpha', 0.4, 'GridColor', [0 0 0], 'TickLabelInterpreter', 'none', 'XGrid', 'on', 'YGrid', 'on');
