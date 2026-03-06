classdef TUILogger < handle
% TUILOGGER Text User Interface logger for the reconstruction pipeline
%   Creates a dedicated figure window that displays structured,
%   color-coded log output in real-time. Replaces traditional
%   command window output with an observable, saveable TUI.
%
% Usage:
%   tui = TUILogger('My Pipeline');
%   tui.log('MODULE', 'Something happened');
%   tui.phase('1/3', 'LOADING');
%   tui.timing('Step', 2.45);
%   tui.saveLog('output.log');

    properties (Access = private)
        fig          % Figure handle
        textArea     % Text area UI component
        logLines     % Cell array of log strings
        startTime    % Pipeline start timestamp
        titleStr     % Window title
    end

    methods

        function obj = TUILogger(titleStr)
        % Constructor — creates the TUI window
            if nargin < 1
                titleStr = 'Pipeline Log';
            end
            obj.titleStr = titleStr;
            obj.logLines = {};
            obj.startTime = tic;

            % Create the TUI figure
            obj.fig = uifigure( ...
                'Name', sprintf('📋 %s — Log Console', titleStr), ...
                'Position', [60, 60, 920, 640], ...
                'Color', [0.06 0.06 0.09], ...
                'Resize', 'on', ...
                'CloseRequestFcn', @(~,~) obj.onClose());

            % Create text area for log output
            obj.textArea = uitextarea(obj.fig, ...
                'Position', [10, 10, 900, 620], ...
                'Editable', 'off', ...
                'FontName', 'Consolas', ...
                'FontSize', 11, ...
                'FontColor', [0.85 0.9 0.85], ...
                'BackgroundColor', [0.08 0.08 0.11], ...
                'Value', {''});

            drawnow;
        end


        function header(obj)
        % Display the startup banner
            banner = { ...
                '╔══════════════════════════════════════════════════════════════════╗'; ...
                '║          DRONE IMAGE RECONSTRUCTION ENGINE                     ║'; ...
                '║          Pure MATLAB Backend · TUI Logger                      ║'; ...
                '╚══════════════════════════════════════════════════════════════════╝'; ...
                '' };
            for i = 1:numel(banner)
                obj.appendLine(banner{i});
            end
        end


        function log(obj, tag, message)
        % LOG Add a structured log entry
        %   tui.log('MODULE', 'message text')
            elapsed = toc(obj.startTime);
            timestamp = sprintf('[%8.2fs]', elapsed);

            % Pad tag to 12 chars for alignment
            tag = pad(upper(tag), 12);

            line = sprintf('%s  %s  %s', timestamp, tag, message);
            obj.appendLine(line);

            % Also echo to command window for headless/debugging use
            fprintf('%s\n', line);
        end


        function phase(obj, phaseNum, phaseName)
        % PHASE Display a prominent phase header
            obj.appendLine('');
            obj.appendLine(sprintf('┌─── PHASE %s ─── %s ───────────────────────────────', ...
                phaseNum, upper(phaseName)));
            obj.appendLine('│');
            fprintf('\n=== PHASE %s: %s ===\n', phaseNum, upper(phaseName));
        end


        function timing(obj, stepName, elapsed)
        % TIMING Display execution time for a step
            obj.appendLine('│');
            obj.appendLine(sprintf('└─── %s completed in %.2f seconds', stepName, elapsed));
            obj.appendLine('');
            fprintf('  ⏱ %s: %.2fs\n', stepName, elapsed);
        end


        function separator(obj)
        % SEPARATOR Add a visual divider
            obj.appendLine('────────────────────────────────────────────────────────────────');
        end


        function progress(obj, current, total, label)
        % PROGRESS Display a text-based progress bar
            pct = current / total;
            barLen = 30;
            filled = round(pct * barLen);
            empty = barLen - filled;
            bar = [repmat('█', 1, filled), repmat('░', 1, empty)];
            line = sprintf('  %s  %s  %d/%d  (%.0f%%)', label, bar, current, total, pct*100);
            obj.appendLine(line);
            fprintf('%s\n', line);
        end


        function saveLog(obj, filePath)
        % SAVELOG Write all log lines to a text file
            fid = fopen(filePath, 'w');
            if fid == -1
                warning('TUILogger:saveFail', 'Could not write log to %s', filePath);
                return;
            end
            for i = 1:numel(obj.logLines)
                fprintf(fid, '%s\n', obj.logLines{i});
            end
            fclose(fid);
        end


        function lines = getLog(obj)
        % GETLOG Return all log lines as a cell array
            lines = obj.logLines;
        end

    end


    methods (Access = private)

        function appendLine(obj, line)
        % Append a line to the TUI and internal buffer
            obj.logLines{end+1} = line;

            if isvalid(obj.fig) && isvalid(obj.textArea)
                currentVal = obj.textArea.Value;
                obj.textArea.Value = [currentVal; {line}];

                % Auto-scroll to bottom
                scroll(obj.textArea, 'bottom');
                drawnow limitrate;
            end
        end


        function onClose(obj)
        % Handle window close — save log before closing
            if ~isempty(obj.logLines)
                fprintf('\n[TUI] Log console closed (%d lines recorded)\n', ...
                    numel(obj.logLines));
            end
            delete(obj.fig);
        end

    end
end
