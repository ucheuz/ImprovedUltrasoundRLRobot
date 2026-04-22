% Dual-arm training curves in a 2×3 layout (five metric panels + one auxiliary),
% aligned with the episode-indexed figure style used for dual-arm analysis.
%
% Expected CSVs under matlab_data/ (from training PlottingCallback-style export):
%   rewards.csv, final_distances.csv, success_rates.csv, episode_lengths.csv
% Optional (if your callback writes them):
%   change_distances.csv, final_quaternion_diff.csv, change_quaternion_diff.csv
%
% Run from the Two-Arm Robot folder, or call this script from elsewhere (paths are
% resolved relative to this file).

clear; clc; close all;

rootDir = fileparts(mfilename('fullpath'));
if isempty(rootDir)
    rootDir = pwd;
end
dataDir = fullfile(rootDir, 'matlab_data');

rewards = read_series_csv(fullfile(dataDir, 'rewards.csv'));
finalD = read_series_csv(fullfile(dataDir, 'final_distances.csv'));
changeD = read_series_csv(fullfile(dataDir, 'change_distances.csv'));
finalQ = read_series_csv(fullfile(dataDir, 'final_quaternion_diff.csv'));
changeQ = read_series_csv(fullfile(dataDir, 'change_quaternion_diff.csv'));
successR = read_series_csv(fullfile(dataDir, 'success_rates.csv'));
epLen = read_series_csv(fullfile(dataDir, 'episode_lengths.csv'));

lens = [];
if ~isempty(rewards), lens = [lens, numel(rewards)]; end
if ~isempty(finalD), lens = [lens, numel(finalD)]; end
if ~isempty(changeD), lens = [lens, numel(changeD)]; end
if ~isempty(finalQ), lens = [lens, numel(finalQ)]; end
if ~isempty(changeQ), lens = [lens, numel(changeQ)]; end
if ~isempty(successR), lens = [lens, numel(successR)]; end
if ~isempty(epLen), lens = [lens, numel(epLen)]; end

if isempty(lens)
    error('No readable series found in %s (expected at least rewards.csv).', dataDir);
end

nEp = min(lens);
episodes = (1:nEp)';

if ~isempty(rewards), rewards = rewards(1:nEp); else, rewards = nan(nEp, 1); end
if ~isempty(finalD), finalD = finalD(1:nEp); else, finalD = nan(nEp, 1); end
if ~isempty(changeD), changeD = changeD(1:nEp); else, changeD = nan(nEp, 1); end
if ~isempty(finalQ), finalQ = finalQ(1:nEp); else, finalQ = nan(nEp, 1); end
if ~isempty(changeQ), changeQ = changeQ(1:nEp); else, changeQ = nan(nEp, 1); end
if ~isempty(successR), successR = successR(1:nEp); else, successR = nan(nEp, 1); end
if ~isempty(epLen), epLen = epLen(1:nEp); else, epLen = nan(nEp, 1); end

lineColor = [0.15 0.35 0.75];
lineW = 0.9;

figure('Name', 'Dual-Arm R2: Training performance', 'Position', [80, 80, 1280, 720]);

subplot(2, 3, 1);
plot(episodes, rewards, 'Color', lineColor, 'LineWidth', lineW);
grid on;
xlabel('Number of episodes');
ylabel('Episode return');
title('Cumulative reward (per episode)');
xlim([1, nEp]);

subplot(2, 3, 2);
plot(episodes, finalD, 'Color', lineColor, 'LineWidth', lineW);
grid on;
xlabel('Number of episodes');
ylabel('Final Euclidean distance (m)');
title('Final Euclidean distance vs episode');
xlim([1, nEp]);

subplot(2, 3, 3);
if all(isnan(changeD))
    plot(episodes, nan(size(episodes)));
    title('Change in Euclidean distance (missing CSV)');
else
    plot(episodes, changeD, 'Color', lineColor, 'LineWidth', lineW);
    title('Change in Euclidean distance vs episode');
end
grid on;
xlabel('Number of episodes');
ylabel('Change in Euclidean distance (m)');
xlim([1, nEp]);

subplot(2, 3, 4);
if all(isnan(finalQ))
    plot(episodes, nan(size(episodes)));
    title('Final quaternion difference (missing CSV)');
else
    plot(episodes, finalQ, 'Color', lineColor, 'LineWidth', lineW);
    title('Final quaternion difference vs episode');
end
grid on;
xlabel('Number of episodes');
ylabel('Final quaternion difference (rad)');
xlim([1, nEp]);

subplot(2, 3, 5);
if all(isnan(changeQ))
    plot(episodes, nan(size(episodes)));
    title('Change in quaternion difference (missing CSV)');
else
    plot(episodes, changeQ, 'Color', lineColor, 'LineWidth', lineW);
    title('Change in quaternion difference vs episode');
end
grid on;
xlabel('Number of episodes');
ylabel('Change in quaternion difference (rad)');
xlim([1, nEp]);

subplot(2, 3, 6);
if ~all(isnan(successR))
    plot(episodes, successR, 'Color', [0.1 0.55 0.25], 'LineWidth', lineW);
    grid on;
    xlabel('Number of episodes');
    ylabel('Success (0-1)');
    title('100-episode success moving average');
    ylim([0, 1.05]);
    xlim([1, nEp]);
elseif ~all(isnan(epLen))
    plot(episodes, epLen, 'Color', [0.55 0.35 0.1], 'LineWidth', lineW);
    grid on;
    xlabel('Number of episodes');
    ylabel('Timesteps');
    title('Episode length');
    xlim([1, nEp]);
else
    axis off;
    text(0.5, 0.5, {'No success_rates.csv or'; 'episode_lengths.csv for this panel.'}, ...
        'HorizontalAlignment', 'center', 'FontSize', 11);
end

sgtitle('Dual-Arm R2: training curves (from matlab_data)', 'Interpreter', 'none');
set(gcf, 'Color', 'w');

function v = read_series_csv(fpath)
    v = [];
    if ~isfile(fpath)
        return;
    end
    v = readmatrix(fpath);
    v = v(:);
end
