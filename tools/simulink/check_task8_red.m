modelPath = 'D:/matlab/minimal_foc_controller_task7_abcplant.slx';
modelName = 'minimal_foc_controller_task7_abcplant';
plantPath = [modelName '/plant'];

load_system(modelPath);

plantBlocks = find_system(plantPath, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'Block');
pmsmBlocks = find_official_pmsm_blocks(plantBlocks);
phaseVoltageBlocks = find_named_blocks(plantBlocks, {'phase_voltage_mux', 'Average-Value Inverter', 'Controlled Voltage Source (Three-Phase)'});
infoSelectors = find_named_blocks(plantBlocks, {'info_bus_selector'});

fprintf('TASK7_PLANT_BLOCKS=%d\n', numel(plantBlocks));
fprintf('TASK7_PMSM_BLOCKS=%d\n', numel(pmsmBlocks));
fprintf('TASK7_PHASE_SOURCE_BLOCKS=%d\n', numel(phaseVoltageBlocks));
fprintf('TASK7_INFO_SELECTOR_BLOCKS=%d\n', numel(infoSelectors));

assert(~isempty(pmsmBlocks), ...
    'RED CHECK: Task 7 plant does not yet contain an official Surface Mount PMSM block.');
assert(~isempty(phaseVoltageBlocks), ...
    'RED CHECK: Task 7 plant does not yet contain the Task 8 official phase-voltage path.');
assert(~isempty(infoSelectors), ...
    'RED CHECK: Task 7 plant does not yet contain the Task 8 official info-bus selector path.');

close_system(modelName, 0);

function matches = find_named_blocks(blocks, names)
matches = {};
for i = 1:numel(blocks)
    blkName = get_param(blocks{i}, 'Name');
    for j = 1:numel(names)
        if strcmp(blkName, names{j})
            matches{end+1} = blocks{i}; %#ok<AGROW>
            break;
        end
    end
end
end

function matches = find_official_pmsm_blocks(blocks)
matches = {};
for i = 1:numel(blocks)
    params = safe_get_param(blocks{i}, 'DialogParameters');
    if ~isstruct(params)
        continue;
    end
    if isfield(params, 'port_config') && isfield(params, 'sim_type') && ...
            isfield(params, 'Rs') && isfield(params, 'P')
        matches{end + 1} = blocks{i}; %#ok<AGROW>
    end
end
end

function value = safe_get_param(blockPath, paramName)
try
    value = get_param(blockPath, paramName);
catch
    value = [];
end
end
