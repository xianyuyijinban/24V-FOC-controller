initScript = 'D:/matlab/minimal_foc_init.m';
modelPath = 'D:/matlab/minimal_foc_controller_task8_official_pmsm.slx';
modelName = 'minimal_foc_controller_task8_official_pmsm';
plantPath = [modelName '/plant'];

run(initScript);
load_system(modelPath);

plantPorts = get_param(plantPath, 'Ports');
feedbackPorts = get_param([modelName '/feedback'], 'Ports');
bridgePorts = get_param([modelName '/avg inverter bridge'], 'Ports');
positionLoopPorts = get_param([modelName '/position loop'], 'Ports');
speedLoopPorts = get_param([modelName '/speed loop'], 'Ports');
currentLoopPorts = get_param([modelName '/current loop'], 'Ports');

fprintf('TASK8_PLANT_IN=%d OUT=%d\n', plantPorts(1), plantPorts(2));
fprintf('TASK8_FEEDBACK_IN=%d OUT=%d\n', feedbackPorts(1), feedbackPorts(2));
fprintf('TASK8_BRIDGE_IN=%d OUT=%d\n', bridgePorts(1), bridgePorts(2));
fprintf('TASK8_POSITION_LOOP_IN=%d OUT=%d\n', positionLoopPorts(1), positionLoopPorts(2));
fprintf('TASK8_SPEED_LOOP_IN=%d OUT=%d\n', speedLoopPorts(1), speedLoopPorts(2));
fprintf('TASK8_CURRENT_LOOP_IN=%d OUT=%d\n', currentLoopPorts(1), currentLoopPorts(2));

assert(plantPorts(1) == 4 && plantPorts(2) == 5, ...
    'Task 8 plant ports are incorrect.');
assert(feedbackPorts(1) == 6 && feedbackPorts(2) == 7, ...
    'Task 8 feedback ports are incorrect.');
assert(bridgePorts(1) == 5 && bridgePorts(2) == 3, ...
    'Task 8 average bridge ports are incorrect.');
assert(positionLoopPorts(1) == 4 && positionLoopPorts(2) == 1, ...
    'Task 8 position loop ports are incorrect.');
assert(speedLoopPorts(1) == 4 && speedLoopPorts(2) == 1, ...
    'Task 8 speed loop ports are incorrect.');
assert(currentLoopPorts(1) == 6 && currentLoopPorts(2) == 2, ...
    'Task 8 current loop ports are incorrect.');

assert(~isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Kp_pos')), ...
    'Task 8 top level is missing Kp_pos.');
assert(~isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Ki_pos')), ...
    'Task 8 top level is missing Ki_pos.');
assert(~isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Kp_speed')), ...
    'Task 8 top level is missing Kp_speed.');
assert(~isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Ki_speed')), ...
    'Task 8 top level is missing Ki_speed.');
assert(~isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Kp_iq')), ...
    'Task 8 top level is missing Kp_iq.');
assert(~isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Ki_iq')), ...
    'Task 8 top level is missing Ki_iq.');

plantBlocks = find_system(plantPath, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'Block');
pmsmBlocks = find_official_pmsm_blocks(plantBlocks);
assert(~isempty(pmsmBlocks), ...
    'Task 8 plant does not contain an official PMSM block.');
assert(~isempty(find_system(plantPath, 'SearchDepth', 1, 'Name', 'info_bus_selector')), ...
    'Task 8 plant does not contain the info-bus selector.');

phPos = get_param([modelName '/position loop'], 'PortHandles');
phSpeed = get_param([modelName '/speed loop'], 'PortHandles');
phCurrent = get_param([modelName '/current loop'], 'PortHandles');

assert(strcmp(get_line_source_parent(phPos.Inport(3)), [modelName '/Kp_pos']), ...
    'Position loop Kp is not fed by Kp_pos.');
assert(strcmp(get_line_source_parent(phPos.Inport(4)), [modelName '/Ki_pos']), ...
    'Position loop Ki is not fed by Ki_pos.');
assert(strcmp(get_line_source_parent(phSpeed.Inport(3)), [modelName '/Kp_speed']), ...
    'Speed loop Kp is not fed by Kp_speed.');
assert(strcmp(get_line_source_parent(phSpeed.Inport(4)), [modelName '/Ki_speed']), ...
    'Speed loop Ki is not fed by Ki_speed.');
assert(strcmp(get_line_source_parent(phCurrent.Inport(5)), [modelName '/Kp_iq']), ...
    'Current loop Kp is not fed by Kp_iq.');
assert(strcmp(get_line_source_parent(phCurrent.Inport(6)), [modelName '/Ki_iq']), ...
    'Current loop Ki is not fed by Ki_iq.');

src3 = get_line_source_parent(phCurrent.Inport(3));
src4 = get_line_source_parent(phCurrent.Inport(4));

assert(strcmp(src3, [modelName '/feedback']), ...
    'Current loop id_meas is not fed by feedback.');
assert(strcmp(src4, [modelName '/feedback']), ...
    'Current loop iq_meas is not fed by feedback.');

simOut = sim(modelName, 'StopTime', '0.2', 'ReturnWorkspaceOutputs', 'on');

assert(any(strcmp(simOut.who, 'phase_currents_log')), ...
    'phase_currents_log was not created by the simulation.');

phaseData = simOut.get('phase_currents_log');
if size(phaseData, 2) >= 4
    phaseData = phaseData(:, end-2:end);
elseif size(phaseData, 2) ~= 3
    error('Unexpected phase_currents_log shape: %dx%d', size(phaseData, 1), size(phaseData, 2));
end

ia = phaseData(:, 1);
ib = phaseData(:, 2);
ic = phaseData(:, 3);

iaRange = max(ia) - min(ia);
ibRange = max(ib) - min(ib);
icRange = max(ic) - min(ic);
phaseDiff = max(abs(ia - ib));
sumResidual = max(abs(ia + ib + ic));

fprintf('IA range=%g min=%g max=%g\n', iaRange, min(ia), max(ia));
fprintf('IB range=%g min=%g max=%g\n', ibRange, min(ib), max(ib));
fprintf('IC range=%g min=%g max=%g\n', icRange, min(ic), max(ic));
fprintf('PHASE_DIFF_MAX=%g\n', phaseDiff);
fprintf('SUM_RESIDUAL_MAX=%g\n', sumResidual);

assert(iaRange > 1e-4, 'ia is too flat.');
assert(ibRange > 1e-4, 'ib is too flat.');
assert(icRange > 1e-4, 'ic is too flat.');
assert(phaseDiff > 1e-4, 'Phase currents are not meaningfully different.');
assert(sumResidual < 1e-2, 'ia + ib + ic residual is too large.');

disp('SIM_OK_TASK8');

close_system(modelName, 0);

function matches = find_official_pmsm_blocks(blocks)
matches = {};
for i = 1:numel(blocks)
    params = safe_get_param(blocks{i}, 'DialogParameters');
    if ~isstruct(params)
        continue;
    end
    if isfield(params, 'port_config') && isfield(params, 'sim_type') && ...
            isfield(params, 'Rs') && isfield(params, 'P')
        matches{end+1} = blocks{i}; %#ok<AGROW>
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

function src = get_line_source_parent(inportHandle)
lineHandle = get_param(inportHandle, 'Line');
srcPort = get_param(lineHandle, 'SrcPortHandle');
src = getfullname(get_param(srcPort, 'Parent'));
end
