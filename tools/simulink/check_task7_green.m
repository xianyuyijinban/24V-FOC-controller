initScript = 'D:/matlab/minimal_foc_init.m';
modelPath = 'D:/matlab/minimal_foc_controller_task7_abcplant.slx';
modelName = 'minimal_foc_controller_task7_abcplant';

run(initScript);
load_system(modelPath);

plantPorts = get_param([modelName '/plant'], 'Ports');
feedbackPorts = get_param([modelName '/feedback'], 'Ports');
bridgePorts = get_param([modelName '/avg inverter bridge'], 'Ports');

fprintf('TASK7_PLANT_IN=%d OUT=%d\n', plantPorts(1), plantPorts(2));
fprintf('TASK7_FEEDBACK_IN=%d OUT=%d\n', feedbackPorts(1), feedbackPorts(2));
fprintf('TASK7_BRIDGE_IN=%d OUT=%d\n', bridgePorts(1), bridgePorts(2));

assert(plantPorts(1) == 3 && plantPorts(2) == 5, ...
    'Task 7 plant ports are incorrect.');
assert(feedbackPorts(1) == 6 && feedbackPorts(2) == 7, ...
    'Task 7 feedback ports are incorrect.');
assert(bridgePorts(1) == 5 && bridgePorts(2) == 2, ...
    'Task 7 average bridge ports are incorrect.');

ph = get_param([modelName '/current loop'], 'PortHandles');

src3 = get_line_source_parent(ph.Inport(3));
src4 = get_line_source_parent(ph.Inport(4));

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

disp('SIM_OK_TASK7');

close_system(modelName, 0);

function src = get_line_source_parent(inportHandle)
lineHandle = get_param(inportHandle, 'Line');
srcPort = get_param(lineHandle, 'SrcPortHandle');
src = getfullname(get_param(srcPort, 'Parent'));
end
