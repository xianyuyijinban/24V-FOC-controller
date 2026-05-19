src = 'D:/matlab/minimal_foc_controller_task7_abcplant.slx';
dst = 'D:/matlab/minimal_foc_controller_task8_official_pmsm.slx';
initScript = 'D:/matlab/minimal_foc_init.m';

if ~isfile(src)
    error('Source model not found: %s', src);
end

if ~isfile(initScript)
    error('Init script not found: %s', initScript);
end

run(initScript);
if ~exist('Kd_pos', 'var')
    Kd_pos = 0.10;
end
probe = probe_official_pmsm_block();

fprintf('TASK8_PMSM_LIB=%s\n', probe.Library);
fprintf('TASK8_PMSM_BLOCK=%s\n', probe.Path);
fprintf('TASK8_PMSM_PARAMS=%s\n', strjoin(probe.ParamNames, ', '));

[~, dstMdl] = fileparts(dst);
if bdIsLoaded(dstMdl)
    close_system(dstMdl, 0);
end

copyfile(src, dst, 'f');
load_system(dst);

[~, mdl] = fileparts(dst);
bridgeSys = [mdl '/avg inverter bridge'];
plantSys = [mdl '/plant'];

try
    ensure_top_level_pi_param_blocks(mdl);
    rebuild_position_loop([mdl '/position loop']);
    rebuild_speed_loop([mdl '/speed loop']);
    rebuild_current_loop([mdl '/current loop']);
    rebuild_avg_inverter_bridge_phase(bridgeSys);
    rebuild_plant_official_pmsm(plantSys, probe);
    ensure_phase_current_observation(mdl);
    rewire_top_level(mdl);
    layout_top_level(mdl);

    set_param(mdl, 'SimulationCommand', 'update');
    delete_dangling_lines(mdl);

    save_system(mdl, dst);
    close_system(mdl, 0);
catch ME
    if bdIsLoaded(mdl)
        close_system(mdl, 0);
    end
    rethrow(ME);
end

fprintf('Created official-PMSM-upgraded model: %s\n', dst);

function rebuild_avg_inverter_bridge_phase(sys)
delete_all_lines_in_system(sys);
delete_all_child_blocks(sys);

set_param(sys, 'Position', [500 86 688 172]);

add_block('simulink/Sources/In1', [sys '/duty_a'], ...
    'Position', [30 20 60 34], 'Port', '1');
add_block('simulink/Sources/In1', [sys '/duty_b'], ...
    'Position', [30 50 60 64], 'Port', '2');
add_block('simulink/Sources/In1', [sys '/duty_c'], ...
    'Position', [30 80 60 94], 'Port', '3');
add_block('simulink/Sources/In1', [sys '/theta_elec'], ...
    'Position', [30 110 60 124], 'Port', '4');
add_block('simulink/Sources/In1', [sys '/Vbus_fb'], ...
    'Position', [30 140 60 154], 'Port', '5');

    bridgeBlk = [sys '/svpwm_to_phase_voltages'];
add_block('simulink/User-Defined Functions/MATLAB Function', bridgeBlk, ...
    'Position', [120 24 355 165]);

add_block('simulink/Sinks/Out1', [sys '/va_cmd'], ...
    'Position', [430 40 460 54], 'Port', '1');
add_block('simulink/Sinks/Out1', [sys '/vb_cmd'], ...
    'Position', [430 80 460 94], 'Port', '2');
add_block('simulink/Sinks/Out1', [sys '/vc_cmd'], ...
    'Position', [430 120 460 134], 'Port', '3');

configure_bridge_matlab_function(bridgeBlk);

add_line(sys, 'duty_a/1',     'svpwm_to_phase_voltages/1', 'autorouting', 'on');
add_line(sys, 'duty_b/1',     'svpwm_to_phase_voltages/2', 'autorouting', 'on');
add_line(sys, 'duty_c/1',     'svpwm_to_phase_voltages/3', 'autorouting', 'on');
add_line(sys, 'theta_elec/1', 'svpwm_to_phase_voltages/4', 'autorouting', 'on');
add_line(sys, 'Vbus_fb/1',    'svpwm_to_phase_voltages/5', 'autorouting', 'on');

add_line(sys, 'svpwm_to_phase_voltages/1', 'va_cmd/1', 'autorouting', 'on');
add_line(sys, 'svpwm_to_phase_voltages/2', 'vb_cmd/1', 'autorouting', 'on');
add_line(sys, 'svpwm_to_phase_voltages/3', 'vc_cmd/1', 'autorouting', 'on');
end

function rebuild_plant_official_pmsm(sys, probe)
delete_all_lines_in_system(sys);
delete_all_child_blocks(sys);

set_param(sys, 'Position', [505 180 830 370]);

add_block('simulink/Sources/In1', [sys '/va_cmd'], ...
    'Position', [30 30 60 44], 'Port', '1');
add_block('simulink/Sources/In1', [sys '/vb_cmd'], ...
    'Position', [30 65 60 79], 'Port', '2');
add_block('simulink/Sources/In1', [sys '/vc_cmd'], ...
    'Position', [30 100 60 114], 'Port', '3');
add_block('simulink/Sources/In1', [sys '/Tload_cmd'], ...
    'Position', [30 135 60 149], 'Port', '4');

add_block('simulink/Signal Routing/Mux', [sys '/phase_voltage_mux'], ...
    'Inputs', '3', ...
    'Position', [105 40 110 120]);

    pmsmBlk = [sys '/official_pmsm'];
add_block(probe.Path, pmsmBlk, ...
    'Position', [180 28 360 170]);
configure_pmsm_block(pmsmBlk, probe.DialogParameters);

selectorBlk = [sys '/info_bus_selector'];
add_block('simulink/Signal Routing/Bus Selector', selectorBlk, ...
    'OutputSignals', strjoin(official_info_bus_signals(), ','), ...
    'Position', [440 48 530 188]);
configure_info_bus_selector(selectorBlk);

add_block('simulink/Sinks/Out1', [sys '/ia_meas'], ...
    'Position', [600 40 630 54], 'Port', '1');
add_block('simulink/Sinks/Out1', [sys '/ib_meas'], ...
    'Position', [600 75 630 89], 'Port', '2');
add_block('simulink/Sinks/Out1', [sys '/ic_meas'], ...
    'Position', [600 110 630 124], 'Port', '3');
add_block('simulink/Sinks/Out1', [sys '/speed_mech'], ...
    'Position', [600 145 630 159], 'Port', '4');
add_block('simulink/Sinks/Out1', [sys '/theta_mech'], ...
    'Position', [600 180 630 194], 'Port', '5');

add_line(sys, 'va_cmd/1',            'phase_voltage_mux/1', 'autorouting', 'on');
add_line(sys, 'vb_cmd/1',            'phase_voltage_mux/2', 'autorouting', 'on');
add_line(sys, 'vc_cmd/1',            'phase_voltage_mux/3', 'autorouting', 'on');
add_line(sys, 'Tload_cmd/1',         'official_pmsm/1',     'autorouting', 'on');
add_line(sys, 'phase_voltage_mux/1', 'official_pmsm/2',     'autorouting', 'on');
add_line(sys, 'official_pmsm/1',     'info_bus_selector/1', 'autorouting', 'on');

set_param(bdroot(sys), 'SimulationCommand', 'update');

add_line(sys, 'info_bus_selector/1', 'ia_meas/1',    'autorouting', 'on');
add_line(sys, 'info_bus_selector/2', 'ib_meas/1',    'autorouting', 'on');
add_line(sys, 'info_bus_selector/3', 'ic_meas/1',    'autorouting', 'on');
add_line(sys, 'info_bus_selector/4', 'speed_mech/1', 'autorouting', 'on');
add_line(sys, 'info_bus_selector/5', 'theta_mech/1', 'autorouting', 'on');
end

function ensure_top_level_pi_param_blocks(mdl)
delete_block_if_exists([mdl '/Ki_pos']);
create_or_replace_constant(mdl, 'Kp_pos',   'Kp_pos',   [40 80 120 100]);
create_or_replace_constant(mdl, 'Kd_pos',   'Kd_pos',   [40 110 120 130]);
create_or_replace_constant(mdl, 'Kp_speed', 'Kp_speed', [40 170 120 190]);
create_or_replace_constant(mdl, 'Ki_speed', 'Ki_speed', [40 200 120 220]);
create_or_replace_constant(mdl, 'Kp_iq',    'Kp_iq',    [40 260 120 280]);
create_or_replace_constant(mdl, 'Ki_iq',    'Ki_iq',    [40 290 120 310]);
end

function create_or_replace_constant(mdl, name, valueExpr, position)
blockPath = [mdl '/' name];
delete_block_if_exists(blockPath);
add_block('simulink/Sources/Constant', blockPath, ...
    'Value', valueExpr, ...
    'Position', position);
end

function rebuild_position_loop(sys)
delete_all_lines_in_system(sys);
delete_all_child_blocks(sys);

add_block('simulink/Sources/In1', [sys '/pos_ref'], ...
    'Position', [30 40 60 54], 'Port', '1');
add_block('simulink/Sources/In1', [sys '/theta_mech_fb'], ...
    'Position', [30 85 60 99], 'Port', '2');
add_block('simulink/Sources/In1', [sys '/speed_mech_fb'], ...
    'Position', [30 130 60 144], 'Port', '3');
add_block('simulink/Sources/In1', [sys '/Kp_pos'], ...
    'Position', [30 175 60 189], 'Port', '4');
add_block('simulink/Sources/In1', [sys '/Kd_pos'], ...
    'Position', [30 220 60 234], 'Port', '5');

add_block('simulink/Math Operations/Sum', [sys '/pos_error_raw'], ...
    'Inputs', '+-', ...
    'Position', [105 55 125 105]);
add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/wrap_pos_error'], ...
    'Position', [170 55 315 105]);
add_block('simulink/Math Operations/Product', [sys '/p_term'], ...
    'Inputs', '2', ...
    'Position', [370 30 410 60]);
add_block('simulink/Math Operations/Product', [sys '/d_term'], ...
    'Inputs', '2', ...
    'Position', [370 140 410 170]);
add_block('simulink/Math Operations/Sum', [sys '/pd_sum'], ...
    'Inputs', '+-', ...
    'Position', [490 85 510 125]);
add_block('simulink/Discontinuities/Saturation', [sys '/speed_sat'], ...
    'UpperLimit', 'speed_limit', ...
    'LowerLimit', '-speed_limit', ...
    'Position', [565 85 615 125]);
add_block('simulink/Sinks/Out1', [sys '/speed_ref'], ...
    'Position', [670 98 700 112], 'Port', '1');

configure_wrap_matlab_function([sys '/wrap_pos_error'], 'wrap_pos_error');

add_line(sys, 'pos_ref/1',           'pos_error_raw/1',   'autorouting', 'on');
add_line(sys, 'theta_mech_fb/1',     'pos_error_raw/2',   'autorouting', 'on');
add_line(sys, 'pos_error_raw/1',     'wrap_pos_error/1',  'autorouting', 'on');
add_line(sys, 'wrap_pos_error/1',    'p_term/1',          'autorouting', 'on');
add_line(sys, 'Kp_pos/1',            'p_term/2',          'autorouting', 'on');
add_line(sys, 'speed_mech_fb/1',     'd_term/1',          'autorouting', 'on');
add_line(sys, 'Kd_pos/1',            'd_term/2',          'autorouting', 'on');
add_line(sys, 'p_term/1',            'pd_sum/1',          'autorouting', 'on');
add_line(sys, 'd_term/1',            'pd_sum/2',          'autorouting', 'on');
add_line(sys, 'pd_sum/1',            'speed_sat/1',       'autorouting', 'on');
add_line(sys, 'speed_sat/1',         'speed_ref/1',       'autorouting', 'on');
end

function rebuild_speed_loop(sys)
delete_all_lines_in_system(sys);
delete_all_child_blocks(sys);

add_block('simulink/Sources/In1', [sys '/speed_ref'], ...
    'Position', [30 40 60 54], 'Port', '1');
add_block('simulink/Sources/In1', [sys '/theta_mech_fb'], ...
    'Position', [30 85 60 99], 'Port', '2');
add_block('simulink/Sources/In1', [sys '/Kp_speed'], ...
    'Position', [30 130 60 144], 'Port', '3');
add_block('simulink/Sources/In1', [sys '/Ki_speed'], ...
    'Position', [30 175 60 189], 'Port', '4');

add_block('simulink/Discrete/Unit Delay', [sys '/theta_prev'], ...
    'SampleTime', 'Ts_speed', ...
    'InitialCondition', '0', ...
    'Position', [120 135 170 165]);
add_block('simulink/Math Operations/Sum', [sys '/delta_theta_raw'], ...
    'Inputs', '+-', ...
    'Position', [120 55 140 105]);
add_block('simulink/User-Defined Functions/MATLAB Function', [sys '/wrap_delta_theta'], ...
    'Position', [185 55 335 105]);
add_block('simulink/Math Operations/Gain', [sys '/speed_raw_gain'], ...
    'Gain', '1 / Ts_speed', ...
    'Position', [375 67 435 93]);
add_block('simulink/Discrete/Unit Delay', [sys '/speed_prev'], ...
    'SampleTime', 'Ts_speed', ...
    'InitialCondition', '0', ...
    'Position', [510 150 560 180]);
add_block('simulink/Math Operations/Sum', [sys '/speed_residual'], ...
    'Inputs', '+-', ...
    'Position', [500 55 520 105]);
add_block('simulink/Math Operations/Gain', [sys '/speed_lpf_delta'], ...
    'Gain', '((2*pi*200.0)*Ts_speed) / (1.0 + (2*pi*200.0)*Ts_speed)', ...
    'Position', [560 67 655 93]);
add_block('simulink/Math Operations/Sum', [sys '/speed_mech_est'], ...
    'Inputs', '++', ...
    'Position', [705 55 725 105]);
add_block('simulink/Math Operations/Sum', [sys '/speed_error'], ...
    'Inputs', '+-', ...
    'Position', [775 55 795 105]);
add_block('simulink/Math Operations/Product', [sys '/p_term'], ...
    'Inputs', '2', ...
    'Position', [840 30 880 60]);
add_block('simulink/Math Operations/Abs', [sys '/abs_speed_error'], ...
    'Position', [835 80 865 110]);
add_block('simulink/Math Operations/Abs', [sys '/abs_kp_speed'], ...
    'Position', [835 125 865 155]);
add_block('simulink/Sources/Constant', [sys '/kp_eps'], ...
    'Value', '1e-6', ...
    'Position', [835 170 875 190]);
add_block('simulink/Math Operations/MinMax', [sys '/kp_abs_safe'], ...
    'Function', 'max', ...
    'Inputs', '2', ...
    'Position', [915 130 955 185]);
add_block('simulink/Sources/Constant', [sys '/sep_numerator'], ...
    'Value', '0.4 * iq_limit', ...
    'Position', [835 215 900 235]);
add_block('simulink/Math Operations/Product', [sys '/sep_threshold'], ...
    'Inputs', '*/', ...
    'Position', [995 145 1040 175]);
add_block('simulink/Logic and Bit Operations/Relational Operator', [sys '/integrate_enable'], ...
    'Operator', '<', ...
    'Position', [1080 80 1135 110]);
add_block('simulink/Sources/Constant', [sys '/zero_error'], ...
    'Value', '0', ...
    'Position', [1080 130 1115 150]);
add_block('simulink/Signal Routing/Switch', [sys '/gated_error'], ...
    'Criteria', 'u2 >= Threshold', ...
    'Threshold', '0.5', ...
    'Position', [1175 90 1210 140]);
add_block('simulink/Discrete/Unit Delay', [sys '/i_state_prev'], ...
    'SampleTime', 'Ts_speed', ...
    'InitialCondition', '0', ...
    'Position', [1260 175 1310 205]);
add_block('simulink/Math Operations/Sum', [sys '/i_state_sum'], ...
    'Inputs', '++', ...
    'Position', [1265 90 1285 140]);
add_block('simulink/Discontinuities/Saturation', [sys '/i_state_sat'], ...
    'UpperLimit', '0.9 * iq_limit', ...
    'LowerLimit', '-0.9 * iq_limit', ...
    'Position', [1330 90 1380 140]);
add_block('simulink/Math Operations/Product', [sys '/i_term'], ...
    'Inputs', '2', ...
    'Position', [1425 155 1465 185]);
add_block('simulink/Math Operations/Sum', [sys '/pi_sum'], ...
    'Inputs', '++', ...
    'Position', [1520 85 1540 125]);
add_block('simulink/Discontinuities/Saturation', [sys '/iq_sat'], ...
    'UpperLimit', 'iq_limit', ...
    'LowerLimit', '-iq_limit', ...
    'Position', [1590 85 1640 125]);
add_block('simulink/Sinks/Out1', [sys '/iq_ref'], ...
    'Position', [1695 98 1725 112], 'Port', '1');
add_block('simulink/Sinks/Out1', [sys '/speed_mech_fb'], ...
    'Position', [1695 58 1725 72], 'Port', '2');

configure_wrap_matlab_function([sys '/wrap_delta_theta'], 'wrap_delta_theta');

add_line(sys, 'theta_mech_fb/1',    'delta_theta_raw/1',   'autorouting', 'on');
add_line(sys, 'theta_prev/1',       'delta_theta_raw/2',   'autorouting', 'on');
add_line(sys, 'theta_mech_fb/1',    'theta_prev/1',        'autorouting', 'on');
add_line(sys, 'delta_theta_raw/1',  'wrap_delta_theta/1',  'autorouting', 'on');
add_line(sys, 'wrap_delta_theta/1', 'speed_raw_gain/1',    'autorouting', 'on');
add_line(sys, 'speed_raw_gain/1',   'speed_residual/1',    'autorouting', 'on');
add_line(sys, 'speed_prev/1',       'speed_residual/2',    'autorouting', 'on');
add_line(sys, 'speed_residual/1',   'speed_lpf_delta/1',   'autorouting', 'on');
add_line(sys, 'speed_prev/1',       'speed_mech_est/1',    'autorouting', 'on');
add_line(sys, 'speed_lpf_delta/1',  'speed_mech_est/2',    'autorouting', 'on');
add_line(sys, 'speed_mech_est/1',   'speed_prev/1',        'autorouting', 'on');
add_line(sys, 'speed_ref/1',        'speed_error/1',       'autorouting', 'on');
add_line(sys, 'speed_mech_est/1',   'speed_error/2',       'autorouting', 'on');
add_line(sys, 'speed_error/1',      'p_term/1',            'autorouting', 'on');
add_line(sys, 'Kp_speed/1',         'p_term/2',            'autorouting', 'on');
add_line(sys, 'speed_error/1',      'abs_speed_error/1',   'autorouting', 'on');
add_line(sys, 'Kp_speed/1',         'abs_kp_speed/1',      'autorouting', 'on');
add_line(sys, 'abs_kp_speed/1',     'kp_abs_safe/1',       'autorouting', 'on');
add_line(sys, 'kp_eps/1',           'kp_abs_safe/2',       'autorouting', 'on');
add_line(sys, 'sep_numerator/1',    'sep_threshold/1',     'autorouting', 'on');
add_line(sys, 'kp_abs_safe/1',      'sep_threshold/2',     'autorouting', 'on');
add_line(sys, 'abs_speed_error/1',  'integrate_enable/1',  'autorouting', 'on');
add_line(sys, 'sep_threshold/1',    'integrate_enable/2',  'autorouting', 'on');
add_line(sys, 'speed_error/1',      'gated_error/1',       'autorouting', 'on');
add_line(sys, 'integrate_enable/1', 'gated_error/2',       'autorouting', 'on');
add_line(sys, 'zero_error/1',       'gated_error/3',       'autorouting', 'on');
add_line(sys, 'gated_error/1',      'i_state_sum/1',       'autorouting', 'on');
add_line(sys, 'i_state_prev/1',     'i_state_sum/2',       'autorouting', 'on');
add_line(sys, 'i_state_sum/1',      'i_state_sat/1',       'autorouting', 'on');
add_line(sys, 'i_state_sat/1',      'i_state_prev/1',      'autorouting', 'on');
add_line(sys, 'i_state_sat/1',      'i_term/1',            'autorouting', 'on');
add_line(sys, 'Ki_speed/1',         'i_term/2',            'autorouting', 'on');
add_line(sys, 'p_term/1',           'pi_sum/1',            'autorouting', 'on');
add_line(sys, 'i_term/1',           'pi_sum/2',            'autorouting', 'on');
add_line(sys, 'pi_sum/1',           'iq_sat/1',            'autorouting', 'on');
add_line(sys, 'iq_sat/1',           'iq_ref/1',            'autorouting', 'on');
add_line(sys, 'speed_mech_est/1',   'speed_mech_fb/1',     'autorouting', 'on');
end

function rebuild_current_loop(sys)
delete_all_lines_in_system(sys);
delete_all_child_blocks(sys);

add_block('simulink/Sources/In1', [sys '/id_ref'], ...
    'Position', [30 35 60 49], 'Port', '1');
add_block('simulink/Sources/In1', [sys '/iq_ref'], ...
    'Position', [30 70 60 84], 'Port', '2');
add_block('simulink/Sources/In1', [sys '/id_meas'], ...
    'Position', [30 105 60 119], 'Port', '3');
add_block('simulink/Sources/In1', [sys '/iq_meas'], ...
    'Position', [30 140 60 154], 'Port', '4');
add_block('simulink/Sources/In1', [sys '/Kp_iq'], ...
    'Position', [30 175 60 189], 'Port', '5');
add_block('simulink/Sources/In1', [sys '/Ki_iq'], ...
    'Position', [30 210 60 224], 'Port', '6');

add_block('simulink/Math Operations/Sum', [sys '/id_error'], ...
    'Inputs', '+-', ...
    'Position', [105 40 125 90]);
add_block('simulink/Math Operations/Product', [sys '/vd_p_term'], ...
    'Inputs', '2', ...
    'Position', [175 35 215 65]);
add_block('simulink/Math Operations/Product', [sys '/vd_i_input'], ...
    'Inputs', '2', ...
    'Position', [175 110 215 140]);
add_block('simulink/Discrete/Discrete-Time Integrator', [sys '/vd_i_state'], ...
    'SampleTime', 'Ts_current', ...
    'InitialCondition', '0', ...
    'Position', [255 107 315 143]);
add_block('simulink/Math Operations/Sum', [sys '/vd_sum'], ...
    'Inputs', '++', ...
    'Position', [360 58 380 98]);
add_block('simulink/Discontinuities/Saturation', [sys '/vd_sat'], ...
    'UpperLimit', 'vq_limit', ...
    'LowerLimit', '-vq_limit', ...
    'Position', [430 58 480 98]);

add_block('simulink/Math Operations/Sum', [sys '/iq_error'], ...
    'Inputs', '+-', ...
    'Position', [105 155 125 205]);
add_block('simulink/Math Operations/Product', [sys '/vq_p_term'], ...
    'Inputs', '2', ...
    'Position', [175 150 215 180]);
add_block('simulink/Math Operations/Product', [sys '/vq_i_input'], ...
    'Inputs', '2', ...
    'Position', [175 225 215 255]);
add_block('simulink/Discrete/Discrete-Time Integrator', [sys '/vq_i_state'], ...
    'SampleTime', 'Ts_current', ...
    'InitialCondition', '0', ...
    'Position', [255 222 315 258]);
add_block('simulink/Math Operations/Sum', [sys '/vq_sum'], ...
    'Inputs', '++', ...
    'Position', [360 173 380 213]);
add_block('simulink/Discontinuities/Saturation', [sys '/vq_sat'], ...
    'UpperLimit', 'vq_limit', ...
    'LowerLimit', '-vq_limit', ...
    'Position', [430 173 480 213]);

add_block('simulink/Sinks/Out1', [sys '/vd_cmd'], ...
    'Position', [540 73 570 87], 'Port', '1');
add_block('simulink/Sinks/Out1', [sys '/vq_cmd'], ...
    'Position', [540 188 570 202], 'Port', '2');

add_line(sys, 'id_ref/1',   'id_error/1',   'autorouting', 'on');
add_line(sys, 'id_meas/1',  'id_error/2',   'autorouting', 'on');
add_line(sys, 'id_error/1', 'vd_p_term/1',  'autorouting', 'on');
add_line(sys, 'Kp_iq/1',    'vd_p_term/2',  'autorouting', 'on');
add_line(sys, 'id_error/1', 'vd_i_input/1', 'autorouting', 'on');
add_line(sys, 'Ki_iq/1',    'vd_i_input/2', 'autorouting', 'on');
add_line(sys, 'vd_i_input/1', 'vd_i_state/1', 'autorouting', 'on');
add_line(sys, 'vd_p_term/1',  'vd_sum/1',     'autorouting', 'on');
add_line(sys, 'vd_i_state/1', 'vd_sum/2',     'autorouting', 'on');
add_line(sys, 'vd_sum/1',     'vd_sat/1',     'autorouting', 'on');
add_line(sys, 'vd_sat/1',     'vd_cmd/1',     'autorouting', 'on');

add_line(sys, 'iq_ref/1',   'iq_error/1',   'autorouting', 'on');
add_line(sys, 'iq_meas/1',  'iq_error/2',   'autorouting', 'on');
add_line(sys, 'iq_error/1', 'vq_p_term/1',  'autorouting', 'on');
add_line(sys, 'Kp_iq/1',    'vq_p_term/2',  'autorouting', 'on');
add_line(sys, 'iq_error/1', 'vq_i_input/1', 'autorouting', 'on');
add_line(sys, 'Ki_iq/1',    'vq_i_input/2', 'autorouting', 'on');
add_line(sys, 'vq_i_input/1', 'vq_i_state/1', 'autorouting', 'on');
add_line(sys, 'vq_p_term/1',  'vq_sum/1',     'autorouting', 'on');
add_line(sys, 'vq_i_state/1', 'vq_sum/2',     'autorouting', 'on');
add_line(sys, 'vq_sum/1',     'vq_sat/1',     'autorouting', 'on');
add_line(sys, 'vq_sat/1',     'vq_cmd/1',     'autorouting', 'on');
end

function rewire_top_level(mdl)
positionCmdBlk = resolve_position_command_source(mdl);
posRefScopeBlk = resolve_first_existing_block(mdl, {'pos_ref', 'pos ref'});
speedRefScopeBlk = resolve_first_existing_block(mdl, {'speed_ref'});
iqRefScopeBlk = resolve_first_existing_block(mdl, {'Iq_ref', 'iq_ref'});

delete_connected_lines([mdl '/avg inverter bridge']);
delete_connected_lines([mdl '/plant']);
clear_rewire_target_ports(mdl);

add_line(mdl, port_ref(positionCmdBlk, 1), 'position loop/1', 'autorouting', 'on');
add_line(mdl, 'feedback/3',                'position loop/2', 'autorouting', 'on');
add_line(mdl, 'speed loop/2',              'position loop/3', 'autorouting', 'on');
add_line(mdl, 'Kp_pos/1',                  'position loop/4', 'autorouting', 'on');
add_line(mdl, 'Kd_pos/1',                  'position loop/5', 'autorouting', 'on');

add_line(mdl, 'position loop/1',           'speed loop/1', 'autorouting', 'on');
add_line(mdl, 'feedback/3',                'speed loop/2', 'autorouting', 'on');
add_line(mdl, 'Kp_speed/1',                'speed loop/3', 'autorouting', 'on');
add_line(mdl, 'Ki_speed/1',                'speed loop/4', 'autorouting', 'on');

add_line(mdl, 'id_ref_zero/1',             'current loop/1', 'autorouting', 'on');
add_line(mdl, 'speed loop/1',              'current loop/2', 'autorouting', 'on');
add_line(mdl, 'feedback/5',                'current loop/3', 'autorouting', 'on');
add_line(mdl, 'feedback/6',                'current loop/4', 'autorouting', 'on');
add_line(mdl, 'Kp_iq/1',                   'current loop/5', 'autorouting', 'on');
add_line(mdl, 'Ki_iq/1',                   'current loop/6', 'autorouting', 'on');

add_line(mdl, 'feedback/5', 'FOC math/1', 'autorouting', 'on');
add_line(mdl, 'feedback/6', 'FOC math/2', 'autorouting', 'on');
add_line(mdl, 'feedback/1', 'FOC math/3', 'autorouting', 'on');
add_line(mdl, 'current loop/1', 'FOC math/4', 'autorouting', 'on');
add_line(mdl, 'current loop/2', 'FOC math/5', 'autorouting', 'on');
add_line(mdl, 'feedback/7', 'FOC math/6', 'autorouting', 'on');

add_line(mdl, 'avg inverter bridge/1', 'plant/1', 'autorouting', 'on');
add_line(mdl, 'avg inverter bridge/2', 'plant/2', 'autorouting', 'on');
add_line(mdl, 'avg inverter bridge/3', 'plant/3', 'autorouting', 'on');
add_line(mdl, 'Tload_cmd/1',           'plant/4', 'autorouting', 'on');

add_line(mdl, 'plant/1', 'feedback/3',           'autorouting', 'on');
add_line(mdl, 'plant/2', 'feedback/4',           'autorouting', 'on');
add_line(mdl, 'plant/3', 'feedback/5',           'autorouting', 'on');
add_line(mdl, 'plant/1', 'phase_currents/1',     'autorouting', 'on');
add_line(mdl, 'plant/2', 'phase_currents/2',     'autorouting', 'on');
add_line(mdl, 'plant/3', 'phase_currents/3',     'autorouting', 'on');
add_line(mdl, 'plant/1', 'phase_currents_mux/1', 'autorouting', 'on');
add_line(mdl, 'plant/2', 'phase_currents_mux/2', 'autorouting', 'on');
add_line(mdl, 'plant/3', 'phase_currents_mux/3', 'autorouting', 'on');
add_line(mdl, 'phase_currents_mux/1', 'phase_currents_log/1', 'autorouting', 'on');
add_line(mdl, 'plant/4', 'speed_mech/1',         'autorouting', 'on');
add_line(mdl, 'plant/4', 'feedback/2',           'autorouting', 'on');
add_line(mdl, 'plant/5', 'theta_mech/1',         'autorouting', 'on');
add_line(mdl, 'plant/5', 'feedback/1',           'autorouting', 'on');
add_line(mdl, 'Vbus_default/1', 'feedback/6',    'autorouting', 'on');

add_line(mdl, 'FOC math/3', 'avg inverter bridge/1', 'autorouting', 'on');
add_line(mdl, 'FOC math/4', 'avg inverter bridge/2', 'autorouting', 'on');
add_line(mdl, 'FOC math/5', 'avg inverter bridge/3', 'autorouting', 'on');
add_line(mdl, 'feedback/1', 'avg inverter bridge/4', 'autorouting', 'on');
add_line(mdl, 'feedback/7', 'avg inverter bridge/5', 'autorouting', 'on');

connect_optional_line(mdl, positionCmdBlk,           1, posRefScopeBlk,              1);
connect_optional_line(mdl, [mdl '/position loop'],   1, speedRefScopeBlk,            1);
connect_optional_line(mdl, [mdl '/speed loop'],      1, iqRefScopeBlk,               1);
connect_optional_line(mdl, [mdl '/current loop'],    1, [mdl '/foc_debug_mux'],      1);
connect_optional_line(mdl, [mdl '/current loop'],    2, [mdl '/foc_debug_mux'],      2);
connect_optional_line(mdl, [mdl '/FOC math'],        1, [mdl '/foc_debug_mux'],      3);
connect_optional_line(mdl, [mdl '/FOC math'],        2, [mdl '/foc_debug_mux'],      4);
connect_optional_line(mdl, [mdl '/FOC math'],        3, [mdl '/foc_debug_mux'],      5);
connect_optional_line(mdl, [mdl '/FOC math'],        4, [mdl '/foc_debug_mux'],      6);
connect_optional_line(mdl, [mdl '/FOC math'],        5, [mdl '/foc_debug_mux'],      7);
connect_optional_line(mdl, [mdl '/foc_debug_mux'],   1, [mdl '/foc_debug_scope'],    1);
end

function clear_rewire_target_ports(mdl)
clear_block_ports([mdl '/avg inverter bridge'], 'Inport', 1:5);
clear_block_ports([mdl '/plant'], 'Inport', 1:4);
clear_block_ports([mdl '/feedback'], 'Inport', 1:6);
clear_block_ports([mdl '/position loop'], 'Inport', 1:5);
clear_block_ports([mdl '/speed loop'], 'Inport', 1:4);
clear_block_ports([mdl '/current loop'], 'Inport', 1:6);
clear_block_ports([mdl '/FOC math'], 'Inport', 1:6);
clear_block_ports([mdl '/phase_currents'], 'Inport', 1:3);
clear_block_ports([mdl '/phase_currents_mux'], 'Inport', 1:3);
clear_block_ports([mdl '/phase_currents_log'], 'Inport', 1);
clear_block_ports([mdl '/speed_mech'], 'Inport', 1);
clear_block_ports([mdl '/theta_mech'], 'Inport', 1);
clear_first_existing_block_ports(mdl, {'pos_ref', 'pos ref'}, 'Inport', 1);
clear_first_existing_block_ports(mdl, {'speed_ref'}, 'Inport', 1);
clear_first_existing_block_ports(mdl, {'Iq_ref', 'iq_ref'}, 'Inport', 1);
clear_first_existing_block_ports(mdl, {'foc_debug_mux'}, 'Inport', 1:7);
clear_first_existing_block_ports(mdl, {'foc_debug_scope'}, 'Inport', 1);
end

function layout_top_level(mdl)
set_block_position_if_exists([mdl '/feedback'],           [180 140 360 300]);
set_first_existing_block_position(mdl, {'commend sorse', 'command source'}, [180 420 360 500]);
set_block_position_if_exists([mdl '/position loop'],      [470 90 650 180]);
set_block_position_if_exists([mdl '/speed loop'],         [730 90 910 180]);
set_block_position_if_exists([mdl '/current loop'],       [990 70 1190 210]);
set_block_position_if_exists([mdl '/FOC math'],           [1240 70 1465 250]);
set_block_position_if_exists([mdl '/avg inverter bridge'],[1240 300 1465 390]);
set_block_position_if_exists([mdl '/plant'],              [900 320 1180 540]);

set_block_position_if_exists([mdl '/Vbus_default'],       [40 360 80 380]);
set_block_position_if_exists([mdl '/id_ref_zero'],        [900 20 930 40]);
set_block_position_if_exists([mdl '/Tload_cmd'],          [780 470 820 490]);

set_block_position_if_exists([mdl '/Kp_pos'],             [40 80 120 100]);
set_block_position_if_exists([mdl '/Kd_pos'],             [40 110 120 130]);
set_block_position_if_exists([mdl '/Kp_speed'],           [40 170 120 190]);
set_block_position_if_exists([mdl '/Ki_speed'],           [40 200 120 220]);
set_block_position_if_exists([mdl '/Kp_iq'],              [40 260 120 280]);
set_block_position_if_exists([mdl '/Ki_iq'],              [40 290 120 310]);

set_block_position_if_exists([mdl '/phase_currents'],     [1500 380 1535 470]);
set_block_position_if_exists([mdl '/phase_currents_mux'], [1450 520 1455 590]);
set_block_position_if_exists([mdl '/phase_currents_log'], [1500 540 1595 570]);
set_block_position_if_exists([mdl '/speed_mech'],         [1500 600 1530 614]);
set_block_position_if_exists([mdl '/theta_mech'],         [1500 635 1530 649]);
set_block_position_if_exists([mdl '/foc_debug_mux'],      [1490 150 1495 230]);
set_block_position_if_exists([mdl '/foc_debug_scope'],    [1540 160 1580 220]);
set_block_position_if_exists([mdl '/speed_ref'],          [970 255 1000 269]);
set_first_existing_block_position(mdl, {'Iq_ref', 'iq_ref'}, [1205 255 1235 269]);
set_first_existing_block_position(mdl, {'pos_ref', 'pos ref'}, [390 505 420 519]);
end

function tf = block_exists(mdl, name)
matches = find_system(mdl, 'SearchDepth', 1, 'Name', name);
tf = ~isempty(matches);
end

function set_block_position_if_exists(blockPath, position)
mdl = get_model_name(blockPath);
if ~block_exists(mdl, get_param_name(blockPath))
    return;
end
set_param(blockPath, 'Position', position);
end

function clear_block_ports(blockPath, portType, indices)
for i = 1:numel(indices)
    delete_port_line(blockPath, portType, indices(i));
end
end

function clear_first_existing_block_ports(mdl, names, portType, indices)
blockPath = resolve_first_existing_block(mdl, names);
if isempty(blockPath)
    return;
end
clear_block_ports(blockPath, portType, indices);
end

function set_first_existing_block_position(mdl, names, position)
blockPath = resolve_first_existing_block(mdl, names);
if isempty(blockPath)
    return;
end
set_param(blockPath, 'Position', position);
end

function blockPath = resolve_position_command_source(mdl)
blockPath = '';

scopeBlk = resolve_first_existing_block(mdl, {'pos_ref', 'pos ref'});
if ~isempty(scopeBlk)
    blockPath = get_line_source_parent(scopeBlk, 'Inport', 1);
    if ~isempty(blockPath)
        return;
    end
end

for idx = [2 1]
    blockPath = get_line_source_parent([mdl '/position loop'], 'Inport', idx);
    if ~isempty(blockPath) && ~strcmp(blockPath, [mdl '/feedback'])
        return;
    end
end

blockPath = resolve_first_existing_block(mdl, {'commend sorse', 'command source'});
if ~isempty(blockPath)
    return;
end

blocks = find_system(mdl, 'SearchDepth', 1, 'Type', 'Block');
names = cellfun(@get_param_name, blocks, 'UniformOutput', false);
error('Could not resolve the top-level position reference source. Top-level blocks: %s', ...
    strjoin(names, ', '));
end

function blockPath = resolve_first_existing_block(mdl, names)
blockPath = '';
for i = 1:numel(names)
    if block_exists(mdl, names{i})
        blockPath = [mdl '/' names{i}];
        return;
    end
end
end

function connect_optional_line(mdl, srcBlockPath, srcPortIdx, dstBlockPath, dstPortIdx)
if isempty(srcBlockPath) || isempty(dstBlockPath)
    return;
end
if ~block_exists(get_model_name(srcBlockPath), get_param_name(srcBlockPath)) || ...
        ~block_exists(get_model_name(dstBlockPath), get_param_name(dstBlockPath))
    return;
end

add_line(mdl, port_ref(srcBlockPath, srcPortIdx), port_ref(dstBlockPath, dstPortIdx), ...
    'autorouting', 'on');
end

function ref = port_ref(blockPath, portIdx)
ref = sprintf('%s/%d', get_param_name(blockPath), portIdx);
end

function src = get_line_source_parent(blockPath, portType, idx)
src = '';
try
    ph = get_param(blockPath, 'PortHandles');
    if ~isfield(ph, portType)
        return;
    end
    handles = ph.(portType);
    if idx > numel(handles)
        return;
    end
    lineHandle = get_param(handles(idx), 'Line');
    if lineHandle == -1
        return;
    end
    srcPort = get_param(lineHandle, 'SrcPortHandle');
    src = getfullname(get_param(srcPort, 'Parent'));
catch
    src = '';
end
end

function configure_pmsm_block(blockPath, dialogParameters)
apply_param_if_exists(blockPath, dialogParameters, 'port_config', 'Torque');
apply_param_if_exists(blockPath, dialogParameters, 'sim_type', 'Discrete');
apply_param_if_exists(blockPath, dialogParameters, 'Ts', 'Ts_current');
apply_param_if_exists(blockPath, dialogParameters, 'P', 'pole_pairs');
apply_param_if_exists(blockPath, dialogParameters, 'Rs', 'Rs');

if isfield(dialogParameters, 'Ldq_')
    set_param(blockPath, 'Ldq_', 'Ld');
else
    apply_param_if_exists(blockPath, dialogParameters, 'Ld', 'Ld');
    apply_param_if_exists(blockPath, dialogParameters, 'Lq', 'Lq');
end

apply_param_if_exists(blockPath, dialogParameters, 'lambda_pm', 'psi_f');
apply_param_if_exists(blockPath, dialogParameters, 'mechanical', '[J B 0]');
apply_param_if_exists(blockPath, dialogParameters, 'idq0', '[0 0]');
apply_param_if_exists(blockPath, dialogParameters, 'theta_init', '0');
apply_param_if_exists(blockPath, dialogParameters, 'omega_init', '0');
end

function configure_info_bus_selector(blockPath)
requestedSignals = official_info_bus_signals();
try
    set_param(blockPath, 'OutputSignals', strjoin(requestedSignals, ','));
catch ME
    available = safe_get_param(blockPath, 'InputSignals');
    error(['Could not configure the official PMSM Info bus selector.' newline ...
        'Requested signals: %s' newline ...
        'Available signals: %s' newline ...
        'Original error: %s'], ...
        strjoin(requestedSignals, ', '), stringify_value(available), ME.message);
end
end

function signals = official_info_bus_signals()
signals = {'IaStator', 'IbStator', 'IcStator', 'MtrSpd', 'MtrPos'};
end

function apply_param_if_exists(blockPath, dialogParameters, name, value)
if isfield(dialogParameters, name)
    set_param(blockPath, name, value);
end
end

function configure_bridge_matlab_function(blockPath)
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('MATLAB Function block not found: %s', blockPath);
end

chart.SupportVariableSizing = false;
chart.Script = sprintf([ ...
    'function [va_cmd, vb_cmd, vc_cmd] = svpwm_to_phase_voltages(duty_a, duty_b, duty_c, theta_elec, Vbus_fb)\n' ...
    'duty_a = min(max(duty_a, 0.0), 1.0);\n' ...
    'duty_b = min(max(duty_b, 0.0), 1.0);\n' ...
    'duty_c = min(max(duty_c, 0.0), 1.0);\n' ...
    'ma = 2.0 * duty_a - 1.0;\n' ...
    'mb = 2.0 * duty_b - 1.0;\n' ...
    'mc = 2.0 * duty_c - 1.0;\n' ...
    'va = ma * Vbus_fb * 0.5;\n' ...
    'vb = mb * Vbus_fb * 0.5;\n' ...
    'vc = mc * Vbus_fb * 0.5;\n' ...
    'v0 = (va + vb + vc) / 3.0;\n' ...
    'va_cmd = va - v0;\n' ...
    'vb_cmd = vb - v0;\n' ...
    'vc_cmd = vc - v0;\n' ...
    'va_cmd = va_cmd + 0.0 * theta_elec;\n' ...
    'end\n']);

configure_chart_data(chart, 'duty_a',     'Input');
configure_chart_data(chart, 'duty_b',     'Input');
configure_chart_data(chart, 'duty_c',     'Input');
configure_chart_data(chart, 'theta_elec', 'Input');
configure_chart_data(chart, 'Vbus_fb',    'Input');

configure_chart_data(chart, 'va_cmd', 'Output');
configure_chart_data(chart, 'vb_cmd', 'Output');
configure_chart_data(chart, 'vc_cmd', 'Output');
end

function configure_wrap_matlab_function(blockPath, functionName)
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blockPath);
if isempty(chart)
    error('MATLAB Function block not found: %s', blockPath);
end

chart.SupportVariableSizing = false;
chart.Script = sprintf([ ...
    'function angle_out = %s(angle_in)\n' ...
    'angle_out = angle_in;\n' ...
    'while angle_out > pi\n' ...
    '    angle_out = angle_out - 2.0 * pi;\n' ...
    'end\n' ...
    'while angle_out < -pi\n' ...
    '    angle_out = angle_out + 2.0 * pi;\n' ...
    'end\n' ...
    'end\n'], functionName);

configure_chart_data(chart, 'angle_in', 'Input');
configure_chart_data(chart, 'angle_out', 'Output');
end

function ensure_phase_current_observation(mdl)
scopeBlk = [mdl '/phase_currents'];
muxBlk = [mdl '/phase_currents_mux'];
logBlk = [mdl '/phase_currents_log'];

delete_block_if_exists(scopeBlk);
delete_block_if_exists(muxBlk);
delete_block_if_exists(logBlk);

add_block('simulink/Sinks/Scope', scopeBlk, ...
    'NumInputPorts', '3', ...
    'Position', [980 235 1010 320]);
add_block('simulink/Signal Routing/Mux', muxBlk, ...
    'Inputs', '3', ...
    'Position', [930 360 935 430]);
add_block('simulink/Sinks/To Workspace', logBlk, ...
    'VariableName', 'phase_currents_log', ...
    'SaveFormat', 'Array', ...
    'Position', [980 380 1070 410]);
end

function probe = probe_official_pmsm_block()
candidateLibraries = [{'mcb_lib', 'autoblkslib', 'autoblks_lib', ...
    'powertrainlib', 'autolib', 'autolibmotor', 'autolibmotorctrlr'}, ...
    discover_library_roots_from_slblocks()];
candidateLibraries = unique(candidateLibraries, 'stable');
matches = {};

for i = 1:numel(candidateLibraries)
    try
        load_system(candidateLibraries{i});
    catch
    end
end

for i = 1:numel(candidateLibraries)
    lib = candidateLibraries{i};
    if ~bdIsLoaded(lib)
        continue;
    end

    exactMatches = find_system(lib, ...
        'LookUnderMasks', 'all', ...
        'FollowLinks', 'on', ...
        'MatchFilter', @Simulink.match.allVariants, ...
        'Type', 'Block', ...
        'Name', 'Surface Mount PMSM');
    if isempty(exactMatches)
        regexMatches = find_system(lib, ...
            'RegExp', 'on', ...
            'LookUnderMasks', 'all', ...
            'FollowLinks', 'on', ...
            'MatchFilter', @Simulink.match.allVariants, ...
            'Type', 'Block', ...
            'Name', '.*Surface.*PMSM.*');
    else
        regexMatches = {};
    end
    blockMatches = unique([exactMatches; regexMatches]);

    for j = 1:numel(blockMatches)
        dialogParameters = safe_get_param(blockMatches{j}, 'DialogParameters');
        if ~isstruct(dialogParameters)
            continue;
        end

        score = 0;
        if strcmp(get_param(blockMatches{j}, 'Name'), 'Surface Mount PMSM')
            score = score + 100;
        end
        score = score + 10 * double(isfield(dialogParameters, 'port_config'));
        score = score + 10 * double(isfield(dialogParameters, 'sim_type'));
        score = score + 10 * double(isfield(dialogParameters, 'Rs'));
        score = score + 10 * double(isfield(dialogParameters, 'P'));
        score = score + 10 * double(isfield(dialogParameters, 'lambda_pm'));

        matches{end + 1} = struct( ... %#ok<AGROW>
            'Library', lib, ...
            'Path', blockMatches{j}, ...
            'Name', get_param(blockMatches{j}, 'Name'), ...
            'DialogParameters', dialogParameters, ...
            'ParamNames', {sort(fieldnames(dialogParameters))}, ...
            'Score', score);
    end
end

if isempty(matches)
    relatedProducts = detect_related_products();
    error(['Could not find an official Surface Mount PMSM block in the loaded MathWorks libraries.' newline ...
        'Checked libraries: %s' newline ...
        'Installed related products: %s'], ...
        strjoin(candidateLibraries, ', '), strjoin(relatedProducts, ', '));
end

scores = zeros(1, numel(matches));
for i = 1:numel(matches)
    scores(i) = matches{i}.Score;
end

[~, bestIdx] = max(scores);
probe = matches{bestIdx};
end

function libraries = discover_library_roots_from_slblocks()
libraries = {};

slblocksFiles = which('slblocks.m', '-all');
for i = 1:numel(slblocksFiles)
    fileLibraries = extract_library_roots_from_slblocks_file(slblocksFiles{i});
    libraries = [libraries, fileLibraries]; %#ok<AGROW>
end

libraries = unique(libraries, 'stable');
libraries = libraries(~cellfun(@isempty, libraries));
end

function libraries = extract_library_roots_from_slblocks_file(filePath)
libraries = {};

try
    text = fileread(filePath);
catch
    return;
end

patterns = { ...
    'Browser\.Library\s*=\s*''([^'']+)''', ...
    'blkStruct\.Browser\.Library\s*=\s*''([^'']+)''', ...
    'Browser\(\d+\)\.Library\s*=\s*''([^'']+)'''};

for i = 1:numel(patterns)
    tokens = regexp(text, patterns{i}, 'tokens');
    for j = 1:numel(tokens)
        candidate = strtrim(tokens{j}{1});
        if isempty(candidate)
            continue;
        end
        libraries{end + 1} = candidate; %#ok<AGROW>
    end
end
end

function products = detect_related_products()
products = {};
names = {'Motor Control Blockset', 'Powertrain Blockset', 'Simulink', 'MATLAB'};
for i = 1:numel(names)
    info = ver(names{i});
    if ~isempty(info)
        products{end + 1} = sprintf('%s %s', info.Name, info.Version); %#ok<AGROW>
    end
end
if isempty(products)
    products = {'<none detected>'};
end
end

function configure_chart_data(chart, name, scope)
data = chart.find('-isa', 'Stateflow.Data', 'Name', name);
if isempty(data)
    data = Stateflow.Data(chart);
    data.Name = name;
end
data.Scope = scope;
data.DataType = 'double';
data.Props.Array.Size = '1';
end

function value = safe_get_param(blockPath, paramName)
try
    value = get_param(blockPath, paramName);
catch
    value = [];
end
end

function text = stringify_value(value)
if ischar(value)
    text = value;
elseif isstring(value)
    text = char(strjoin(value, ", "));
elseif iscell(value)
    try
        text = strjoin(string(value), ', ');
        text = char(text);
    catch
        text = '<cell>';
    end
elseif isempty(value)
    text = '<unavailable>';
else
    text = '<non-text>';
end
end

function delete_block_if_exists(blockPath)
mdl = get_model_name(blockPath);
matches = find_system(mdl, 'SearchDepth', 1, 'Name', get_param_name(blockPath));
if isempty(matches)
    return;
end
delete_connected_lines(matches{1});
delete_block(matches{1});
end

function name = get_param_name(blockPath)
[~, name] = fileparts(blockPath);
end

function mdl = get_model_name(blockPath)
parts = strsplit(blockPath, '/');
mdl = parts{1};
end

function delete_all_child_blocks(sys)
blocks = find_system(sys, 'SearchDepth', 1, 'Type', 'Block');
blocks(strcmp(blocks, sys)) = [];
blocks = sort(blocks);
for i = numel(blocks):-1:1
    delete_block(blocks{i});
end
end

function delete_all_lines_in_system(sys)
lines = find_system(sys, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
for i = 1:numel(lines)
    try
        delete_line(lines(i));
    catch
    end
end
end

function delete_connected_lines(block)
mdl = get_model_name(block);
matches = find_system(mdl, 'SearchDepth', 1, 'Name', get_param_name(block));
if isempty(matches)
    return;
end
block = matches{1};
ph = get_param(block, 'PortHandles');
fields = fieldnames(ph);
for i = 1:numel(fields)
    handles = ph.(fields{i});
    if isempty(handles)
        continue;
    end
    for j = 1:numel(handles)
        try
            lh = get_param(handles(j), 'Line');
            if lh ~= -1
                delete_line(lh);
            end
        catch
        end
    end
end
end

function delete_port_line(block, portType, idx)
ph = get_param(block, 'PortHandles');
if ~isfield(ph, portType)
    return;
end
handles = ph.(portType);
if idx > numel(handles)
    return;
end
try
    lh = get_param(handles(idx), 'Line');
    if lh ~= -1
        delete_line(lh);
    end
catch
end
end

function delete_dangling_lines(sys)
lines = find_system(sys, 'FindAll', 'on', 'SearchDepth', 1, 'Type', 'line');
for i = 1:numel(lines)
    try
        src = get_param(lines(i), 'SrcBlockHandle');
        dst = get_param(lines(i), 'DstBlockHandle');
        if isequal(src, -1) || isempty(dst) || all(dst == -1)
            delete_line(lines(i));
        end
    catch
    end
end
end
