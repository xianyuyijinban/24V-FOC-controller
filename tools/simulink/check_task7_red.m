modelPath = 'D:/matlab/minimal_foc_controller_task6_observer_load.slx';
modelName = 'minimal_foc_controller_task6_observer_load';

load_system(modelPath);

plantPorts = get_param([modelName '/plant'], 'Ports');
feedbackPorts = get_param([modelName '/feedback'], 'Ports');

fprintf('TASK6_PLANT_IN=%d OUT=%d\n', plantPorts(1), plantPorts(2));
fprintf('TASK6_FEEDBACK_IN=%d OUT=%d\n', feedbackPorts(1), feedbackPorts(2));

assert(plantPorts(2) == 5, ...
    'RED CHECK: Task 6 plant still lacks Task 7 phase-current outputs (expected 5 outputs).');
assert(feedbackPorts(1) == 6, ...
    'RED CHECK: Task 6 feedback still lacks Task 7 abc current inputs (expected 6 inputs).');

close_system(modelName, 0);
