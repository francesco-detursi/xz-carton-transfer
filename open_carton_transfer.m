function open_carton_transfer(variant)
% Open the optimised cycle by default, or pass 'baseline' for the original.
if nargin < 1
    variant = 'optimised';
end
variant = validatestring(variant, {'optimised', 'baseline'}, mfilename, 'variant');

project = fileparts(mfilename('fullpath'));
matlabRoot = fullfile(project, 'matlab');
model = 'xz_packaging_automatic';
folder = fullfile(matlabRoot, 'models', variant);
file = fullfile(folder, [model '.slx']);
assert(isfile(file), 'Missing model: %s', file);

if bdIsLoaded(model) && ~sameFile(get_param(model, 'FileName'), file)
    assert(strcmp(get_param(model, 'Dirty'), 'off'), ...
        'Save or close the current automatic model before switching cycles.');
    close_system(model, 0);
end

addpath(fullfile(matlabRoot, 'import'), '-begin');
addpath(folder, '-begin');
load_system(file);
assert(sameFile(get_param(model, 'FileName'), file), ...
    'MATLAB opened a different model path.');
assert(strcmp(get_param(model, 'Dirty'), 'off'), ...
    'Save or close the modified model before preparing the nominal demo.');
addpath(fullfile(matlabRoot, 'visualisation'), '-begin');
try
    prepare_carrier_visualisation(model, variant);
    set_param(model, 'SimulationCommand', 'update');
catch problem
    close_system(model, 0);
    rethrow(problem);
end
% The carrier overlay is assembled in memory from the distributed files.
set_param(model, 'Dirty', 'off');
open_system(model);
end

function match = sameFile(left, right)
match = strcmpi(strrep(left, '\', '/'), strrep(right, '\', '/'));
end
