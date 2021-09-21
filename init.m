root = fileparts(mfilename('fullpath'));
com.mathworks.services.Prefs.setBooleanPref('EditorGraphicalDebugging', false);
warning('off', 'all');
addpath(genpath(fullfile(root,'..','vgtk')));
addpath(genpath(root));