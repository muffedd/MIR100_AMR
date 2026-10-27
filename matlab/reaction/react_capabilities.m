function caps = react_capabilities()
%REACT_CAPABILITIES  Gate optional APIs. Never errors.
%  Per MathWorks docs: controllerTEB (Navigation Toolbox) introduced R2023a;
%  the warehouse unseen-obstacle example is 'Since R2024b'; robotLidarPointCloudGenerator
%  (Robotics System Toolbox) additionally REQUIRES Computer Vision Toolbox.
caps = struct('matlab', version, 'release', '', 'navToolbox', false, ...
    'roboticsToolbox', false, 'computerVisionToolbox', false, 'controllerTEB', false, ...
    'robotLidarPointCloudGenerator', false, 'robotScenario', false, 'robotSensor', false, 'note', '');
try, caps.release = version('-release'); catch, end
try, caps.navToolbox = license('test','Navigation_Toolbox') == 1 && ~isempty(ver('nav')); catch, end
try, caps.roboticsToolbox = license('test','Robotics_System_Toolbox') == 1; catch, end
try, caps.computerVisionToolbox = license('test','Video_and_Image_Blockset') == 1 && ~isempty(ver('vision')); catch, end
caps.robotSensor = exist('robotSensor','class') == 8 || exist('robotSensor','file') > 0;
caps.controllerTEB = exist('controllerTEB','class') == 8 || exist('controllerTEB','file') > 0;
caps.robotLidarPointCloudGenerator = exist('robotLidarPointCloudGenerator','file') > 0 || exist('robotLidarPointCloudGenerator','class') == 8;
caps.robotScenario = exist('robotScenario','class') == 8 || exist('robotScenario','file') > 0;
caps.note = 'Existence checks only; no claim that the toolbox functions run correctly.';
end
