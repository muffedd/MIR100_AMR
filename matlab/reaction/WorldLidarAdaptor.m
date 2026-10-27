classdef WorldLidarAdaptor < robotics.SensorAdaptor
    % Real MathWorks lidar on static scenario meshes, excluding its host.
    % The stock adaptor includes the scanner's own opaque housing. This
    % adapter keeps that geometry intact and excludes only the host robot
    % from ray targets. Single-robot/static-obstacle scenarios only.
    properties
        UpdateRate
        SensorToBase
    end
    methods
        function obj=WorldLidarAdaptor(model,sensorToBase)
            obj@robotics.SensorAdaptor(model);
            obj.UpdateRate=model.UpdateRate;
            obj.SensorToBase=sensorToBase;
        end
        function setup(~,~,~)
        end
        function [cloud,valid]=read(obj,scene,platform,~,t)
            if numel(scene.Platforms)~=1 || ~strcmp(scene.Platforms(1).Name,platform.Name)
                error('WorldLidarAdaptor:dynamicPlatforms', ...
                    'This adapter supports one host robot and static meshes only.');
            end
            m=read(platform,'base');
            worldFromSensor=trvec2tform(m(1:3))*quat2tform(m(10:13))*obj.SensorToBase;
            sensorFromWorld=inv(worldFromSensor);
            meshes=scene.Meshes;
            targets=repmat(struct('Position',sensorFromWorld(1:3,4)', ...
                'Orientation',quaternion(tform2quat(sensorFromWorld)), ...
                'Mesh',[]),1,numel(meshes));
            for k=1:numel(meshes),targets(k).Mesh=meshes{k};end
            [cloud,valid]=obj.SensorModel(targets,t);
        end
        function reset(obj)
            reset(obj.SensorModel);release(obj.SensorModel);
        end
        function out=getEmptyOutputs(~)
            out={pointCloud([nan nan nan]),false};
        end
    end
end
