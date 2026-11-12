function prepare_carrier_visualisation(model, variant)
% Articulate the existing X/Z carrier representation for the nominal demo.
root = fileparts(mfilename('fullpath'));
motion = readmatrix(fullfile(root,[variant '_carrier_motion.csv']));
assert(size(motion,2)==3 && all(isfinite(motion(:))) && ...
    motion(1,1)==0 && all(diff(motion(:,1))>0), 'Invalid carrier motion data.');
t = motion(:,1);
x = motion(:,2);
z = motion(:,3);
[linkX,linkZ] = carrierTracks(x,z);

% Recreate the workspace signals if the demo is opened again in this session.
replay.x_slide = channel(t,x);
if getSimulinkBlockHandle([model '/Media X carrier plane']) ~= -1
    for k = 1:size(linkX,3)
        replay = addChannels('X',k,t,linkX(:,:,k),replay);
    end
    for k = 1:size(linkZ,3)
        replay = addChannels('Z',k,t,linkZ(:,:,k),replay);
    end
    assignin('base','xz_media_carrier_replay',replay);
    return
end

load_system('sm_lib');
load_system('nesl_utility');

% Hide the stationary CAD snapshots and neutral internal routing inserts.
for sid = [1464:3:1587 733:3:940]
    h = Simulink.ID.getHandle(sprintf('%s:%d',model,sid));
    set_param(h,'GraphicType','None');
end

xMesh = fullfile(root,'igus_1500_050_link_path_axes.stp');
zMesh = fullfile(root,'igus_07_50_link_mount_axes.stp');
assert(isfile(xMesh) && isfile(zMesh),'Local link geometry is missing.');
world = ports([model '/World']).RConn;
xPlane = transform(model,'Media X carrier plane',[0 -0.330 0],[1 0 0],90,[2120 140 2170 180]);
wire(model,world,ports(xPlane).LConn);

% The Z-carrier plane rides on an independent replay of measured X.
xSlidePlane = transform(model,'Media Z carrier travel axis',[-0.050 0 0],[0 1 0],90,[2120 220 2170 260]);
wire(model,world,ports(xSlidePlane).LConn);
slide = add_block(sprintf('sm_lib/Joints/Prismatic\nJoint'),[model '/Media Z carrier X position'], ...
    'Position',[2220 220 2270 260]);
wire(model,ports(xSlidePlane).RConn,ports(slide).LConn);
zPlane = transform(model,'Media Z carrier plane',[0 0 0],[0 0 1],90,[2320 220 2370 260]);
wire(model,ports(slide).RConn,ports(zPlane).LConn);
set_param(slide,'MotionActuationMode','InputMotion','TorqueActuationMode','ComputedTorque');
replay.x_slide = channel(t,x);
drive(model,slide,'x_slide','m',280);

for k=1:size(linkX,3)
    replay = addLink(model,'X',k,ports(xPlane).RConn,xMesh, ...
        t,linkX(:,:,k),replay,false);
end
for k=1:size(linkZ,3)
    replay = addLink(model,'Z',k,ports(zPlane).RConn,zMesh, ...
        t,linkZ(:,:,k),replay,true);
end
assignin('base','xz_media_carrier_replay',replay);
end

function replay = addChannels(axis,k,t,values,replay)
components = {'px','py','rz'};
for j = 1:numel(components)
    field = sprintf('%s%02d_%s',lower(axis),k,components{j});
    replay.(field) = channel(t,values(:,j));
end
end

function [x,z] = carrierTracks(qx,qz)
x = zeros(numel(qx),3,22);
z = zeros(numel(qx),3,20);
for n=1:numel(qx)
    [tx,~,ix] = x_carrier_pose_from_axis(qx(n));
    [tz,~,iz] = carrier_pose_from_axes(qx(n),qz(n));
    x(n,1,:) = tx(1,4,:);
    x(n,2,:) = tx(3,4,:);
    x(n,3,:) = reshape(ix.heading_rad,1,1,[]);
    z(n,1,:) = tz(2,4,:);
    z(n,2,:) = tz(3,4,:);
    z(n,3,:) = -reshape(iz.heading_rad,1,1,[]);
end
end

function replay = addLink(model,axis,k,base,mesh,t,values,replay,zLink)
label = sprintf('Media %s link %02d',axis,k);
y = 440+(k-1)*230+5200*zLink;
joint = add_block('sm_lib/Joints/Planar Joint',[model '/' label ' joint'], ...
    'Position',[2500 y 2550 y+40]);
wire(model,base,ports(joint).LConn);
solid = add_block('sm_lib/Body Elements/File Solid',[model '/' label], ...
    'ExtGeomFileName',mesh,'InertiaType','Custom','Mass','1e-6', ...
    'MomentsOfInertia','[1e-10 1e-10 1e-10]','MomentsOfInertiaUnits','kg*m^2', ...
    'GraphicDiffuseColor','[0.25 0.28 0.31]', ...
    'Position',[2720 y 2770 y+40]);
follower = ports(joint).RConn;
if zLink
    correction = transform(model,[label ' mesh basis'],[0 0 0],[1 1 1],-120,[2610 y 2660 y+40]);
    wire(model,follower,ports(correction).LConn);
    follower = ports(correction).RConn;
end
wire(model,follower,ports(solid).RConn);
primitive = {'Px','Py','Rz'};
units = {'m','m','rad'};
for j=1:numel(primitive)
    prefix = primitive{j};
    set_param(joint,[prefix 'MotionActuationMode'],'InputMotion', ...
        [prefix 'TorqueActuationMode'],'ComputedTorque');
    field = sprintf('%s%02d_%s',lower(axis),k,lower(prefix));
    replay.(field) = channel(t,values(:,j));
    drive(model,joint,field,units{j},y+55*j);
end
end

function data = channel(t,position)
velocity = gradient(position,t);
acceleration = gradient(velocity,t);
data = [t position velocity acceleration];
end

function block = transform(model,label,offset,axis,angle,position)
block = add_block(sprintf('sm_lib/Frames and\nTransforms/Rigid\nTransform'), ...
    [model '/' label],'Position',position,'RotationMethod','ArbitraryAxis', ...
    'RotationArbitraryAxis',mat2str(axis),'RotationAngle',num2str(angle), ...
    'RotationAngleUnits','deg','TranslationMethod','Cartesian', ...
    'TranslationCartesianOffset',mat2str(offset),'TranslationCartesianOffsetUnits','m');
end

function drive(model,joint,field,unit,y)
source = add_block('simulink/Sources/From Workspace',[model '/Track ' field], ...
    'VariableName',['xz_media_carrier_replay.' field],'Position',[1920 y 2010 y+30]);
split = add_block('simulink/Signal Routing/Demux',[model '/Derivatives ' field], ...
    'Outputs','3','Position',[2070 y 2080 y+40]);
converter = add_block(sprintf('nesl_utility/Simulink-PS\nConverter'), ...
    [model '/Drive ' field],'Unit',unit,'FilteringAndDerivatives','provide', ...
    'UdotUserProvided','2','Position',[2180 y 2210 y+40]);
cp = ports(converter);
sp = ports(split);
wire(model,ports(source).Outport,sp.Inport);
for j=1:3
    wire(model,sp.Outport(j),cp.Inport(j));
end
jp = ports(joint);
physical = [jp.LConn jp.RConn];
input = physical(arrayfun(@(h)get_param(h,'Line')==-1,physical));
assert(isscalar(input),'Expected one free input on %s.',getfullname(joint));
wire(model,cp.RConn,input);
end

function p = ports(block)
p = get_param(block,'PortHandles');
end

function wire(model,source,destination)
add_line(model,source,destination,'autorouting','on');
end
