function [T_world, joints_world_m, info] = carrier_pose_from_axes(qx_m,qz_m)
%CARRIER_POSE_FROM_AXES Pose the20 original igus links from machine coordinates.
% qX positive worldX, qZ positive down; inputs/outputs in SI units.
% T_world(:,:,i) applies to igus_07_50_link_mount_axes local geometry (metres).
% Kinematic visualization only: no chain force/stiffness/fatigue model.
validateattributes(qx_m,{'double'},{'real','finite','scalar','>=',0,'<=',.5});
validateattributes(qz_m,{'double'},{'real','finite','scalar','>=',0,'<=',.2});
qz=1000*qz_m;
s=((1:20)-.5)'*20;
a=(414+qz-pi*38)/2; radius=38;
for iteration=1:16
    raw=(s-a)/radius;
    theta=min(pi,max(0,raw));
    residual=[sum(20*sin(theta))-76;sum(20*cos(theta))-(14+qz)];
    if norm(residual,inf)<1e-9
        break
    end
    active=raw>0 & raw<pi;
    da=-double(active)/radius;
    dr=-theta.*double(active)/radius;
    J=[sum(20*cos(theta).*da),sum(20*cos(theta).*dr);...
       sum(-20*sin(theta).*da),sum(-20*sin(theta).*dr)];
    step=J\residual;
    a=a-step(1);radius=radius-step(2);
end
theta=min(pi,max(0,(s-a)/radius));
delta=[zeros(20,1),20*sin(theta),20*cos(theta)];
points=[0,65,76-qz; [0,65,76-qz]+cumsum(delta,1)];
closure_mm=norm(points(end,:)-[0,141,90]);
assert(closure_mm<1e-7,'Carrier:Closure','Carrier endpoint solve did not converge.');
origin=[-50+1000*qx_m,-65,357.75];
joints_world_m=(points+origin)/1000;
T_world=repmat(eye(4),1,1,20);
for i=1:20
    c=cos(theta(i)); sn=sin(theta(i));
    T_world(1:3,1:3,i)=[1,0,0;0,c,sn;0,-sn,c];
    T_world(1:3,4,i)=mean(joints_world_m(i:i+1,:),1)';
end
turn=diff(theta); turn=turn(turn>1e-9);
info=struct('closure_m',closure_mm/1000,'heading_rad',theta,...
    'minimum_polygon_joint_radius_m',min(20./(2*sin(turn/2)))/1000,...
    'iterations',iteration,'scope','Kinematic visualization; no force feedback');
end
