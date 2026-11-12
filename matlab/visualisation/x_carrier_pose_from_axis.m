function [T_world,joints_world_m,info] = x_carrier_pose_from_axis(qx_m)
%X_CARRIER_POSE_FROM_AXIS Original22-link X carrier kinematic visualization.
% SI input/output. Apply T_world to igus_1500_050_link_path_axes geometry in metres.
% No carrier-force, fill, load, deflection or fatigue model.
validateattributes(qx_m,{'double'},{'real','finite','scalar','>=',0,'<=',.5});
pitch=33.33; s=((1:22)-.5)'*pitch; target=[1000*qx_m-250;250];
radius=125;a=(22*pitch+target(1)-pi*radius)/2;
for iteration=1:20
 raw=(s-a)/radius;theta=min(pi,max(0,raw));
 residual=[sum(pitch*cos(theta));sum(pitch*sin(theta))]-target;
 if norm(residual,inf)<1e-9,break;end
 active=raw>0 & raw<pi;da=-double(active)/radius;dr=-theta.*double(active)/radius;
 J=[sum(-pitch*sin(theta).*da),sum(-pitch*sin(theta).*dr);sum(pitch*cos(theta).*da),sum(pitch*cos(theta).*dr)];
 step=J\residual;a=a-step(1);radius=radius-step(2);
end
assert(norm(residual,inf)<1e-7,'CarrierX:Closure','X carrier solve did not converge.');
points=[180,-330,100;[180,-330,100]+cumsum([pitch*cos(theta),zeros(22,1),pitch*sin(theta)],1)];
joints_world_m=points/1000;T_world=repmat(eye(4),1,1,22);
for i=1:22
 c=cos(theta(i));sn=sin(theta(i));
 T_world(1:3,1:3,i)=[c,-sn,0;0,0,-1;sn,c,0];
 T_world(1:3,4,i)=joints_world_m(i,:)';
end
turn=diff(theta);turn=turn(turn>1e-9);
info=struct('closure_m',norm(points(end,:)-[-70+1000*qx_m,-330,350])/1000,...
 'heading_rad',theta,'radius_parameter_m',radius/1000,...
 'minimum_polygon_joint_radius_m',min(pitch./(2*sin(turn/2)))/1000,...
 'iterations',iteration,'scope','Kinematic visualization only; no force feedback');
end
