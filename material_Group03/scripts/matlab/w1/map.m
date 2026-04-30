%% Clear all
clear all; clear figure; close all;

%% Load world elements
pots = load("pots.csv");
pot_radius = 0.3/2; % 0.2 m

%% Plot world
xlims = [-1.3, 7];
ylims = [-1.5 6.75];

figure(1);
xlim(xlims);
ylim(ylims);
xlabel("x-axis [m]")
ylabel("y-axis [m]")
axis equal;
hold on;

% Draw pots
for i=1:length(pots)
    x = pots(i, 1);
    y = pots(i, 2);
    circle(x,y,pot_radius);
end

% Plot origin
plot(0,0, "Marker", "x", "Color", "r");

%% Guide points

path = [
    -1  0;
    6.1 0;
    6.1 1.3;
    -1  1.3;
    -1  2.5;
    6.1 2.5;
    6.1 3.9;
    -1  3.9;
    -1  6;
];

plot(path(:, 1), path(:,2), "Color", "b", "Marker", "o")

%% Vector field
n = 100;
xpts = linspace(xlims(1), xlims(2), n);
ypts = linspace(ylims(1), ylims(2), n);

XX = zeros(100);
YY = zeros(100);
UU = zeros(100);
VV = zeros(100);

k = 1;

for i=1:length(xpts)
    for j=1:length(ypts)
        XX(i, j) = xpts(i);
        YY(i, j) = ypts(j);
        [UU(i, j), VV(i,j)] = create_vector(path, XX(i, j), YY(i, j));
    end
end

quiver(XX, YY, UU, VV)

%% Plot trajectory
z0 = [0 0];
z = z0;
dt = 0.05;

zlogs = [z0];
heading = [1, 0];

kangle = 1;

while norm(z-path(length(path),:)) > 0.5
    vp = zeros(1, 2);
    [vp(1), vp(2)] = create_vector(path, z(1), z(2));
    proj = vp*heading';
    
    if proj > 0
        vr = proj*heading;
    else
        vr = [0 0];
    end

    angle = atan2(heading(2), heading(1));
    angle = angle + sign(cross([heading 0],[vp 0])*[0 0 1]')*real(acos(proj/norm(vp)))*kangle;
    heading = [cos(angle) sin(angle)];
    
    z = z + vr*dt;
    zlogs(length(zlogs)+1,:) = z;
end

xtraj = zlogs(:, 1); ytraj = zlogs(:, 2);

plot(xtraj, ytraj, "Color", "r", "Marker", "diamond");
