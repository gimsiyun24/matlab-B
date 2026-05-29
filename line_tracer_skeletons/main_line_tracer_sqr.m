clear; clc; close all;

%% Simulation Set up
dt = 0.05;
T = 80;
N = round(T/dt);

% Robot initial state
x = 1.0;
y = 0.8;
theta = 0;

wheel_base = 0.25;

% Sensor positions in robot frame [forward, side]
sensor_pos = [
    0.18,  0.12;   % left
    0.18,  0.06;
    0.18,  0.00;   % center
    0.18, -0.06;
    0.18, -0.12    % right
];

robot_log = zeros(N,3);

figure;

%% Simulation loop
for k = 1:N
    sensor = read_line_sensor(x, y, theta, sensor_pos);

    [vL, vR] = student_controller_sqr(sensor);

    vL = max(min(vL, 0.8), -0.8);
    vR = max(min(vR, 0.8), -0.8);

    v = (vR + vL)/2;
    omega = (vR - vL)/wheel_base;

    x = x + v*cos(theta)*dt;
    y = y + v*sin(theta)*dt;
    theta = theta + omega*dt;

    robot_log(k,:) = [x y theta];

    if mod(k,5) == 0
        clf;
        hold on; grid on; axis equal;

        draw_square_track();

        plot(robot_log(1:k,1), robot_log(1:k,2), 'b');

        draw_robot(x, y, theta, sensor_pos, sensor);

        xlim([0 6]);
        ylim([0 4]);
        title('MATLAB Square Line Tracer Simulation');
        xlabel('x');
        ylabel('y');

        drawnow;
    end
end

%% draw square track
function draw_square_track()
    x = [1 5 5 1 1];
    y = [1 1 3 3 1];

    plot(x, y, 'k', 'LineWidth', 3);
end

%% read line sesnor
function sensor = read_line_sensor(x, y, theta, sensor_pos)

    sensor = zeros(1, size(sensor_pos,1));

    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];

    for i = 1:size(sensor_pos,1)
        p = [x; y] + R * sensor_pos(i,:)';

        dist = distance_to_square(p(1), p(2));

        sensor(i) = min(dist / 0.12, 1);
    end
end

function d = distance_to_square(px, py)

    corners = [
        1 1;
        5 1;
        5 3;
        1 3;
        1 1
    ];

    d = inf;

    for i = 1:4
        x1 = corners(i,1);
        y1 = corners(i,2);
        x2 = corners(i+1,1);
        y2 = corners(i+1,2);

        d = min(d, point_to_segment_distance(px, py, x1, y1, x2, y2));
    end
end

%% point to segment distance
function d = point_to_segment_distance(px, py, x1, y1, x2, y2)

    A = [px - x1, py - y1];
    B = [x2 - x1, y2 - y1];

    t = dot(A, B) / dot(B, B);
    t = max(0, min(1, t));

    closest = [x1, y1] + t * B;

    d = sqrt((px - closest(1))^2 + (py - closest(2))^2);
end

%% draw robot
function draw_robot(x, y, theta, sensor_pos, sensor)

    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];

    body = [
        0.25  0.15;
        0.25 -0.15;
       -0.15 -0.15;
       -0.15  0.15
    ]';

    body_world = [x; y] + R*body;
    fill(body_world(1,:), body_world(2,:), [0.8 0.8 0.8]);

    front = [x; y] + R*[0.3; 0];
    plot([x front(1)], [y front(2)], 'r', 'LineWidth', 2);

    for i = 1:size(sensor_pos,1)
        p = [x; y] + R*sensor_pos(i,:)';

        if sensor(i) < 0.5
            plot(p(1), p(2), 'ko', 'MarkerFaceColor', 'k');
        else
            plot(p(1), p(2), 'ko', 'MarkerFaceColor', 'w');
        end
    end
end