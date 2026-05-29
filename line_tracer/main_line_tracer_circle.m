clear; clc; close all;

%% Simulation Set up
dt = 0.05;
T = 60;
N = round(T/dt);

% Robot initial state: [x, y, theta]
x = 2;
y = -0.3;
theta = pi/2;

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
    % Sensor reading
    sensor = read_line_sensor(x, y, theta, sensor_pos);

    % 학생들이 작성하는 제어기
    [vL, vR] = student_controller_circle(sensor);

    % Safety limit
    vL = max(min(vL, 0.8), -0.8);
    vR = max(min(vR, 0.8), -0.8);

    % Differential drive model
    v = (vR + vL)/2;
    omega = (vR - vL)/wheel_base;

    x = x + v*cos(theta)*dt;
    y = y + v*sin(theta)*dt;
    theta = theta + omega*dt;

    robot_log(k,:) = [x y theta];

    % Plot
    if mod(k,5) == 0
        clf;
        hold on; grid on; axis equal;

        % Draw circular line
        th = linspace(0, 2*pi, 500);
        cx = 4;
        cy = 0;
        r = 2;

        plot(cx + r*cos(th), cy + r*sin(th), 'k', 'LineWidth', 3);

        % Draw robot trajectory
        plot(robot_log(1:k,1), robot_log(1:k,2), 'b');

        % Draw robot and sensors
        draw_robot(x, y, theta, sensor_pos, sensor);

        xlim([1 7]);
        ylim([-3 3]);
        title('MATLAB Circular Line Tracer Simulation');
        xlabel('x');
        ylabel('y');

        drawnow;
    end
end

%% read line sesnor
function sensor = read_line_sensor(x, y, theta, sensor_pos)

    sensor = zeros(1, size(sensor_pos,1));

    R = [cos(theta) -sin(theta);
         sin(theta)  cos(theta)];

    for i = 1:size(sensor_pos,1)
        p = [x; y] + R * sensor_pos(i,:)';

        dist = line_distance(p(1), p(2));

        % 검은 선 근처면 0, 멀면 1
        sensor(i) = min(dist / 0.12, 1);
    end
end

%% line distance
function d = line_distance(px, py)

    cx = 4;
    cy = 0;
    r = 2;

    dist_center = sqrt((px - cx)^2 + (py - cy)^2);

    % 원 둘레까지의 거리
    d = abs(dist_center - r);
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