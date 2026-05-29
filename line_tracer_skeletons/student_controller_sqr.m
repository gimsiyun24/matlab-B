function [vL, vR] = student_controller_sqr(sensor)
% sensor 값:
% sensor(1) = 가장 왼쪽 센서
% sensor(3) = 가운데 센서
% sensor(5) = 가장 오른쪽 센서
%
% 값이 작을수록 검은 선 위에 있음
% 값이 클수록 흰 바닥 위에 있음

    base_speed = 0.35;

    left  = sensor(1);
    center = sensor(3);
    right = sensor(5);


    % 예시: 매우 엉성한 제어기
    %-----------------------------------------------------------------
    threshold = 0.8;

    if center < threshold
        % 가운데 센서가 선을 보면 직진
        vL = base_speed;
        vR = base_speed;

    elseif left < threshold
        % 왼쪽 센서가 선을 보면 왼쪽으로 회전
        % 회전은 되지만 조금 과격해서 흔들림
        vL = 0.10;
        vR = 0.48;

    elseif right < threshold
        % 오른쪽 센서가 선을 보면 오른쪽으로 회전
        vL = 0.48;
        vR = 0.10;

    else
        % 선을 잠깐 놓쳤을 때는 천천히 직진
        vL = 0.20;
        vR = 0.20;
    end


end