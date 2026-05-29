function [vL, vR] = student_controller_sqr(sensor)
% sensor 값:
% sensor(1) = 가장 왼쪽 센서
% sensor(3) = 가운데 센서
% sensor(5) = 가장 오른쪽 센서
%
% 값이 작을수록 검은 선 위에 있음
% 값이 클수록 흰 바닥 위에 있음

    persistent last_error

    if isempty(last_error)
        last_error = 0;
    end

    weights = [2 1 0 -1 -2];
    line_strength = max(0, 1 - sensor);
    total_strength = sum(line_strength);

    if total_strength > 0.08
        error = sum(weights .* line_strength) / total_strength;
        last_error = error;

        turn = 0.25 * error;
        base_speed = 0.36 - 0.08 * min(abs(error), 2);

        vL = base_speed - turn;
        vR = base_speed + turn;
    else
        turn = 0.30 * sign(last_error);

        if turn == 0
            turn = 0.22;
        end

        vL = 0.12 - turn;
        vR = 0.12 + turn;
    end
end
