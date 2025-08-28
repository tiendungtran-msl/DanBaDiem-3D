clc
clear

%===========   Khai báo mục tiêu:  =============%
% Vị trí:
X_mt(1) = 30000;
Y_mt(1) = 40000;
Z_mt(1) = 10000;
% Tọa độ cực:
r_mt(1) = sqrt(X_mt(1)^2 + Y_mt(1)^2 + Z_mt(1)^2); % Cự li đến tâm tọa độ cực của mục tiêu
epsilon_mt(1) = atan2(sqrt(X_mt(1)^2 + Y_mt(1)^2), Z_mt(1)); % Góc tà mục tiêu (elevation angle)
beta_mt(1) = atan2(Y_mt(1), X_mt(1)); % Góc phương vị (azimuth angle)
epsilon_mt_dot(1) = 0;
beta_mt_dot(1) = 0;
% Vận tốc:
W_mt_theta(1) = 10; % Gia tốc pháp tuyến theo góc nghiêng
W_mt_phi(1) = 0;   % Gia tốc pháp tuyến theo góc phương vị
V_mt = 200;
Theta_mt(1) = -60 * pi/180; % Góc nghiêng quỹ đạo mục tiêu
Phi_mt(1) = 120 * pi/180; % Góc lệch quỹ đạo theo phương vị
Theta_mt_dot(1) = W_mt_theta(1) / V_mt;
Phi_mt_dot(1) = W_mt_phi(1) / V_mt;
Vx_mt(1) = V_mt * cos(Theta_mt(1)) * cos(Phi_mt(1));
Vy_mt(1) = V_mt * cos(Theta_mt(1)) * sin(Phi_mt(1));
Vz_mt(1) = V_mt * sin(Theta_mt(1));

%=============  Khai báo tên lửa:  ============%
% Vị trí:
X_tl(1) = 50;
Y_tl(1) = 0;
Z_tl(1) = 0;
% Tọa độ cực:
r_tl(1) = sqrt(X_tl(1)^2 + Y_tl(1)^2 + Z_tl(1)^2);    % Cự li tên lửa đến tâm tọa độ cực
epsilon_tl(1) = atan2(sqrt(X_tl(1)^2 + Y_tl(1)^2), Z_tl(1)); % Góc tà tên lửa
beta_tl(1) = atan2(Y_tl(1), X_tl(1)); % Góc phương vị tên lửa
epsilon_tl_dot(1) = 0;
beta_tl_dot(1) = 0;
% Vận tốc:
W_tl_theta(1) = 0; % Gia tốc pháp tuyến theo góc nghiêng ban đầu
W_tl_phi(1) = 0;   % Gia tốc pháp tuyến theo góc phương vị ban đầu
m = 3;             % Hệ số tỉ lệ giữa tốc độ tên lửa và tốc độ mục tiêu
V_tl = m * V_mt;   % Tốc độ tên lửa
Theta_tl(1) = 45 * pi/180; % Góc nghiêng quỹ đạo tên lửa ban đầu
Phi_tl(1) = -30 * pi/180;    % Góc lệch quỹ đạo theo phương vị ban đầu
Theta_tl_dot(1) = W_tl_theta(1) / V_tl;
Phi_tl_dot(1) = W_tl_phi(1) / V_tl;
Vx_tl(1) = V_tl * cos(Theta_tl(1)) * cos(Phi_tl(1));
Vy_tl(1) = V_tl * cos(Theta_tl(1)) * sin(Phi_tl(1));
Vz_tl(1) = V_tl * sin(Theta_tl(1));

%============ Các tham số: ===========%
dt = 0.02;  % Thời gian lấy mẫu
N = 20000;
i = 2;      % Khởi tạo điểm bắt đầu trong chu trình lặp
Kp = 0.5;     % Hệ số tỉ lệ của bộ điều khiển PD
Kd = 5;     % Hệ số đạo hàm của bộ điều khiển PD
L = 30;     % Cự ly kích hoạt ngòi nổ khi cách mục tiêu L(m)
W_max = 14 * 9.8;  % Giới hạn gia tốc dựa trên quá tải tạo được tối đa của tên lửa
h_eps(1) = 0;   % Khởi tạo sai số theo góc tà tại i=1
h_beta(1) = 0;  % Khởi tạo sai số theo góc phương vị tại i=1

%======== Bắt đầu dẫn theo phương pháp 3 điểm với bộ điều khiển PD =========%
while (i <= N)
    %--------- Đo các tham số của mục tiêu tại thời điểm i: ----------%
    % Vị trí:
    X_mt(i) = X_mt(i-1) + Vx_mt(i-1) * dt;
    Y_mt(i) = Y_mt(i-1) + Vy_mt(i-1) * dt;
    Z_mt(i) = Z_mt(i-1) + Vz_mt(i-1) * dt;
    % Tọa độ cực:
    r_mt(i) = sqrt(X_mt(i)^2 + Y_mt(i)^2 + Z_mt(i)^2);
    epsilon_mt(i) = atan2(sqrt(X_mt(i)^2 + Y_mt(i)^2), Z_mt(i));
    beta_mt(i) = atan2(Y_mt(i), X_mt(i));
    epsilon_mt_dot(i) = (epsilon_mt(i) - epsilon_mt(i-1)) / dt;
    beta_mt_dot(i) = (beta_mt(i) - beta_mt(i-1)) / dt;
    % Vận tốc:
    W_mt_theta(i) = W_mt_theta(1); % Giữ gia tốc pháp tuyến cố định
    W_mt_phi(i) = W_mt_phi(1);     % Giữ gia tốc phương vị cố định
    Theta_mt_dot(i) = W_mt_theta(i) / V_mt;
    Phi_mt_dot(i) = W_mt_phi(i) / V_mt;
    Theta_mt(i) = Theta_mt(i-1) + Theta_mt_dot(i-1) * dt;
    Phi_mt(i) = Phi_mt(i-1) + Phi_mt_dot(i-1) * dt;
    Vx_mt(i) = V_mt * cos(Theta_mt(i)) * cos(Phi_mt(i));
    Vy_mt(i) = V_mt * cos(Theta_mt(i)) * sin(Phi_mt(i));
    Vz_mt(i) = V_mt * sin(Theta_mt(i));
    
    %--------- Đo các tham số của tên lửa tại thời điểm i: -----------%
    % Vị trí:
    X_tl(i) = X_tl(i-1) + Vx_tl(i-1) * dt;
    Y_tl(i) = Y_tl(i-1) + Vy_tl(i-1) * dt;
    Z_tl(i) = Z_tl(i-1) + Vz_tl(i-1) * dt;
    % Tọa độ cực:
    r_tl(i) = sqrt(X_tl(i)^2 + Y_tl(i)^2 + Z_tl(i)^2);
    epsilon_tl(i) = atan2(sqrt(X_tl(i)^2 + Y_tl(i)^2), Z_tl(i));
    beta_tl(i) = atan2(Y_tl(i), X_tl(i));
    epsilon_tl_dot(i) = (epsilon_tl(i) - epsilon_tl(i-1)) / dt;
    beta_tl_dot(i) = (beta_tl(i) - beta_tl(i-1)) / dt;
    % Vận tốc:
    Theta_tl_dot(i) = W_tl_theta(i-1) / V_tl;
    Phi_tl_dot(i) = W_tl_phi(i-1) / V_tl;
    Theta_tl(i) = Theta_tl(i-1) + Theta_tl_dot(i-1) * dt;
    Phi_tl(i) = Phi_tl(i-1) + Phi_tl_dot(i-1) * dt;
    Vx_tl(i) = V_tl * cos(Theta_tl(i)) * cos(Phi_tl(i));
    Vy_tl(i) = V_tl * cos(Theta_tl(i)) * sin(Phi_tl(i));
    Vz_tl(i) = V_tl * sin(Theta_tl(i));

    %--------- Tính toán lệnh điều khiển với bộ PD cho hai mặt phẳng: -----------%
    % Sai số theo góc tà (epsilon)
    h_eps(i) = r_tl(i) * (epsilon_mt(i) - epsilon_tl(i));
    h_eps_dot = (h_eps(i) - h_eps(i-1)) / dt;
    W_tl_theta(i) = Kp * h_eps(i) + Kd * h_eps_dot;
    
    % Sai số theo góc phương vị (beta)
    h_beta(i) = r_tl(i) * (beta_mt(i) - beta_tl(i));
    h_beta_dot = (h_beta(i) - h_beta(i-1)) / dt;
    W_tl_phi(i) = Kp * h_beta(i) + Kd * h_beta_dot;
    
%     % Giới hạn gia tốc
%     W_tl_theta(i) = min(max(W_tl_theta(i), -W_max), W_max);
%     W_tl_phi(i) = min(max(W_tl_phi(i), -W_max), W_max);
    
    % Kiểm tra xem tên lửa đã bắn trúng mục tiêu chưa
    if sqrt((X_tl(i)-X_mt(i))^2 + (Y_tl(i)-Y_mt(i))^2 + (Z_tl(i)-Z_mt(i))^2) < L
        disp(['Tên lửa bắn trúng mục tiêu tại bước thứ ', num2str(i), ' (thời gian = ', num2str(i*dt), ' giây)']);
        break;
    end    
    i = i + 1;
end

%================= Vẽ quỹ đạo ===============%
figure;
plot3(X_mt, Y_mt, Z_mt, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Quỹ đạo mục tiêu');
hold on;
plot3(X_tl, Y_tl, Z_tl, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Quỹ đạo tên lửa');
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Quỹ đạo của mục tiêu và tên lửa theo phương pháp 3 điểm');
legend;
axis;

%================= Hiệu ứng điểm chạy với đường ngắm ==============%
h_mt = plot3(NaN, NaN, NaN, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'Mục tiêu');
h_tl = plot3(NaN, NaN, NaN, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Tên lửa');
h_ray_mt = plot3([0 0], [0 0], [0 0], 'b--', 'LineWidth', 0.25, 'DisplayName', 'Đường ngắm mục tiêu'); % Tia tới mục tiêu
h_ray_tl = plot3([0 0], [0 0], [0 0], 'r--', 'LineWidth', 0.25, 'DisplayName', 'Đường ngắm tên lửa'); % Tia tới tên lửa

for j = 1:15:i
    % Cập nhật vị trí mục tiêu và tên lửa
    set(h_mt, 'XData', X_mt(j), 'YData', Y_mt(j), 'ZData', Z_mt(j));
    set(h_tl, 'XData', X_tl(j), 'YData', Y_tl(j), 'ZData', Z_tl(j));
    
    % Cập nhật đường ngắm từ gốc (0,0,0) đến mục tiêu và tên lửa
    set(h_ray_mt, 'XData', [0 X_mt(j)], 'YData', [0 Y_mt(j)], 'ZData', [0 Z_mt(j)]);
    set(h_ray_tl, 'XData', [0 X_tl(j)], 'YData', [0 Y_tl(j)], 'ZData', [0 Z_tl(j)]);

    drawnow limitrate;
    pause(1e-5); % Điều chỉnh tốc độ chạy (giây)
end

% Chạy tới điểm cuối
set(h_mt, 'XData', X_mt(i), 'YData', Y_mt(i), 'ZData', Z_mt(i));
set(h_tl, 'XData', X_tl(i), 'YData', Y_tl(i), 'ZData', Z_tl(i));
set(h_ray_mt, 'XData', [0 X_mt(i)], 'YData', [0 Y_mt(i)], 'ZData', [0 Z_mt(i)]);
set(h_ray_tl, 'XData', [0 X_tl(i)], 'YData', [0 Y_tl(i)], 'ZData', [0 Z_tl(i)]);
drawnow limitrate;
pause(1e-5);