function draw_gripper_end_2D()
    figure('Color','w'); hold on; axis equal; axis off;

    % --- Contorno del polígono ---
    x = [0, 20, 20, 32, 32, 24.5, 0, 0];
    y = [0, 0, 5, 5, 25, 25, 12.5, 0];
    plot(x, y, 'k-', 'LineWidth', 1.8);

    % --- Agujeros ---
    r = 1.5; % radio = Ø3/2
    theta = linspace(0, 2*pi, 200);

    % Centro 1
    xc1 = 28; yc1 = 8.5;
    plot(xc1 + r*cos(theta), yc1 + r*sin(theta), 'k-', 'LineWidth', 1.5);

    % Centro 2
    xc2 = 28; yc2 = 21.5;
    plot(xc2 + r*cos(theta), yc2 + r*sin(theta), 'k-', 'LineWidth', 1.5);

    % Opcional: ajusta límites para que quede bien centrado
    xlim([-2 35]); ylim([-2 28]);
end