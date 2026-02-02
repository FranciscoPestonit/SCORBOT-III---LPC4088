function draw_gripper(theta)
    % theta: ángulo de apertura (en grados)

    clf; hold on; axis equal; axis([-80 80 -80 20]);
    title(['Apertura: ' num2str(theta) '°']);

    %% BASE
    base_w = 60; base_h = 10;
    fill([-base_w/2 base_w/2 base_w/2 -base_w/2], ...
         [-base_h/2 -base_h/2 base_h/2 base_h/2], [0.6 0.6 0.6]);

    %% BRAZOS (izq y der)
    L = 40; % longitud de los brazos
    pivot_y = -base_h/2;
    R_left  = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
    R_right = [cosd(-theta) -sind(-theta); sind(-theta) cosd(-theta)];

    % Brazo izquierdo
    p1 = [0; pivot_y];
    p2 = p1 + R_left * [0; -L];
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'r-', 'LineWidth', 4);

    % Brazo derecho
    p1 = [0; pivot_y];
    p2 = p1 + R_right * [0; -L];
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'b-', 'LineWidth', 4);

    %% GARRAS (hooks)
    % Garras al final de cada brazo
    hook_len = 15; hook_ang = 30; % longitud y orientación
    R_hook_L = R_left * [cosd(-hook_ang) -sind(-hook_ang); sind(-hook_ang) cosd(-hook_ang)];
    R_hook_R = R_right * [cosd(hook_ang) -sind(hook_ang); sind(hook_ang) cosd(hook_ang)];

    hook_L_end = p2 + R_hook_L * [0; -hook_len];
    hook_R_end = p2 + R_hook_R * [0; -hook_len];

    plot([p2(1) hook_L_end(1)], [p2(2) hook_L_end(2)], 'k-', 'LineWidth', 3);
    plot([-p2(1) -hook_L_end(1)], [p2(2) hook_L_end(2)], 'k-', 'LineWidth', 3);

    drawnow;
end
