function assemble_gripper()
    % Ensambla el gripper 2D usando las tres piezas: base, hook y end
    % Cada pieza se une por sus agujeros definidos manualmente

    figure('Color','w'); hold on; axis equal; axis off;

    %% === 1. Coordenadas de los agujeros ===
    base_holes = [14, 8.5;
                  27, 8.5;
                  91, 8.5;
                 104, 8.5];      % agujeros base

    hook_holes = [4.5, 5; 50.5, 5];    % [agujero_izq, agujero_dcha]
    end_holes  = [28, 21.5; 28, 8.5];  % [superior, inferior]

    %% === 2. Dibujar base ===
    draw_gripper_base_2D();

    %% === 3. Dibujar los hooks ===
    % Suponemos que cada hook se une a un agujero de la base por su primer agujero
    for i = 1:size(base_holes,1)
        % Dibujar hook "izquierdo"
        draw_gripper_hook_2D_at(base_holes(i,:), hook_holes(1,:), 0);

        % Dibujar hook "derecho" (paralelo)
        draw_gripper_hook_2D_at(base_holes(i,:), hook_holes(1,:), 10); % offset lateral opcional
    end

    %% === 4. Dibujar el elemento final ===
    % El end se conecta con el extremo de los hooks (segundo agujero)
    % En este ejemplo colocamos el end de modo que su agujero inferior
    % coincida con el segundo agujero de los hooks del último par

    last_hook_right_hole = base_holes(end,:) + (hook_holes(2,:) - hook_holes(1,:));
    draw_gripper_end_2D_at(last_hook_right_hole, end_holes(2,:));
end


%% === Función auxiliar: mover y dibujar HOOK ===
function draw_gripper_hook_2D_at(target, hook_ref, offset)
    % Dibuja un hook desplazado para que el agujero hook_ref coincida con target
    if nargin < 3, offset = 0; end

    % Captura la geometría original del hook
    [xh, yh] = get_hook_outline();

    % Calcular desplazamiento
    dx = target(1) - hook_ref(1);
    dy = target(2) - hook_ref(2);

    % Aplicar desplazamiento
    xh = xh + dx;
    yh = yh + dy + offset;  % offset vertical para paralelo

    % Dibujar
    plot(xh, yh, 'k-', 'LineWidth', 1.5);
end


%% === Función auxiliar: mover y dibujar END ===
function draw_gripper_end_2D_at(target, end_ref)
    [xe, ye] = get_end_outline();

    dx = target(1) - end_ref(1);
    dy = target(2) - end_ref(2);

    xe = xe + dx;
    ye = ye + dy;

    plot(xe, ye, 'k-', 'LineWidth', 1.8);
end


%% === Función auxiliar: geometría de hook ===
function [x, y] = get_hook_outline()
    % Coordenadas simplificadas del hook
    x = [0, 50, 50, 0, 0];
    y = [0, 0, 10, 10, 0];
end


%% === Función auxiliar: geometría de end ===
function [x, y] = get_end_outline()
    x = [0, 20, 20, 32, 32, 24.5, 0, 0];
    y = [0, 0, 5, 5, 25, 25, 12.5, 0];
end
