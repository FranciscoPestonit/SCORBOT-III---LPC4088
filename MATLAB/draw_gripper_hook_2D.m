function draw_gripper_hook_2D()
    % === Dimensiones principales (mm) ===
    L = 55;     % largo total
    H = 10;     % alto
    d = 3;      % diámetro agujeros
    r = d/2;    % radio agujeros

    % === Posiciones de los agujeros ===
    x_centros = [-L/2 + 4.5, L/2 - 4.5];
    y_centros = 0; % centrados verticalmente

    % === Configuración de la figura ===
    figure('Color','w');
    hold on; axis equal; axis off;

    % === Dibujar contorno ===
    rectX = [-L/2,  L/2,  L/2, -L/2, -L/2];
    rectY = [-H/2, -H/2,  H/2,  H/2, -H/2];
    plot(rectX, rectY, 'k-', 'LineWidth', 1.2);

    % === Dibujar agujeros ===
    theta = linspace(0, 2*pi, 100);
    for i = 1:numel(x_centros)
        xh = r*cos(theta) + x_centros(i);
        yh = r*sin(theta);
        plot(xh, yh, 'k-', 'LineWidth', 1);
    end

    % === Ajustar vista ===
    xlim([-L/2-5, L/2+5]);
    ylim([-H, H]);
end
