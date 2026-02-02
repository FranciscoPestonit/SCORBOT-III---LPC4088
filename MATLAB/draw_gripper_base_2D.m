function draw_base_outline()
    % === Dimensiones principales (mm) ===
    L = 118;   % largo total
    H = 17;    % alto
    d = 3;     % diámetro agujeros
    r = d/2;   % radio agujeros

    % === Posiciones de los agujeros ===
    x_centros = [-L/2 + 14, -L/2 + 27, L/2 - 27, L/2 - 14];
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
    xlim([-L/2-10, L/2+10]);
    ylim([-H, H]);
end
