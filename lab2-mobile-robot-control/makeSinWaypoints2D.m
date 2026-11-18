function path = makeSinWaypoints2D(x0, x1, dx, A, lambda, y0)
% makeSinWaypoints2D  Generates Nx2 waypoints [x y] along a sinusoid.
% x0..x1  : x-range (m)
% dx      : spacing between waypoints (m)
% A       : amplitude (m)
% lambda  : wavelength (m)
% y0      : y-offset (m)

    x = (x0:dx:x1)';                         % Nx1
    y = y0 + A * sin(2*pi*(x - x0)/lambda);  % Nx1

    path = [x, y];                           % Nx2
end