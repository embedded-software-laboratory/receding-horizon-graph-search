function [x_globals,y_globals] = translate_global(yaw, x0, y0, x_locals, y_locals)
    
    c = cos(yaw);
    s = sin(yaw);
    
    x_globals = [c -s] * [x_locals ; y_locals] + x0;
    y_globals = [s  c] * [x_locals ; y_locals] + y0;
    
end
