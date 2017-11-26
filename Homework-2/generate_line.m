function [ line_x, line_y ] = generate_line( x0, x1, y0, y1 )

    dx = abs(x1 - x0);
    dy = abs(y1 - y0);
    dx2 = x1 - x0;
    dy2 = y1 - y0;

    s = abs(dy) > abs(dx);

    if (s) 
        dx2 = dx;
        dx = dy;
        dy = dx2;
    end

    inc1 = 2 * dy;
    d = inc1 - dx;
    inc2 = d - dx;

    line_x = [x0];
    line_y = [y0];

    count = 2; 
    while x0 ~= x1 || y0 ~= y1
        if (s)
            y0= y0 + sign(dy2); 
        else
            x0 =x0 + sign(dx2);
        end
        
        if (d < 0)
            d = d + inc1;
        else 
          d = d +  inc2;
          if (s) 
              x0 = x0 + sign(dx2); 
          else
              y0 = y0 + sign(dy2);
          end
        end

       
        line_x(count) = x0;
        line_y(count) = y0;
        
        count = count + 1;
    end

end

