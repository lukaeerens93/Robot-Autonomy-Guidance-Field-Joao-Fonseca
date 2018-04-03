# Guidance for the robot
import math
import numpy as np

def GUIDANCE(Image_W, Image_L, Scaling, Target, H_Obs, S_Obs, State0, Clearance_Target, t):

    # Select how fast the field should fall(bigger Dispersion_rate, slower decay)
    # Dispersion_rate = Max_size ^ 2 * 120;
    # Hard_Constant = 100;
    # Soft_Constant = 100;

    Dispersion_rate = math.pow(Clearance_Target , 2) / math.log(2)
    Hard_Constant = 1
    Soft_Constant = 1

    # Decode state0
    x0 = State0[0]
    y0 = State0[1]

    vx0 = State0[3]
    vy0 = State0[4]

    # Decode Target
    x_target = Target[0]
    y_target = Target[1]

    # Target
    Target_Constant = 0.01
    Target_Field = -Target_Constant*[x0 - x_target, y0 - y_target]/((x0 - x_target)^2 + (y0 - y_target)^2)^(1/2)






    # HardObstacles
    [rows, columns] = H_Obs.size

    if vx0 == 0 and vy0 == 0:
        check = 1
    else:
        V_X = vx0 * (H_Obs[0][0] - x0)
        V_Y = vy0 * (H_Obs[1][0] - y0)
        check = max( 0, np.sign(V_X + V_Y) )


        Direction = -sign((-x0 + x_H_Obs(1)) * (-y0 + y_target) - (-y0 + y_H_Obs(1)) * (-x0 + x_target)) * check;

        if showGraph == 1 & & RandshowGraph == 1
            u_Hard = Hard_Constant * Direction * (-y + y_H_Obs(1)). * exp(
                -((x - x_H_Obs(1)). * (x - x_H_Obs(1)) + (y - y_H_Obs(1)). * (y - y_H_Obs(1))) / Dispersion_rate);
            v_Hard = Hard_Constant * Direction * (x - x_H_Obs(1)). * exp(
                -((x - x_H_Obs(1)). * (x - x_H_Obs(1)) + (y - y_H_Obs(1)). * (y - y_H_Obs(1))) / Dispersion_rate);
        end
        Hard_Fieldx = Hard_Constant * Direction * (-y0 + y_H_Obs(1)). * exp(
            -((x0 - x_H_Obs(1)). * (x0 - x_H_Obs(1)) + (y0 - y_H_Obs(1)). * (y0 - y_H_Obs(1))) / Dispersion_rate);
        Hard_Fieldy = Hard_Constant * Direction * (x0 - x_H_Obs(1)). * exp(
            -((x0 - x_H_Obs(1)). * (x0 - x_H_Obs(1)) + (y0 - y_H_Obs(1)). * (y0 - y_H_Obs(1))) / Dispersion_rate);

        for i=2:columns
        if vx0 == 0 & & vy0 == 0
            check = 1;
        else
            check = max(0, sign(vx0 * (x_H_Obs(i) - x0) + vy0 * (y_H_Obs(i) - y0)));
        end
        Direction = -sign((-x0 + x_H_Obs(i)) * (-y0 + y_target) - (-y0 + y_H_Obs(i)) * (-x0 + x_target)) * check;

        if showGraph == 1 & & RandshowGraph == 1
            u_Hard = u_Hard + Hard_Constant * Direction * (-y + y_H_Obs(i)). * exp(
                -((x - x_H_Obs(i)). * (x - x_H_Obs(i)) + (y - y_H_Obs(i)). * (y - y_H_Obs(i))) / Dispersion_rate);
            v_Hard = v_Hard + Hard_Constant * Direction * (x - x_H_Obs(i)). * exp(
                -((x - x_H_Obs(i)). * (x - x_H_Obs(i)) + (y - y_H_Obs(i)). * (y - y_H_Obs(i))) / Dispersion_rate);
        end
        Hard_Fieldx = Hard_Fieldx + Hard_Constant * Direction * (-y0 + y_H_Obs(i)). * exp(
            -((x0 - x_H_Obs(i)). * (x0 - x_H_Obs(i)) + (y0 - y_H_Obs(i)). * (y0 - y_H_Obs(i))) / Dispersion_rate);
        Hard_Fieldy = Hard_Fieldy + Hard_Constant * Direction * (x0 - x_H_Obs(i)). * exp(
            -((x0 - x_H_Obs(i)). * (x0 - x_H_Obs(i)) + (y0 - y_H_Obs(i)). * (y0 - y_H_Obs(i))) / Dispersion_rate);
    end
    Hard_Field = [Hard_Fieldx, Hard_Fieldy];
    % figure
    % quiver(x, y, u_Hard, v_Hard, 'color', [0 1 0])

    % SoftObstacles
    [rows, columns] = size(y_S_Obs);

    if vx0 == 0 & & vy0 == 0
        check = 1;
    else
        check = max(0, sign(vx0 * (x_S_Obs(1) - x0) + vy0 * (y_S_Obs(1) - y0)));
    end
    Direction = -sign((-x0 + x_S_Obs(1)) * (-y0 + y_target) - (-y0 + y_S_Obs(1)) * (-x0 + x_target)) * check;

    if showGraph == 1 & & RandshowGraph == 1
        u_Soft = Soft_Constant * Direction * (-y + y_S_Obs(1)). * exp(
            -((x - x_S_Obs(1)). * (x - x_S_Obs(1)) + (y - y_S_Obs(1)). * (y - y_S_Obs(1))) / Dispersion_rate);
        v_Soft = Soft_Constant * Direction * (x - x_S_Obs(1)). * exp(
            -((x - x_S_Obs(1)). * (x - x_S_Obs(1)) + (y - y_S_Obs(1)). * (y - y_S_Obs(1))) / Dispersion_rate);
    end
    Soft_Fieldx = Soft_Constant * Direction * (-y0 + y_S_Obs(1)). * exp(
        -((x0 - x_S_Obs(1)). * (x0 - x_S_Obs(1)) + (y0 - y_S_Obs(1)). * (y0 - y_S_Obs(1))) / Dispersion_rate);
    Soft_Fieldy = Soft_Constant * Direction * (x0 - x_S_Obs(1)). * exp(
        -((x0 - x_S_Obs(1)). * (x0 - x_S_Obs(1)) + (y0 - y_S_Obs(1)). * (y0 - y_S_Obs(1))) / Dispersion_rate);

    for i=2:columns
    if vx0 == 0 & & vy0 == 0
        check = 1;
    else
        check = max(0, sign(vx0 * (x_S_Obs(i) - x0) + vy0 * (y_S_Obs(i) - y0)));
    end
    Direction = -sign((-x0 + x_S_Obs(i)) * (-y0 + y_target) - (-y0 + y_S_Obs(i)) * (-x0 + x_target)) * check;

    if showGraph == 1 & & RandshowGraph == 1
        u_Soft = u_Soft + Soft_Constant * Direction * (-y + y_S_Obs(i)). * exp(
            -((x - x_S_Obs(i)). * (x - x_S_Obs(i)) + (y - y_S_Obs(i)). * (y - y_S_Obs(i))) / Dispersion_rate);
        v_Soft = v_Soft + Soft_Constant * Direction * (x - x_S_Obs(i)). * exp(
            -((x - x_S_Obs(i)). * (x - x_S_Obs(i)) + (y - y_S_Obs(i)). * (y - y_S_Obs(i))) / Dispersion_rate);
    end
    Soft_Fieldx = Soft_Fieldx + Soft_Constant * Direction * (-y0 + y_S_Obs(i)). * exp(
        -((x0 - x_S_Obs(i)). * (x0 - x_S_Obs(i)) + (y0 - y_S_Obs(i)). * (y0 - y_S_Obs(i))) / Dispersion_rate);
    Soft_Fieldy = Soft_Fieldy + Soft_Constant * Direction * (x0 - x_S_Obs(i)). * exp(
        -((x0 - x_S_Obs(i)). * (x0 - x_S_Obs(i)) + (y0 - y_S_Obs(i)). * (y0 - y_S_Obs(i))) / Dispersion_rate);


end
Soft_Field = [Soft_Fieldx, Soft_Fieldy];

% Open
window
if t == 0
    figure(1)
end

if showGraph == 1 & & RandshowGraph == 1
    % Update
    around
    90 % of
    iterations
    if rand > .1 | | t == 0
        quiver(x, y, u_Target + u_Soft + u_Hard, v_Target + v_Soft + v_Hard, 'color', [0 0 1])
    end
    hold
    off
end