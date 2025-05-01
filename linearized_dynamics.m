function [A, B, E] = linearized_dynamics(params)
    g = params.g;
    m = params.m;
    
    if params.normalize
        A = [0 0 0 1 0 0;
             0 0 0 0 1 0;
             0 0 0 0 0 1;
             0 0 -1 0 0 0;
             0 0 0 0 0 0;
             0 0 0 0 0 0];
    
        B = [0 0;
             0 0;
             0 0;
             -1 0;
             1 0;
             0 1/params.Ibar];

        E = [0;0;0;1;0;0];
    else
        A = [0 0 0 1 0 0;
             0 0 0 0 1 0;
             0 0 0 0 0 1;
             0 0 -g 0 0 0;
             0 0 0 0 0 0;
             0 0 0 0 0 0];
    
        B = [0 0;
             0 0;
             0 0;
             0 0;
             1/m 0;
             0 1/params.I];

        E = [0;0;0;1/m;0;0];
    end
end
