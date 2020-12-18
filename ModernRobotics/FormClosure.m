function [result, k] = FormClosure(x,y, angles, A, b, f, beq)
    % function FormClosure check the given arguments of the grasp is 
    % form closure or not
    assert(size(x, 2) == size(y, 2));
    assert(size(x, 2) == size(angles, 2));
    
    num_contacts = size(x, 2);
    F = [];
    for i=1:num_contacts
        pt = [x(i), y(i), 0];
        ang = [cos(angles(i)), sin(angles(i)), 0];
        % calc a moment 
        G = cross(pt, ang);
        % calc a force
        F = [ [F]; G(3), ang(1), ang(2)];
    end
    k = [];
    Aeq = F';
    result = false;
    k = linprog(f, A, b, Aeq, beq);
    if k > 0
        result = true;
    end
end

