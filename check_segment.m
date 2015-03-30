function valid = check_segment(a, b, obstacles)

valid = all(cellfun(@(obs) isempty(polyxpoly([a(1) b(1)], [a(2) b(2)], obs(1,:), obs(2,:))), obstacles));