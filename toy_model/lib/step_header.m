function step_header(k, title, eqs)
%STEP_HEADER  Banner with the step number, title and the equations of the block.
line = repmat('=', 1, 78);
fprintf('\n%s\n  AB02 TOY MODEL | STEP %d | %s\n%s\n', line, k, upper(title), line);
for i = 1:numel(eqs), fprintf('    %s\n', eqs{i}); end
fprintf('%s\n', repmat('-', 1, 78));
end
