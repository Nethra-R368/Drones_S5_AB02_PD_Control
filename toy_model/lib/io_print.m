function io_print(kind, name, value, unit, desc)
%IO_PRINT  Prints one INPUT / OUTPUT / CHECK line: name, size, unit, meaning and value.
%   kind : 'IN' | 'OUT' | 'CHK'
sz = sprintf('%dx%d', size(value, 1), size(value, 2));
fprintf('  [%-3s] %-12s %-7s %-10s %s\n', kind, name, sz, unit, desc);
if ~(isnumeric(value) || islogical(value)), return, end
value = double(value);
if isscalar(value)
    fprintf('          = %.6g\n', value);
elseif isvector(value) && numel(value) <= 12
    fprintf('          = [%s]''\n', strjoin(arrayfun(@(x) sprintf('% .4f', x), value(:)', 'UniformOutput', false), ' '));
elseif size(value, 1) <= 4 && size(value, 2) <= 6
    for i = 1:size(value, 1)
        fprintf('          | %s |\n', strjoin(arrayfun(@(x) sprintf('% .4f', x), value(i,:), 'UniformOutput', false), ' '));
    end
else
    fprintf('          (array: min % .4g, max % .4g, norm %.4g)\n', min(value(:)), max(value(:)), norm(value(:)));
end
end
