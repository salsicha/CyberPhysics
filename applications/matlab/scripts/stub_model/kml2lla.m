% Convert kml 3D coordinates 1to 3 row matrix

function m = kml2mat(filename)

f = fopen(filename,'r');
while (~feof(f))
    tline = fgetl(f);
    expr = '<coordinates>';
    matches = regexp(tline,expr,'match');
    if length(matches)
        disp(matches)
        % Assume that the next line after <coordinates> is always the data
        eval( sprintf('a = [%s];',fgetl(f)) );
    end
end
fclose(f);
clear f

% reshape from 1xN into 3 x N/3
m = reshape(a,3,size(a,2)/3);