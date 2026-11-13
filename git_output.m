function [status, output] = git_output(root, arguments)
%GIT_OUTPUT Run git -C root <arguments> without a shell.
% Java's ProcessBuilder starts git directly. MATLAB's system() goes
% through cmd.exe, which could not find git in a MATLAB process launched
% with Start-Process on the Windows lab PC even though git was on PATH.
% arguments: a cell array of strings, or one string split on spaces.
if ischar(arguments) || isstring(arguments)
    arguments = cellstr(split(strtrim(string(arguments))))';
end
command = java.util.ArrayList();
for value = [{'git','-C',char(root)}, arguments]
    command.add(java.lang.String(char(value{1})));
end
builder = java.lang.ProcessBuilder(command);
builder.redirectErrorStream(true);
try
    process = builder.start();
catch exception
    status = -1;
    output = sprintf('git could not be started: %s', exception.message);
    return
end
reader = java.io.BufferedReader(java.io.InputStreamReader(process.getInputStream(), 'UTF-8'));
lines = cell(0,1);
while true
    line = reader.readLine();
    if isempty(line), break; end
    lines{end+1,1} = char(line); %#ok<AGROW>
end
output = strjoin(lines, newline);
status = process.waitFor();
end
