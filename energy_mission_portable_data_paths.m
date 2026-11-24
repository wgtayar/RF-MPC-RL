function [cfg,report]=energy_mission_portable_data_paths(cfg,sourceRoot)
%ENERGY_MISSION_PORTABLE_DATA_PATHS Re-point stored data-file paths of a checkpoint
% configuration that do not exist on this machine to the identical file in the
% source checkout, after verifying its content.
% A checkpoint's configuration stores absolute paths of two data files from the
% machine it was trained on: cfg.PROXY.kneeCsv ('Symmetric Knee Torques.csv') and
% cfg.BATTERY.model_options.parameter_source ('our params.xlsx'). On another
% machine those paths do not exist. Each missing path is replaced by
% fullfile(sourceRoot,<file name>) only if that file's content, with CR bytes
% removed (Windows checkouts convert LF to CRLF), has the pinned SHA-256 of the
% file in every RF-MPC-RL commit used for training and evaluation. Existing
% paths are left untouched. report lists every substitution.
pinned=struct('name',{'Symmetric Knee Torques.csv','our params.xlsx'}, ...
    'sha256',{'d7d7cc971f416ece3cc8897497f2fda42bd56edd1f4372d2af207be9b4290965', ...
              '18ec7700b630f1d355b22eebabfc4ff45779efdd25382b5d244ca8d66f1ee975'}, ...
    'field',{{'PROXY','kneeCsv'},{'BATTERY','model_options','parameter_source'}});
report=struct('field',{},'stored_path',{},'used_path',{},'sha256',{});
for k=1:numel(pinned)
    f=pinned(k).field;
    if ~localHas(cfg,f), continue; end
    stored=char(getfield(cfg,f{:})); %#ok<GFLD>
    if isfile(stored), continue; end
    parts=regexp(stored,'[\\/]','split');
    assert(strcmp(parts{end},pinned(k).name),'portablePaths:Name', ...
        'Stored path %s does not name %s.',stored,pinned(k).name);
    candidate=fullfile(sourceRoot,pinned(k).name);
    assert(isfile(candidate),'portablePaths:Missing','Source copy missing: %s',candidate);
    sha=localShaWithoutCR(candidate);
    if contains(pinned(k).name,'.xlsx'), sha=sha256_file(candidate); end   % binary: exact bytes
    assert(strcmp(sha,pinned(k).sha256),'portablePaths:Content', ...
        'Content of %s (%s) differs from the pinned %s.',candidate,sha,pinned(k).sha256);
    cfg=setfield(cfg,f{:},candidate); %#ok<SFLD>
    report(end+1)=struct('field',strjoin(f,'.'),'stored_path',stored,'used_path',candidate,'sha256',sha); %#ok<AGROW>
end
end

function tf=localHas(s,f)
tf=true;
for i=1:numel(f)
    if ~isstruct(s) || ~isfield(s,f{i}), tf=false; return; end
    s=s.(f{i});
end
end

function sha=localShaWithoutCR(path)
fid=fopen(path,'r');bytes=fread(fid,Inf,'*uint8');fclose(fid);
bytes(bytes==13)=[];
md=java.security.MessageDigest.getInstance('SHA-256');
d=typecast(md.digest(bytes),'uint8');
sha=lower(reshape(dec2hex(d,2).',1,[]));
end
