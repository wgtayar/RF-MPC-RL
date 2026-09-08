function digest = sha256_file(filename)
%sha256_file Hash file bytes in bounded memory, independent of MAT contents.
    fid = fopen(filename, 'rb');
    if fid < 0
        error('sha256_file:OpenFailed', 'Cannot open %s.', filename);
    end
    cleanup = onCleanup(@() fclose(fid));
    engine = java.security.MessageDigest.getInstance('SHA-256');
    while ~feof(fid)
        bytes = fread(fid, 1024*1024, '*uint8');
        if isempty(bytes)
            break
        end
        engine.update(bytes);
    end
    digest = lower(reshape(dec2hex(typecast(engine.digest(), 'uint8'), 2).', 1, []));
end
