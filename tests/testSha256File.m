classdef testSha256File < matlab.unittest.TestCase
    properties
        Root
    end
    methods (TestMethodSetup)
        function setup(testCase)
            fixture = testCase.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            testCase.Root = fixture.Folder;
        end
    end
    methods (Test)
        function emptyFile(testCase)
            path = fullfile(testCase.Root,'empty.bin');
            localBytes(path,zeros(0,1,'uint8'));
            testCase.verifyEqual(sha256_file(path), ...
                'e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855');
        end
        function knownText(testCase)
            path = fullfile(testCase.Root,'abc.bin');
            localBytes(path,uint8('abc'));
            testCase.verifyEqual(sha256_file(path), ...
                'ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad');
        end
        function exactBufferBoundary(testCase)
            path = fullfile(testCase.Root,'buffer.bin');
            localBytes(path,zeros(1024*1024,1,'uint8'));
            testCase.verifyEqual(sha256_file(path), ...
                '30e14955ebf1352266dc2ff8067e68104607e750abb9d3b36582b8af909fcb58');
        end
    end
end

function localBytes(path,bytes)
    fid = fopen(path,'wb');
    assert(fid>=0);
    cleanup = onCleanup(@() fclose(fid));
    fwrite(fid,bytes,'uint8');
end
