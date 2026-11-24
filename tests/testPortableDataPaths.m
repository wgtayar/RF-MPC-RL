classdef testPortableDataPaths < matlab.unittest.TestCase
    % Stored data-file paths of a checkpoint configuration are re-pointed to the
    % verified identical files of the checkout only when they do not exist.
    properties
        Source
        Folder
    end
    methods(TestClassSetup)
        function prepare(tc)
            tc.Source=bootstrap_RF_MPC_RL();
            f=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);tc.Folder=f.Folder;
        end
    end
    methods(Test)
        function missingPathsAreRepointedAndReported(tc)
            cfg.PROXY.kneeCsv='/home/other/machine/Symmetric Knee Torques.csv';
            cfg.BATTERY.model_options.parameter_source='D:\other\our params.xlsx';
            [c,r]=energy_mission_portable_data_paths(cfg,tc.Source);
            tc.verifyEqual(c.PROXY.kneeCsv,fullfile(tc.Source,'Symmetric Knee Torques.csv'));
            tc.verifyEqual(c.BATTERY.model_options.parameter_source,fullfile(tc.Source,'our params.xlsx'));
            tc.verifyEqual(numel(r),2);
            tc.verifyEqual(r(1).stored_path,'/home/other/machine/Symmetric Knee Torques.csv');
        end

        function existingPathsAreUntouched(tc)
            p=fullfile(tc.Source,'Symmetric Knee Torques.csv');
            cfg.PROXY.kneeCsv=p;
            [c,r]=energy_mission_portable_data_paths(cfg,tc.Source);
            tc.verifyEqual(c.PROXY.kneeCsv,p);
            tc.verifyEmpty(r);
        end

        function crlfCopyIsAcceptedAndChangedContentRejected(tc)
            src=fullfile(tc.Folder,'crlf');mkdir(src);
            txt=fileread(fullfile(tc.Source,'Symmetric Knee Torques.csv'));
            fid=fopen(fullfile(src,'Symmetric Knee Torques.csv'),'w');fwrite(fid,strrep(txt,newline,[char(13) newline]));fclose(fid);
            cfg.PROXY.kneeCsv='/x/Symmetric Knee Torques.csv';
            c=energy_mission_portable_data_paths(cfg,src);
            tc.verifyEqual(c.PROXY.kneeCsv,fullfile(src,'Symmetric Knee Torques.csv'));
            fid=fopen(fullfile(src,'Symmetric Knee Torques.csv'),'a');fwrite(fid,'1');fclose(fid);
            tc.verifyError(@()energy_mission_portable_data_paths(cfg,src),'portablePaths:Content');
        end
    end
end
