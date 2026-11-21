classdef testEnergyMissionPlantMismatch < matlab.unittest.TestCase
    % Plant battery mismatch (protocol A4, evaluation only) in the shift
    % evaluation: R1 scales the plant cells' resistances, R2 replaces the
    % plant cell capacity, R3 offsets the EKF's initial SOC estimate. The EKF
    % model, observation, reward scales and policies stay nominal; defaults
    % reproduce the nominal plant. One short fixed-speed mission per case
    % (plumbing and direction of the effect only).
    properties
        Folder
        Source
        Options
        Nominal
    end
    methods(TestClassSetup)
        function prepare(tc)
            tc.Source=bootstrap_RF_MPC_RL();
            [status,text]=git_output(tc.Source,'status --porcelain');
            tc.assumeTrue(status==0 && isempty(strtrim(text)),'Evaluation needs a clean committed source tree.');
            fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);
            tc.Folder=fixture.Folder;
            tc.Options=struct('policy','fixed','speed_m_s',1.0,'start_soc',.95,'min_start_soc',.9495, ...
                'smoke_distance_m',3,'smoke_duration_s',10);
            tc.Nominal=tc.shift('nominal',struct());
        end
    end
    methods(Test)
        function defaultsReproduceTheNominalPlant(tc)
            n=tc.Nominal;m=n.missions;
            tc.verifyEqual(numel(m),1);tc.verifyTrue(m.mission_complete);
            tc.verifyTrue(n.plant.nominal);tc.verifyTrue(m.plant_nominal);
            tc.verifyEqual([m.plant_resistance_scale,m.plant_capacity_Ah,m.plant_soc_estimate_offset],[1 2 0]);
            manifest=jsondecode(fileread(fullfile(tc.Folder,'nominal','manifest.json')));
            tc.verifyTrue(manifest.plant.nominal,'manifest.json records the nominal plant.');
            dataset=fullfile(tc.Folder,'nominal','mission_01');
            saved=load(fullfile(dataset,'manifest','configuration.mat'),'cfg');cfg=saved.cfg;
            tc.verifyFalse(isfield(cfg.BATTERY,'plant_mismatch'),'Defaults declare no mismatch.');
            saved=load(fullfile(dataset,'manifest','run.mat'),'manifest');run=saved.manifest;
            tc.verifyFalse(isfield(run,'plant_mismatch'));
            report=validate_exact_state_dataset(dataset,true);
            tc.verifyFalse(report.plant_mismatch.declared);
            tc.verifyEqual(m.start_soc_estimated,m.start_soc_true,'Nominal estimate starts at the truth.');
            % The plant path with identity parameters is bitwise the nominal one.
            same=tc.shift('identity',struct('plant_resistance_scale',1,'plant_capacity_Ah',2));
            s=same.missions;
            tc.verifyFalse(same.plant.nominal,'An explicit capacity is a declared (identity) plant.');
            for name={'charge_As','energy_Wh','time_s','distance_m','end_soc_true','end_soc_estimated','score'}
                tc.verifyEqual(s.(name{1}),m.(name{1}),name{1});
            end
            tc.verifyTrue(s.dataset_validated);
        end

        function higherResistanceDrawsMoreCharge(tc)
            r=tc.shift('r1',struct('plant_resistance_scale',1.3));
            m=r.missions(1);n=tc.Nominal.missions(1);
            tc.verifyEqual(m.plant_resistance_scale,1.3);tc.verifyFalse(m.plant_nominal);
            tc.verifyGreaterThan(m.charge_As,n.charge_As,'R1: more charge for the same mission.');
            tc.verifyEqual(m.start_soc_estimated,m.start_soc_true,'R1 leaves the initial estimate nominal.');
            tc.verifyTrue(m.dataset_validated);tc.verifyEqual(r.failed_qps,0);
            tc.verifyPlantRecorded('r1',1.3,2,0);
        end

        function smallerCapacityDrainsTrueSocFaster(tc)
            r=tc.shift('r2',struct('plant_capacity_Ah',1.8));
            m=r.missions(1);n=tc.Nominal.missions(1);
            drop=m.start_soc_true-m.end_soc_true;nominalDrop=n.start_soc_true-n.end_soc_true;
            tc.verifyEqual(drop,m.charge_As/(3600*1.8),'AbsTol',1e-12,'True SOC is booked on the plant capacity.');
            tc.verifyEqual(nominalDrop,n.charge_As/(3600*2),'AbsTol',1e-12);
            tc.verifyGreaterThan(drop/m.charge_As,nominalDrop/n.charge_As,'R2: larger true-SOC drop per Coulomb.');
            tc.verifyGreaterThan(drop,nominalDrop);
            tc.verifyTrue(m.dataset_validated);tc.verifyEqual(r.failed_qps,0);
            tc.verifyPlantRecorded('r2',1,1.8,0);
            % The estimator, observation and reward scales stay nominal.
            saved=load(fullfile(tc.Folder,'r2','mission_01','manifest','configuration.mat'),'cfg');cfg=saved.cfg;
            tc.verifyEqual(cfg.BATTERY.C_nom_Ah,2);
            scales=energy_mission_scales(cfg);tc.verifyEqual(scales.pack_capacity_As,7200);
        end

        function estimateOffsetAppliesAtTheFirstMissionOnly(tc)
            % From 0.90 true / 0.95 estimated, the EKF pulls the estimate towards the
            % truth (about 0.905 after mission 1, 0.902 after mission 2), so a
            % 0.903 minimum start yields exactly two missions.
            o=struct('plant_soc_estimate_offset',.05,'start_soc',.90,'min_start_soc',.903);
            r=tc.shift('r3',o);m=r.missions;
            tc.verifyEqual(numel(m),2,'Two missions: the second carries the estimator.');
            tc.verifyEqual(m(1).start_soc_estimated,m(1).start_soc_true+.05,'AbsTol',1e-15, ...
                'R3: estimated SOC = true + offset at the first mission start.');
            tc.verifyEqual(m(1).start_soc_true,.90);
            tc.verifyEqual(m(2).start_soc_estimated,m(1).end_soc_estimated,'Later missions carry the estimator (no new offset).');
            tc.verifyEqual(m(2).start_soc_true,m(1).end_soc_true);
            tc.verifyTrue(all([m.dataset_validated]));
            tc.verifyPlantRecorded('r3',1,2,.05);
        end

        function validatorRequiresAConsistentDeclaration(tc)
            % A manifest that declares a mismatch the configuration and the
            % captured electrical states do not carry fails validation (and
            % vice versa): checks are extended, never skipped.
            copy=fullfile(tc.Folder,'tampered');
            copyfile(fullfile(tc.Folder,'nominal','mission_01'),copy);
            path=fullfile(copy,'manifest','run.mat');
            saved=load(path,'manifest');manifest=saved.manifest;
            manifest.plant_mismatch=struct('version','plant_battery_mismatch_v1','resistance_scale',1.3, ...
                'capacity_Ah',2,'soc_estimate_offset',0,'scope','tampered'); %#ok<STRNU>
            save(path,'manifest','-v7');
            index=fullfile(copy,'checksums','files.csv');
            files=read_index_csv(index,{'file','bytes','sha256'},{'string','double','string'});
            k=find(replace(files.file,'\','/')=="manifest/run.mat");
            tc.assertNumElements(k,1);
            info=dir(path);files.bytes(k)=info.bytes;files.sha256(k)=string(sha256_file(path));
            writetable(files,index);
            tc.verifyError(@()validate_exact_state_dataset(copy,true),'validate_exact_state_dataset:PlantMismatch');
            bad=tc.Options;bad.plant_capacity_Ah=-1;
            tc.verifyError(@()run_energy_mission_shift_evaluation(fullfile(tc.Folder,'bad'),bad),'MATLAB:expectedPositive');
        end
    end
    methods
        function r=shift(tc,name,extra)
            o=tc.Options;
            for n=fieldnames(extra).',o.(n{1})=extra.(n{1});end
            root=fullfile(tc.Folder,name);
            evalc('r=run_energy_mission_shift_evaluation(root,o);');
        end

        function verifyPlantRecorded(tc,name,scale,capacity,offset)
            root=fullfile(tc.Folder,name);
            manifest=jsondecode(fileread(fullfile(root,'manifest.json')));
            tc.verifyFalse(manifest.plant.nominal);
            tc.verifyEqual([manifest.plant.resistance_scale,manifest.plant.capacity_Ah,manifest.plant.soc_estimate_offset], ...
                [scale capacity offset],'AbsTol',1e-15);
            csv=readtable(fullfile(root,'missions.csv'));
            tc.verifyTrue(all(csv.plant_nominal==0));
            tc.verifyEqual(csv.plant_resistance_scale(1),scale);tc.verifyEqual(csv.plant_capacity_Ah(1),capacity);
            dataset=fullfile(root,'mission_01');
            saved=load(fullfile(dataset,'manifest','configuration.mat'),'cfg');cfg=saved.cfg;
            declaration=cfg.BATTERY.plant_mismatch;
            tc.verifyEqual([declaration.resistance_scale,declaration.capacity_Ah,declaration.soc_estimate_offset], ...
                [scale capacity offset]);
            saved=load(fullfile(dataset,'manifest','run.mat'),'manifest');run=saved.manifest;
            tc.verifyEqual(run.plant_mismatch,declaration,'Dataset manifest records the mismatch.');
            report=validate_exact_state_dataset(dataset,true);
            tc.verifyTrue(report.plant_mismatch.declared);
            tc.verifyEqual(report.plant_mismatch.declaration,declaration);
        end
    end
end
