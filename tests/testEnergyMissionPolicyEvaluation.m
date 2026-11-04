classdef testEnergyMissionPolicyEvaluation < matlab.unittest.TestCase
 % Retained clean qualification checkpoint is local-only, never modified.
 properties
  Baseline
  Folder
 end
 methods(TestClassSetup)
  function locateCapturedCheckpoint(tc)
   source=bootstrap_RF_MPC_RL();tc.Baseline=fullfile(fileparts(source),'BeyondTorque-Local-Artifacts','energy-qualification-20261104-a','learning_smoke');
   tc.assumeTrue(isfile(fullfile(tc.Baseline,'training','final_agent.mat')),'Requires retained clean qualification checkpoint.');
   fixture=tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture);tc.Folder=fixture.Folder;
  end
 end
 methods(Test)
  function capturedActorIsDeterministicAndMapsOnce(tc)
   c=load_energy_mission_checkpoint(tc.Baseline);tc.verifyEqual(c.agent_specification.observation_dimension,87);
   tc.verifyEqual(c.checkpoint_episodes_trained,1);tc.verifyEqual(c.stage,'final');
   first=evaluate(c.actor,{zeros(87,1)});second=evaluate(c.actor,{zeros(87,1)});tc.verifyEqual(first,second);
   [physical,a]=energy_mission_action_transform(double(first{1}),c.bundle.cfg);
   tc.verifyEqual(physical,a.bias+a.scale.*double(first{1}),'AbsTol',1e-14);
   tc.verifyTrue(all(physical>=a.physical_lower & physical<=a.physical_upper));
  end
  function initialCheckpointDoesNotClaimTrainedWeights(tc)
   c=load_energy_mission_checkpoint(fullfile(tc.Baseline,'training','initial_agent.mat'));
   tc.verifyEqual(c.checkpoint_episodes_trained,0);tc.verifyEqual(c.source_run_episodes_trained,1);tc.verifyEqual(c.stage,'initial');
  end
  function checksumCorruptionRejectedBeforeLoading(tc)
   root=clone(tc);fid=fopen(fullfile(root,'training','final_agent.mat'),'a');fwrite(fid,uint8(17),'uint8');fclose(fid);
   tc.verifyError(@()load_energy_mission_checkpoint(root),'energyPolicy:Checksum');
  end
  function rehashedSourceDriftRejected(tc)
   root=clone(tc);relative='training/report.json';p=fullfile(root,relative);r=jsondecode(fileread(p));r.source_unchanged=false;
   writeJson(p,r);rehash(root,relative);tc.verifyError(@()load_energy_mission_checkpoint(root),'energyPolicy:SourceProvenance');
  end
  function rehashedPhysicalMappingMismatchRejected(tc)
   root=clone(tc);relative='training/agent_specification.json';p=fullfile(root,relative);s=jsondecode(fileread(p));s.action_physical_scale(1)=2*s.action_physical_scale(1);
   writeJson(p,s);rehash(root,relative);p=fullfile(root,'training','final_agent.mat');saved=load(p);
   saved.agentSpec.action_physical_scale(1)=2*saved.agentSpec.action_physical_scale(1);save(p,'-struct','saved','-v7.3');rehash(root,'training/final_agent.mat');
   tc.verifyError(@()load_energy_mission_checkpoint(root),'energyPolicy:ActionMapping');
  end
  function overridesPreserveArchivedModelAndReference(tc)
   c=load_energy_mission_checkpoint(tc.Baseline);original=c.bundle;
   original.cfg.TEST_SENTINEL=struct('value',17);
   original.cfg.ENERGY_RL.reference.charge_As=2*original.cfg.ENERGY_RL.reference.charge_As;
   changed=energy_mission_evaluation_bundle(original,struct('initial_soc',1,'target_distance_m',.02,'mission_duration_s',.04,'seed',17));
   tc.verifyEqual(changed.cfg.TEST_SENTINEL,original.cfg.TEST_SENTINEL);
   tc.verifyEqual(changed.cfg.ENERGY,original.cfg.ENERGY);tc.verifyEqual(changed.cfg.TRAIN,original.cfg.TRAIN);
   tc.verifyEqual(changed.cfg.ENERGY_RL.reference,original.cfg.ENERGY_RL.reference);
   tc.verifyEqual(changed.cfg.ENERGY_RL.time_weight,original.cfg.ENERGY_RL.time_weight);
   tc.verifyEqual(changed.cfg.ENERGY_RL.reference_charge_As,original.cfg.ENERGY_RL.reference.charge_As/original.cfg.ENERGY_RL.reference.distance_m*.02);
   tc.verifyEqual(changed.cfg.EP_STEPS,4);tc.verifyEqual(changed.cfg.V_REQ_FIXED,.5);tc.verifyEqual(changed.cfg.RNG_SEED,17);
   tc.verifyEqual(changed.cfg.BATTERY.SOC_init,1);tc.verifyEqual(changed.initial_R,original.initial_R);
  end
  function invalidScenarioOverrideRejected(tc)
   c=load_energy_mission_checkpoint(tc.Baseline);
   tc.verifyError(@()energy_mission_evaluation_bundle(c.bundle,struct('time_weight',.8)),'energyPolicy:ScenarioOverrides');
  end
  function interruptedCheckpointCannotMasqueradeAsFinal(tc)
   root=clone(tc);p=fullfile(root,'training','interrupted_agent.mat');copyfile(fullfile(root,'training','final_agent.mat'),p);
   tc.verifyError(@()load_energy_mission_checkpoint(p),'energyPolicy:Checkpoint');
  end
 end
end
function root=clone(tc),root=tempname(tc.Folder);copyfile(tc.Baseline,root);end
function writeJson(p,v),f=fopen(p,'w');c=onCleanup(@()fclose(f));fprintf(f,'%s\n',jsonencode(v,'PrettyPrint',true));end %#ok<NASGU>
function rehash(root,relative)
p=fullfile(root,relative);ip=fullfile(root,'training','artifact_sha256.csv');t=readtable(ip,'TextType','string');r=replace(t.file,'\','/')==string(relative);
assert(nnz(r)==1);d=dir(p);t.bytes(r)=d.bytes;t.sha256(r)=string(sha256_file(p));writetable(t,ip);
end
