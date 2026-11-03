function s = start(b)
assert(~b.use_pack_sizing,'robotenergy:FixedPack','Size the pack before the experiment.');
if isfield(b,'fixed_pack_spec')
    frozen=b.fixed_pack_spec;
    assert(b.n_series==frozen.n_series && b.n_parallel==frozen.n_parallel && ...
        b.C_nom_Ah==frozen.cell_capacity_Ah,'robotenergy:PackChanged', ...
        'Frozen pack size/capacity changed; use the recorded specification across all episodes.');
end
for k = {'n_series','n_parallel'}
    validateattributes(b.(k{1}),{'numeric'},{'scalar','integer','positive','finite'});
end
validateattributes(b.C_nom_Ah,{'numeric'},{'scalar','positive','finite'});
[t,n] = phase3battery.resolveModelOptions(b.model_options,'our params.xlsx');
validateattributes(b.SOC_init,{'numeric'}, ...
    {'scalar','real','finite','>=',t.SOC(1),'<=',t.SOC(end)});
s = struct('ns',b.n_series,'np',b.n_parallel,'capacity',b.C_nom_Ah, ...
    'soc',b.SOC_init,'rc',zeros(3,1),'chargeAs',0,'energyWh',0, ...
    'estimate',[b.SOC_init;zeros(3,1)],'cov',n.initial_covariance, ...
    'Q',n.process_covariance,'Rv',n.measurement_variance,'elapsed',0);
s.grid=t.SOC; s.values=t{: ,{'R0','R1','R2','R3','C1','C2','C3'}};
s.ocv=pchip(t.SOC,t.OCV);
s.slope=gradient(t.OCV)./gradient(t.SOC);
[breaks,coefs,~,order]=unmkpp(s.ocv);
s.ocvDerivative=mkpp(breaks,coefs(:,1:end-1).*(order-1:-1:1));
s.estimatorVersion=robotenergy.estimatorVersion(b);
s.voltageLimitSteps=0; s.torqueLimitSteps=0;
if isfield(b,'fixed_pack_spec'), s.packId=b.fixed_pack_spec.id; end
end
