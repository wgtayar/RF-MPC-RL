function files = plot_energy_mission_training_summary(summaryDirs, outputDir)
%plot_energy_mission_training_summary Learning and behaviour figures from exported summaries.
%   files = plot_energy_mission_training_summary(summaryDirs, outputDir) reads one
%   or more folders written by export_energy_mission_training_summary (a char,
%   string array or cellstr; a -t2 summary is placed after its -t1 summary when
%   both are given in that order) and writes PNG figures into outputDir:
%     overview.png       all runs: rolling completion rate, rolling return, end reasons
%     <run>_learning.png return and its components, Q0 against return, end reasons,
%                        speed command, kappa and tau of completed episodes
%     <run>_behaviour.png how speed, end SOC and drive headroom depend on initial
%                        SOC, the last decisions before actuator_limit endings,
%                        and the actor's speed action at probe states per checkpoint
%   Works headless (matlab -batch).
    summaryDirs = cellstr(string(summaryDirs));
    if ~isfolder(outputDir), mkdir(outputDir); end
    runs = localGroup(summaryDirs);
    files = {};
    reasons = ["mission_complete","actuator_limit","time_limit","step_limit"];
    colors = [0.13 0.55 0.13; 0.80 0.15 0.15; 0.95 0.60 0.05; 0.45 0.45 0.45];
    overview = localFigure([1500 900]);
    tl = tiledlayout(overview,3,1,'TileSpacing','compact');
    title(tl,'All runs');
    axes1 = nexttile(tl); hold(axes1,'on'); ylabel(axes1,'completion rate (10-ep window)');
    axes2 = nexttile(tl); hold(axes2,'on'); ylabel(axes2,'return (10-ep mean)');
    axes3 = nexttile(tl); hold(axes3,'on'); ylabel(axes3,'run'); xlabel(axes3,'episode');
    for r = 1:numel(runs)
        E = runs(r).episodes; D = runs(r).decisions; name = runs(r).name;
        x = E.global_episode;
        plot(axes1,x,movmean(double(E.completed),[9 0]),'DisplayName',name,'LineWidth',1.2);
        plot(axes2,x,movmean(E.episode_return,[9 0]),'DisplayName',name,'LineWidth',1.2);
        for k = 1:numel(reasons)
            m = E.end_reason == reasons(k);
            scatter(axes3,x(m),r*ones(nnz(m),1),18,colors(k,:),'filled','HandleVisibility','off');
        end
        files{end+1} = localLearning(runs(r),outputDir,reasons,colors); %#ok<AGROW>
        files{end+1} = localBehaviour(runs(r),outputDir); %#ok<AGROW>
        clear D
    end
    legend(axes1,'Location','eastoutside','Interpreter','none'); legend(axes2,'Location','eastoutside','Interpreter','none');
    yticks(axes3,1:numel(runs)); yticklabels(axes3,{runs.name}); set(axes3,'TickLabelInterpreter','none');
    ylim(axes3,[0.5 numel(runs)+0.5]);
    for k = 1:numel(reasons), scatter(axes3,NaN,NaN,18,colors(k,:),'filled','DisplayName',reasons(k)); end
    legend(axes3,'Location','eastoutside','Interpreter','none');
    ylim(axes1,[0 1]);
    files{end+1} = fullfile(outputDir,'overview.png');
    exportgraphics(overview,files{end},'Resolution',110); close(overview);
end

function runs = localGroup(dirs)
    runs = struct('name',{},'episodes',{},'decisions',{},'probe',{});
    for k = 1:numel(dirs)
        summary = jsondecode(fileread(fullfile(dirs{k},'summary.json')));
        base = regexprep(summary.run_id,'-t\d+$','');
        E = readtable(fullfile(dirs{k},'episodes.csv'),'TextType','string');
        D = readtable(fullfile(dirs{k},'decisions.csv'),'TextType','string');
        P = table();
        if isfile(fullfile(dirs{k},'policy_probe.csv')), P = readtable(fullfile(dirs{k},'policy_probe.csv')); end
        i = find(strcmp({runs.name},base),1);
        if isempty(i)
            runs(end+1) = struct('name',base,'episodes',E,'decisions',D,'probe',P); %#ok<AGROW>
            i = numel(runs); offset = 0;
        else
            offset = max(runs(i).episodes.global_episode);
            E.global_episode = E.episode+offset; D.global_episode = D.episode+offset;
            if ~isempty(P), P.checkpoint = P.checkpoint+offset; P.probe_episode = P.probe_episode+offset; end
            runs(i).episodes = [runs(i).episodes; E]; runs(i).decisions = [runs(i).decisions; D];
            runs(i).probe = [runs(i).probe; P];
            continue
        end
        runs(i).episodes.global_episode = E.episode+offset;
        runs(i).decisions.global_episode = D.episode+offset;
    end
end

function file = localLearning(run,outputDir,reasons,colors)
    E = run.episodes; x = E.global_episode;
    f = localFigure([1500 1000]);
    tl = tiledlayout(f,3,2,'TileSpacing','compact'); title(tl,run.name,'Interpreter','none');
    ax = nexttile(tl); hold(ax,'on');
    plot(ax,x,E.episode_return,'.-','DisplayName','return');
    plot(ax,x,E.return_progress,'DisplayName','progress'); plot(ax,x,E.return_time,'DisplayName','time');
    plot(ax,x,E.return_charge,'DisplayName','charge'); plot(ax,x,E.return_terminal,'DisplayName','terminal');
    legend(ax,'Location','best'); ylabel(ax,'episode sum'); title(ax,'Return and its components');
    ax = nexttile(tl); hold(ax,'on');
    plot(ax,x,E.episode_q0,'.-','DisplayName','Q0 (critic at start)');
    plot(ax,x,E.episode_return,'.-','DisplayName','actual return');
    legend(ax,'Location','best'); title(ax,'Critic estimate vs outcome');
    ax = nexttile(tl); hold(ax,'on');
    for k = 1:numel(reasons)
        m = E.end_reason == reasons(k);
        scatter(ax,x(m),E.sim_time_s(m),25,colors(k,:),'filled','DisplayName',reasons(k));
    end
    legend(ax,'Location','best','Interpreter','none'); ylabel(ax,'simulated time (s)'); title(ax,'End reason');
    ax = nexttile(tl); hold(ax,'on');
    plot(ax,x,E.mean_v_exec,'.-','DisplayName','mean'); plot(ax,x,E.max_v_exec,'.-','DisplayName','max');
    plot(ax,x,E.last_v_exec,'.','DisplayName','last decision');
    legend(ax,'Location','best'); ylabel(ax,'executed speed (m/s)'); title(ax,'Speed command');
    ax = nexttile(tl); hold(ax,'on'); m = logical(E.completed);
    plot(ax,x(m),E.kappa_final(m),'o','DisplayName','\kappa'); plot(ax,x(m),E.tau_final(m),'s','DisplayName','\tau');
    legend(ax,'Location','best'); xlabel(ax,'episode'); title(ax,'Completed missions: \kappa, \tau');
    ax = nexttile(tl);
    scatter(ax,E.tau_final(m),E.kappa_final(m),30,x(m),'filled'); colorbar(ax);
    xlabel(ax,'\tau (time)'); ylabel(ax,'\kappa (charge)'); title(ax,'Trade-off, colour = episode');
    file = fullfile(outputDir,[run.name '_learning.png']);
    exportgraphics(f,file,'Resolution',110); close(f);
end

function file = localBehaviour(run,outputDir)
    E = run.episodes; D = run.decisions; P = run.probe;
    f = localFigure([1500 1000]);
    tl = tiledlayout(f,2,3,'TileSpacing','compact'); title(tl,[run.name ' behaviour'],'Interpreter','none');
    half = E.global_episode > max(E.global_episode)/2;
    ax = nexttile(tl); hold(ax,'on');
    scatter(ax,E.initial_soc_pct(~half),E.mean_v_exec(~half),25,'filled','DisplayName','first half');
    scatter(ax,E.initial_soc_pct(half),E.mean_v_exec(half),25,'filled','DisplayName','second half');
    legend(ax,'Location','best'); xlabel(ax,'initial SOC (%)'); ylabel(ax,'mean speed (m/s)'); title(ax,'Speed vs initial SOC');
    ax = nexttile(tl);
    localGroupScatter(ax,E.initial_soc_pct,E.end_soc_pct,E.end_reason);
    xlabel(ax,'initial SOC (%)'); ylabel(ax,'end SOC (%)'); title(ax,'End SOC'); legend(ax,'Interpreter','none','Location','best');
    ax = nexttile(tl);
    localGroupScatter(ax,E.initial_soc_pct,E.min_drive_headroom_V,E.end_reason);
    xlabel(ax,'initial SOC (%)'); ylabel(ax,'min drive headroom (V)'); title(ax,'Voltage headroom'); legend(ax,'Interpreter','none','Location','best');
    ax = nexttile(tl); hold(ax,'on');
    limited = E.global_episode(E.end_reason == "actuator_limit");
    for e = limited(max(1,end-9):end)'
        d = D(D.global_episode == e,:); d = d(max(1,end-5):end,:);
        plot(ax,(-(height(d)-1):0)',d.v_exec,'.-');
    end
    xlabel(ax,'decisions before end'); ylabel(ax,'executed speed (m/s)'); title(ax,'Last 10 actuator\_limit endings');
    ax = nexttile(tl); hold(ax,'on');
    for e = limited(max(1,end-9):end)'
        d = D(D.global_episode == e,:); d = d(max(1,end-5):end,:);
        plot(ax,(-(height(d)-1):0)',d.drive_headroom_V,'.-');
    end
    xlabel(ax,'decisions before end'); ylabel(ax,'drive headroom (V)'); title(ax,'Headroom before actuator\_limit');
    ax = nexttile(tl);
    if ~isempty(P)
        soc = E.initial_soc_pct(match(P.probe_episode,E.global_episode));
        scatter(ax,P.checkpoint,P.a4,12,soc,'filled'); c = colorbar(ax); c.Label.String = 'probe initial SOC (%)';
        xlabel(ax,'checkpoint (episode)'); ylabel(ax,'actor speed action a4 (normalized)');
        title(ax,'Policy at fixed start states');
    end
    file = fullfile(outputDir,[run.name '_behaviour.png']);
    exportgraphics(f,file,'Resolution',110); close(f);
end

function localGroupScatter(ax,x,y,groups)
    hold(ax,'on');
    labels = unique(groups,'stable');
    for k = 1:numel(labels)
        m = groups == labels(k);
        scatter(ax,x(m),y(m),25,'filled','DisplayName',labels(k));
    end
end

function f = localFigure(size)
    f = figure('Visible','off','Position',[0 0 size],'Color','w');
    if isprop(f,'Theme'), f.Theme = 'light'; end
end

function index = match(values,keys)
    [~,index] = ismember(values,keys);
end
