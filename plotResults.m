function plotResults(EKF_data,UKF_data,GSF_EKF_data,GSF_UKF_data,IMM_EKF_data,IMM_UKF_data,particleFilter_data)

if ~isempty(EKF_data)
    EKF_data_set = load(EKF_data);
    
    plotCovs = 1;
    
    plotFilterStates([EKF_data_set.state_out],[EKF_data_set.timeVec],[EKF_data_set.truthDataNav],'EKF',plotCovs,...
        [EKF_data_set.S_mat],[EKF_data_set.nu_mat],[EKF_data_set.obsTime])
    
    EKF_time = EKF_data_set.elapsedTime;
end

if ~isempty(UKF_data)
    UKF_data_set = load(UKF_data);
    
    plotCovs = 1;
    
    plotFilterStates([UKF_data_set.state_out],[UKF_data_set.timeVec],[UKF_data_set.truthDataNav],'UKF',plotCovs,...
        [UKF_data_set.S_mat],[UKF_data_set.nu_mat],[UKF_data_set.obsTime])
    
    UKF_time = UKF_data_set.elapsedTime;
end

if ~isempty(GSF_EKF_data)
    GSF_EKF_data_set = load(GSF_EKF_data);
    
    plotCovs = 1;
    
    plotFilterStates([GSF_EKF_data_set.state_out],[GSF_EKF_data_set.timeVec],[GSF_EKF_data_set.truthDataNav],'GSF - EKF',plotCovs,...
        [GSF_EKF_data_set.S_mat],[GSF_EKF_data_set.nu_mat],[GSF_EKF_data_set.obsTime])
    
    GSF_EKF_time = GSF_EKF_data_set.elapsedTime;
end

if ~isempty(GSF_UKF_data)
    GSF_UKF_data_set = load(GSF_UKF_data);
    
    plotCovs = 1;
    
    plotFilterStates([GSF_UKF_data_set.state_out],[GSF_UKF_data_set.timeVec],[GSF_UKF_data_set.truthDataNav],'GSF - UKF',plotCovs,...
        [GSF_UKF_data_set.S_mat],[GSF_UKF_data_set.nu_mat],[GSF_UKF_data_set.obsTime])
    
    GSF_UKF_time = GSF_UKF_data_set.elapsedTime;
end

if ~isempty(IMM_EKF_data)
    IMM_EKF_data_set = load(IMM_EKF_data);
    
    plotCovs = 1;
    
    plotFilterStates([IMM_EKF_data_set.state_out],[IMM_EKF_data_set.timeVec],[IMM_EKF_data_set.truthDataNav],'IMM - EKF',plotCovs,...
        [IMM_EKF_data_set.S_mat],[IMM_EKF_data_set.nu_mat],[IMM_EKF_data_set.obsTime])
    
    IMM_EKF_time = IMM_EKF_data_set.elapsedTime;
end

if ~isempty(IMM_UKF_data)
    IMM_UKF_data_set = load(IMM_UKF_data);
    
    plotCovs = 1;
    
    plotFilterStates([IMM_UKF_data_set.state_out],[IMM_UKF_data_set.timeVec],[IMM_UKF_data_set.truthDataNav],'IMM - UKF',plotCovs,...
        [IMM_UKF_data_set.S_mat],[IMM_UKF_data_set.nu_mat],[IMM_UKF_data_set.obsTime])
    
    IMM_UKF_time = IMM_UKF_data_set.elapsedTime;
end

if ~isempty(particleFilter_data)
    
    PF_data_set = load(particleFilter_data);
    
    plotCovs = 0;
    
    plotFilterStates([PF_data_set.state_out],[PF_data_set.timeVec],[PF_data_set.truthDataNav],'Particle Filter',plotCovs)
    
    PF_time = PF_data_set.elapsedTime;
end