function plotFilterStates(filterStates,timeVec,truthDataNav,titleStr,plotCovs,S_mat,nu_mat,ObsTime)

figure
subplot(3,1,1)
plot(timeVec,filterStates(1,:))
title(titleStr)
hold on
plot(timeVec,truthDataNav(:,2),'r')
legend('Vn est','Vn truth')
hold on
subplot(3,1,2)
plot(timeVec,filterStates(2,:))
hold on
plot(timeVec,truthDataNav(:,3),'r')
legend('Ve est','Ve truth')
subplot(3,1,3)
plot(timeVec,rad2deg(wrapToPi(filterStates(3,:))))
hold on
plot(timeVec,rad2deg(wrapToPi(truthDataNav(:,4))),'r')
hold on
legend('yaw est','yaw truth')
ylim([-180, 180])

if plotCovs
    if size(S_mat,3) == 1
        figure
        subplot(2,1,1)
        hold on
        plot(ObsTime,2*sqrt(S_mat(1,:)),'--r')
        hold on
        plot(ObsTime,nu_mat(1,:),'b')
        hold on
        plot(ObsTime,-2*sqrt(S_mat(1,:)),'--r')
        
        subplot(2,1,2)
        hold on
        plot(ObsTime,2*sqrt(S_mat(2,:)),'--r')
        hold on
        plot(ObsTime,nu_mat(2,:),'b')
        hold on
        plot(ObsTime,-2*sqrt(S_mat(2,:)),'--r')
    else
        figure
        N = size(S_mat,3);
        for i = 1:N %Vn
            hold on
            subplot(ceil(N/4),4,i)
            hold on
            plot(ObsTime,2*sqrt(S_mat(1,:,i)),'--r')
            hold on
            plot(ObsTime,nu_mat(1,:,i),'b')
            hold on
            plot(ObsTime,-2*sqrt(S_mat(1,:,i)),'--r')
            title(['filter ',int2str(i)])
        end
        figure
        for i = 1:N %Ve
            hold on
            subplot(ceil(N/4),4,i)
            hold on
            plot(ObsTime,2*sqrt(S_mat(2,:,i)),'--r')
            hold on
            plot(ObsTime,nu_mat(2,:,i),'b')
            hold on
            plot(ObsTime,-2*sqrt(S_mat(2,:,i)),'--r')
            title(['filter ',int2str(i)])
        end
    end
    
end

