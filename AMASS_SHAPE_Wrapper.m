%AMASS_SHAPE_Wrapper

clear
clc
close all
%% Setup python path and call AMASS body
%C:\Users\emmay\anaconda3

pyExec = 'C:\Users\emmay\anaconda3\';
pyRoot = fileparts(pyExec);
p = getenv('PATH');
p = strsplit(p, ';');
addToPath = {
   pyRoot
   fullfile(pyRoot, 'Library', 'mingw-w64', 'bin')
   fullfile(pyRoot, 'Library', 'usr', 'bin')
   fullfile(pyRoot, 'Library', 'bin')
   fullfile(pyRoot, 'Scripts')
   fullfile(pyRoot, 'bin')
};
p = [addToPath(:); p(:)];
p = unique(p, 'stable');
p = strjoin(p, ';');
setenv('PATH', p);

%% Write .mat betas and pose_body Initial Variables
pose_body = zeros(1,63);
betas = zeros(1,10);
% betas = ones(1,10); betas = betas*3;

%root_orient and trans aren't manipulated for this but AMASSBody.py is
%designed to take them in if needed.
root_orient = [0,0,0]; %start out at 0, then manipulate
trans = [0,0,0]; %start out at 0, then manipulate

save('pose_body','pose_body')
save('betas','betas')
save('root_orient', 'root_orient')
save('trans', 'trans')

%% Begin
[status,result] = system('python C:\Users\emmay\Desktop\AMASS\amass-master\notebooks\AMASSBody.py');

%AMASS Joints
AJ = load('AMASSSkeleton.txt');
KinectJoints = jsondecode(fileread('joints (2).json')); %Kinect data

%Kinect Joints
KJ = KinectJoints.frames(1).bodies.joint_positions;
Scale = abs((KJ(19,:) - KJ(20,:))/(AJ(2,:) - AJ(5,:)));
screenOutput = eye(2,2);

% [KJ] = OrientBody(AJ,KJ,Scale,screenOutput);

%% Get bodies into same coordinate system
trans = [0 0 0];
save('trans', 'trans')
[status,result] = system('python C:\Users\emmay\Desktop\AMASS\amass-master\notebooks\AMASSBody.py');
AJ = load('AMASSSkeleton.txt');

trans = [0 -(AJ(3,2)+AJ(2,2))/2 0-(AJ(3,3)+AJ(2,3))/2]; %!!! needs something to decide the sign of trans
save('trans', 'trans')
[status,result] = system('python C:\Users\emmay\Desktop\AMASS\amass-master\notebooks\AMASSBody.py');
AJ = load('AMASSSkeleton.txt');

%create coordinate system on kinect skeleton using origin as center pelvis
%joint
Y = (KJ(4,1:3)-KJ(3,1:3))/norm(KJ(4,1:3)-KJ(3,1:3));
temp = (KJ(19,1:3)-KJ(1,1:3))/norm(KJ(19,1:3)-KJ(1,1:3));
Z = cross(temp,Y);
X = cross(Z,Y);
T = eye(4);
T(1:3,1) = X';
T(1:3,2) = Y';
T(1:3,3) = Z';

ave = [(KJ(19,1)+KJ(23,1))/2 (KJ(19,2)+KJ(23,2))/2 (KJ(19,3)+KJ(23,3))/2];
% T(1:3,4) = KJ(1,1:3)'; %zero point is the pelvis KJ(1)
T(1:3,4) = ave';
KJ(:,4) = 1;%zero point is average between the hips
KJT = (inv(T)*KJ')';
KJ = KJT/1000;

lower = 1; %Scale factor low bound
AJ = AJ; %scale up to m
% comparePc(AJ,KJ)

% stepinc = 1;
% disp("Scaling Kinect body to match Amass Torso length")
% [Scale] = ScaleJoints(lower,stepinc, KJT, AJ, AR, KR,screenOutput) %may need the part that decides direction tweaked
% 
% KJTS = KJT*Scale; %kinect joints transformed and scaled
% if screenOutput(1,1) == 1
%     sound(sin(1:3000)); 
%     comparePc(AJ,KJTS)
%     x = input('Does the torso have the right scale?','s');
% end

%% Do steepest descent
step = 50;
eps = 10^-10; %made very small. Changes in beta are so small.
tic
[betas, history_b, history_f, AJMov, AV] = SteepestDescent(betas,step,eps,AJ,KJ)
toc 

save('betas_50','betas')
save('history_b_50','history_b')
save('history_f_50','history_f')
save('AJMov_50','AJMov')
save('AV_50','AV')
%graph the objective function history and beta history
%%
close all

figure(1)
L = length(history_f)
plot(1:L,history_f)
xlabel('Number of Iterations')
ylabel('f(\beta)')
title('Objective Function Value')
savefig('Objective Function Value')

figure(2)
subplot(5,2,1)
plot(1:L,history_b(1,:))
title('Changes in B1')
xlabel('Number of Iterations')
ylabel('B1')

subplot(5,2,2)
plot(1:L,history_b(2,:))
title('Changes in B2')
xlabel('Number of Iterations')
ylabel('B2')

subplot(5,2,3)
plot(1:L,history_b(3,:))
title('Changes in B3')
xlabel('Number of Iterations')
ylabel('B3')

subplot(5,2,4)
plot(1:L,history_b(4,:))
title('Changes in B4')
xlabel('Number of Iterations')
ylabel('B4')

subplot(5,2,5)
plot(1:L,history_b(5,:))
title('Changes in B5')
xlabel('Number of Iterations')
ylabel('B5')

subplot(5,2,6)
plot(1:L,history_b(6,:))
title('Changes in B6')
xlabel('Number of Iterations')
ylabel('B6')

subplot(5,2,7)
plot(1:L,history_b(7,:))
title('Changes in B7')
xlabel('Number of Iterations')
ylabel('B7')

subplot(5,2,8)
plot(1:L,history_b(8,:))
title('Changes in B8')
xlabel('Number of Iterations')
ylabel('B8')

subplot(5,2,9)
plot(1:L,history_b(9,:))
title('Changes in B9')
xlabel('Number of Iterations')
ylabel('B9')

subplot(5,2,10)
plot(1:L,history_b(10,:))
title('Changes in B10')
xlabel('Number of Iterations')
ylabel('B10')
savefig('Betas_with_each_iteration')
%% make movie
% close all
% v = VideoWriter('OptProj_origfcn_1','MPEG-4');
% v.Quality = 100;
% v.FrameRate = 10;
% open(v)
% AF = load('AmassFaces.txt');
% for i=1:length(AV)
%     figure('Units','normalized','Position',[0 0 1 1])
%     %front view
% %     subplot(1,2,1);
%     p1 = patch('Faces', 1+AF, 'Vertices', AV(i).I(:,:));
%     set(p1,'FaceColor',[.89 .855 .788], 'FaceLighting','gouraud','EdgeColor','none','FaceAlpha',.5,'SpecularStrength',.5);
%     light('Position',[1 1 1]*1000,'Style','infinite');
%     light('Position',[-1 -1 1]*1000,'Style','infinite');
%     light('Position',[-.5 -.5 -11]*1000,'Style','infinite');
%     hold on 
%     scatter3(AJMov(i).I(:,1),AJMov(i).I(:,2),AJMov(i).I(:,3),'MarkerEdgeColor','k','MarkerFaceColor','k')
% %     xlabel('X')
% %     ylabel('Y')
% %     zlabel('Z')
% %     scatter3(KJ(:,1),KJ(:,2),KJ(:,3),'MarkerEdgeColor','b','MarkerFaceColor','b')
%     axis equal
%     axis off
%     
% %     subplot(1,2,2);
% %     p1 = patch('Faces', 1+AF, 'Vertices', AV(i).I(:,:));
% %     set(p1,'FaceColor',[.89 .855 .788], 'FaceLighting','gouraud','EdgeColor','none','FaceAlpha',.5,'SpecularStrength',.5);
% %     light('Position',[1 1 1]*1000,'Style','infinite');
% %     light('Position',[-1 -1 1]*1000,'Style','infinite');
% %     light('Position',[-.5 -.5 -11]*1000,'Style','infinite');
% %     hold on 
% %     scatter3(AJMov(i).I(:,1),AJMov(i).I(:,2),AJMov(i).I(:,3),'MarkerEdgeColor','k','MarkerFaceColor','k')
% % %     xlabel('X')
% % %     ylabel('Y')
% % %     zlabel('Z')
% % %     scatter3(KJ(:,1),KJ(:,2),KJ(:,3),'MarkerEdgeColor','b','MarkerFaceColor','b')
% %     axis equal
% %     axis off
% %     view(90,0)
%     
% %     subplot(1,3,3);
% %     p1 = patch('Faces', 1+AF, 'Vertices', AV(i).I(:,:));
% %     set(p1,'FaceColor',[.89 .855 .788], 'FaceLighting','gouraud','EdgeColor','none','FaceAlpha',.5,'SpecularStrength',.5);
% %     light('Position',[1 1 1]*1000,'Style','infinite');
% %     light('Position',[-1 -1 1]*1000,'Style','infinite');
% %     light('Position',[-.5 -.5 -11]*1000,'Style','infinite');
% %     hold on 
% %     scatter3(AJMov(i).I(:,1),AJMov(i).I(:,2),AJMov(i).I(:,3),'MarkerEdgeColor','k','MarkerFaceColor','k')
% %     xlabel('X')
% %     ylabel('Y')
% %     zlabel('Z')
% % %     scatter3(KJ(:,1),KJ(:,2),KJ(:,3),'MarkerEdgeColor','b','MarkerFaceColor','b')
% %     axis equal
% %     axis off
% %     view(0,-180)
% 
%     frame = getframe(gcf);  
%     writeVideo(v,frame)
% 
% end
% close(v)
%%
%[betas, history_b, history_f, AJMov, AV]
