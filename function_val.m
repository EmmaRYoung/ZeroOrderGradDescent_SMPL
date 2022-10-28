function [f] = function_val(AJ, KJ)

%Legs
ATR = norm(AJ(2,:) - AJ(5,:)); %Amass thigh right
ATL = norm(AJ(3,:) - AJ(6,:));%Amass thigh left
% 
ACR = norm(AJ(5,:) - AJ(8,:)); %Amass calf right
ACL = norm(AJ(6,:) - AJ(9,:));%Amass calf left
% 
AFR = norm(AJ(8,:) - AJ(11,:)); %Amass foot right
AFL = norm(AJ(9,:) - AJ(12,:)); %Amass foot left
% 
% 
KTR = norm(KJ(23,:) - KJ(24,:)); %Kinect thigh right
KTL = norm(KJ(19,:) - KJ(20,:));%Kinect thigh left

KCR = norm(KJ(24,:) - KJ(25,:)); %Kinect calf right
KCL = norm(KJ(20,:) - KJ(21,:));%Kinect calf left

KFR = norm(KJ(25,:) - KJ(26,:));%Kinect foot right
KFL = norm(KJ(21,:) - KJ(22,:));%Kinect foot left

%Arms
ABR = norm(AJ(17,:) - AJ(19,:)); %amass bicept right
AF_R = norm(AJ(19,:) - AJ(21,:)); %amass forarm right

ABL = norm(AJ(18,:) - AJ(20,:)); %amass bicept left
AF_L = norm(AJ(20,:) - AJ(22,:)); %amass forarm left
%
%
KBR = norm(KJ(13,:) - KJ(14,:));%kinect bicept right
KF_R = norm(KJ(14,:) - KJ(15,:));%kineft forarm right

KBL = norm(KJ(6,:) - KJ(7,:));%kinect bicept left
KF_L = norm(KJ(7,:) - KJ(8,:));%kinect forarm left

%trunk
%Amass
%define midpoint between hips
mid = (AJ(3,:)+AJ(2,:))/2;
AT = norm(mid(1,:) - AJ(13,:));

%kinect
%from pelvis to neck
KT = norm(KJ(1,:) - KJ(4,:));

%Create vector to pass to objective function
%Keep same order
ALength = [ATR ATL ACR ACL AFR AFL ABR AF_R ABL AF_L AT];
KLength = [KTR KTL KCR KCL KFR KFL KBR KF_R KBL KF_L KT];

for i=1:length(ALength)
    value(i) = (ALength(i) - KLength(i))^2;
end
f = sum(value);
end

