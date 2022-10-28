function [g] = gradient_test(betas,KJ)
%betas should be exported as a [1x10] row vector, but math is done with it as a [10x1] vector. 

%define z in the unit sphere
save('betas','betas')
[status,result] = system('python C:\Users\emmay\Desktop\AMASS\amass-master\notebooks\AMASSBody.py');
AJ_o = load('AMASSSkeleton.txt');
L = 5;

g = zeros(10,1);

for i=1:L
    

a = randn(10,1); %include negative random numbers ?
z = a/norm(a);

del = 0.0001; %.1? larger than 0.0001

betas = reshape(betas,10,1);
betas_o = betas;
temp = zeros(10,1);

% for i=1:length(z)
%     z_vect = temp;
%     z_vect(i,:) = z(i);
betas = betas_o + del*z;

save('betas','betas')
[status,result] = system('python C:\Users\emmay\Desktop\AMASS\amass-master\notebooks\AMASSBody.py');
AJ_p = load('AMASSSkeleton.txt');


%     value(i) = ((function_val(AJ_p, KJ) - function_val(AJ_o,KJ))/del)*z(i);
g = g + ((function_val(AJ_p, KJ) - function_val(AJ_o,KJ))/del)*z;
% end
% g = (1/length(z))*sum(value);
end
g = g/L;
end

