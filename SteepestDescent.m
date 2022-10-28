function [betas, history_b, history_f, AJMov,AV] = SteepestDescent(betas_i,step,eps,AJ,KJ)
%B_i = initial shape parameters
%step = stepsize
%eps = convergence criteria

%This function also needs
%function value (f(B)) 
%Gradient value (\nabla f(B)) given by a gradient estimate from homegrown
%function

betas = betas_i;
save('betas','betas')
[status,result] = system('python C:\Users\emmay\Desktop\AMASS\amass-master\notebooks\AMASSBody.py');
AJ = load('AMASSSkeleton.txt');
AJMov(1).I(:,:) = AJ; 
AV(1).I(:,:) = load("AMASSVerts.txt");

%calculate function with original betas
f(1) = function_val(AJ, KJ);


delF = 100; %just to initialize the algorithm
count = 1;
history_b(:,count) = betas;
history_f(:,count) = f(count);

while count < 4000 %delF > eps
    disp("Steepest descent iteration number")
    disp(count)
    disp("Shape parameters:")
    disp(betas)
    disp("Stopping criteria check")
    disp(delF)
    if count > 1
        disp("Norm of gradient")
        disp(norm(g))
    end
    %calculate gradient
    g = gradient_test(betas,KJ);
    
    %perform steepest descent to get B_p
    betas = reshape(betas,10,1)
    B_p = betas - step*g;
    betas = B_p;
    
    %collect function value for stopping criteria
    
    count = count + 1;
    
    history_b(:,count) = betas;
    
    save('betas','betas')
    [status,result] = system('python C:\Users\emmay\Desktop\AMASS\amass-master\notebooks\AMASSBody.py');
    AJ = load('AMASSSkeleton.txt');
    x = load('AMASSSkeleton.txt');
%     disp('AJtest')
%     disp(AJ(1,:))
    AJMov(count).I(:,:) = x(:,:); 
    x = load("AMASSVerts.txt");
    AV(count).I(:,:) = x(:,:);
    f(count) = function_val(AJ, KJ)
    
    history_f(:,count) = f(count);
    delF = abs(f(count) - f(count-1));

end

