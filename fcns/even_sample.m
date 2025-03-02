function [Et, Ex] = even_sample(t, x, Fs, type)



if nargin < 4, type = 'linear'; end



dt = diff(t);

dt = dt + (dt==0)*1e-5;

t = [t(1);t(1)+cumsum(dt)];



% Obtain the process related parameters

N = size(x, 2);    % number of signals to be interpolated

M = size(t, 1);    % Number of samples provided

t0 = t(1,1);       % Initial time

tf = t(M,1);       % Final time

EM = (tf-t0)*Fs;   % Number of samples in the evenly sampled case with

                   % the specified sampling frequency

Et = linspace(t0, tf, round(EM))';



% Using linear interpolation (used to be cubic spline interpolation)

% and re-sample each signal to obtain the evenly sampled forms

for s = 1:N

  Ex(:,s) = interp1(t(:,1), x(:,s), Et(:,1),type); 

end

end
