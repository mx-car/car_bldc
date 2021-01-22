function [f ferror1 ferror1_var ferror2 ] = derivative_filter(x, threshold_error, threshold_var)
% filters a signal by replacing single spikes with a predicted value
% a error variance is computed to juge if a predicted value is useful.
%   x               = signal input vector
%   f               = filtered vaules of x
%   ferror1         = difference between f and x
%   ferror1_var     = variance of ferror1
%   ferror2         = difference between prediction using the filter and x
%   threshold_error = threhold do identify spikes usi
%   threshold_var   = threhold on the variance of the error between prediction and signal


  error_window_size = 5;
  n = length(x)
  f = zeros(n,1);
  ferror1 = zeros(n,1);
  ferror2 = zeros(n,1);
  ferror1_var = ones(n,1)*2*threshold_var;
  m = 3;
  w = [0, x(1:m-1)];
  for i = m:n
    w = circshift(w,-1)
    w(m) = x(i);
    dx = w(m-1) - w(m-2)
    xh = w(m-1) + dx;
    ferror2(i) = w(m) - xh;
    if(i > error_window_size)
      error_window = ferror1(i-error_window_size:i);
      ferror1_var(i) = var(error_window);
    end
    f(i) = w(m);
    if ferror1_var(i-1) < threshold_var
      if (abs(ferror2(i))  > threshold_error)
        w(m) = xh;
        f(i) = xh;
      end
    end
    ferror1(i) = f(i) - x(i);
  end
end