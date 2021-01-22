clf
hold off
clear;
x = [1:0.5:10]
n = length(x)
x(10) = 2
t = linspace(0,1,n)
plot(t,x, 'b')
hold on

[f, fe, fev] = derivative_filter(x)
plot(t,f, 'og');
plot(t,fe, 'm');
plot(t,fev, 'r');