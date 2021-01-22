clf
m = csvread ('motor03.csv')
n = size(m)(1)
t = m(:,1)
hold off;

%plot(m(:,1), m(:,2),";Rotor ASxxxx;",m(:,1), m(:,3),";Theta ASxxxx;", m(:,1), m(:,4),";Theta SVM;");
%hold on;

a = deg2rad(m(:,3));
s = cos(a);
plot(t,s, 'LineWidth', 1, 'b','DisplayName','Input');
hold on

[f, fe1, fe1var, fe2] = derivative_filter(s', 0.4, 0.1)

plot(t,f, 'og','DisplayName','Output');
plot(t,fe1, 'r','DisplayName','Error Input-Output');
plot(t,fe1var, 'r','DisplayName','ErrorVarianz');
plot(t,fe2, 'b','DisplayName','Error on prediction');
legend