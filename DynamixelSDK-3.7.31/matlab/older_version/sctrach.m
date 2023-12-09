clear
clc

t = -1:0.01:1;
a = 0.5*cos(pi.*t)+0.5;
width = 1/6;
threshold = 0.5*cos(pi*width)+0.5;
sqr_wave = a > threshold;
figure
plot(t,a)
hold on
plot(t, sqr_wave)
xlim([-1.5, 1.5])
ylim([-0.1, 1.5])