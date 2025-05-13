clc
clear
close


data = load('copstream.txt');
copref0 = data(:,1);
copref1 = data(:,2);
copsolved0 = data(:,3);
copsolved1 = data(:,4);

comsolved0 = data(:,7);
comsolved1 = data(:,8);
comvsolved0 = data(:,9);
comvsolved1 = data(:,10);
comasolved0 = data(:,11);
comasolved1 = data(:,12);

comadded0 = data(:,7);
comadded1 = data(:,8);
comvadded0 = data(:,9);
comvadded1 = data(:,10);
comaadded0 = data(:,11);
comaadded1 = data(:,12);


subplot(2,2,1)
hold on
plot(copref0)
plot(copsolved0)
plot(comsolved0)

subplot(2,2,2)
hold on
plot(copref1)
plot(copsolved1)
plot(comsolved1)

subplot(2,2,3)
hold on
plot(copsolved1,copsolved0,'*')
plot(comsolved1,comsolved0)
set(gca,'xdir','reverse')

subplot(2,2,4)
hold on
plot(copref0)
plot(copref1)
plot(comvsolved0)
%plot(comvsolved1)


grid on;

