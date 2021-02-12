clear all;
close all;
clc all;

fileID =fopen('data_simple');%data13
A= fread(fileID);

fileID2 =fopen('data_thread');
B= fread(fileID2);

fileID3 =fopen('data_semaphore');
C= fread(fileID3);

AA=A(2:2:end)*256+A(1:2:end);
BB=B(2:2:end)*256+B(1:2:end);
CC=C(2:2:end)*256+C(1:2:end);

subplot(3,1,1)
plot(AA(4300:4800),'*-')
title("Fréquence initiale F=5kHz sans Thread")
xlabel("Nombre d'échantillons (a.u)");
ylabel("Amplitude");
xlim([0 500]);
grid on;

subplot(3,1,2)
plot(BB(4300:4800),'*-')%80825- 80875
title("Fréquence initiale F=5kHz avec Thread");
xlabel("Nombre d'échantillons (a.u)");
ylabel("Amplitude");
xlim([0 500]);
grid on;

subplot(3,1,3)
plot(CC(4300:4800),'*-')%80825- 80875
title("Fréquence initiale F=5kHz avec Sémaphore");
xlabel("Nombre d'échantillons (a.u)");
ylabel("Amplitude");
xlim([0 500]);
grid on;



