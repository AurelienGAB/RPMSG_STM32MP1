
Nous retrouvons dans le dossier Cortex A7, l'espace utilisateur et le module noyau. 
Et dans le dossier Cortex M4, le programme qui va transmettre les données acquises par SPI. 

<h2>Compilation sous Zephyr</h2>

Pour compiler un programme sous Zephyr, il nous faut créer un répertoire *build* , et lancer la commande **make** dans celui-ci : 

```
mkdir build && cd build
cmake -DBOARD=stm32mp157c_dk2 && make 
```

A la suite de la compilation, plusieurs fichiers ont été crée. Mais celui qui nous intéresse tout particulièrement est présent dans le répertoire *zephyr* avec l'extension **.elf**, par exemple *zephyr.elf*. 
On le transfert par la suite, sur la plateforme dans le répertoire */lib/firmware* pour pouvoir le lancer :

```
scp /zephyr/zephyr.elf root@192.168.1.30:/lib/firmware
```

<h2>Acquisition des données</h2>
Pour lancer une acquisition de données une fois sur la plateforme, il nous faut d'abord insérer le module noyau et chargé le programme sur le cortex M4 : 

```
insmod stm32-rpmsg
echo zephy.elf > /sys/class/remoteproc/remoteproc0/firmware
```

Une fois cette étape réalisée, dans un second terminal, on lance le programme utilisateur :
```
./user_space.out
```
Et enfin on lance le programme présent dans *remoteproc0* avec : 
```
echo start > /sys/class/remoteproc/remoteproc0/state
```
Les tâches en cours d'éxecution sur le M4 sont accessibles à l'aide de *debugfs* : 
```
cat /sys/kernel/debug/remoteproc/remoteproc0/trace0
```
Pour arrêter le traitement, on change l'état du *remoteproc0* :
```
echo stop > /sys/class/remoteproc/remoteproc0/state
```
Si toutes les étapes se sont correctement déroulées un fichier **data** a du être crée dans le répertoire où se situe le programme utilisateur. Ce dernier contient alors les différents données acquises que l'on peut ensuite représenter sous Octave par exemple.
