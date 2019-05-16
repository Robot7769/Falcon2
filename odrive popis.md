
idle - vypnutý výkonový stupeň (řízení motoru )
startovní sekvence - lze použít i později 

1) motor calibration - zjištění odporu a indukčnosti vinutí - pouští do něj různé frekvence (píská) a měří fázový posuv
 - dá se uložit do Odrivu (musí se zvlášť dát Save)
2) encoder offset - zjištění natočení motoru (vzájemná poloha rotoru a statoru - pomalé točení tam a zpet
tyto dva údaje 1) a 2) odrive potřebuje, aby mohl přesně řídit motor -> zjištění pomocí enkodérů , proto točí kolem 
encoder index - pokud má vyvedený indexový drát (Z) - je to jedna konkrétní poloha v otáčce, je možné ho použít místo encoder offset, je to rychlejší,  může takto zjišťovat polohu

sensorless control  - řízení bez enkodérů - tam se žádné kalibrace nedělají, ale motor se nedá řídit pomalu a přesně 
(v podstatě se z toho stane modelářský regulátor )
Full calibration = motor calibration + encoder offset

Closed loop - uzavřená smyčka řízení -> zapne řízení motoru (nutná podmínka - hotová kalibrace)

Závěr: pro start motoru potřebuju encoder offset zapnout , po jeho konci spadne Odrive do Idle, potom zapnout Close loop -> a můžu jezdit 
if chci vypnout řízení motorů -> volnoběh, šetří baterku, -> přepnu do Idle , dokud nevypnu napájení, 

tak potom po Idle stačí znova zapnout Close loop 

if je chyba, tak reboot restartuje celý odrive, potom je potřeba ho znovu připojit (conect)

taky je možné Odrive resetovat -> chybový stav se vymaže 

Odrive má 4 logické vrstvy:



