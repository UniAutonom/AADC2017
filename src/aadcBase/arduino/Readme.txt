In diesem Ordner sind die Sourcen der Arduino-Filter enthalten.

Folders:
* AADC_ArduinoCommunicaton: 	Dieser Filter bildet die serielle Schnittstelle ab und kommuniziert �ber diese mit dem Arduino
* AADC_ArduinoDemux:		Mit diesem Filter werden die verschiedenen Signale, die ueber die serielle Schnittstelle kommen, auf einzelne Pins aufgeteilt, so dass die verschiedenen Sensorarten einzeln zur Verfuegung stehen
* AADC_ArduinoMux:		Mit diesem Filter werden die verschiedenen Befehle f�r die Aktoren zusammengefasst und koennen dann an den ArduinoCommunication uebertragen werden, der diese ueber die serielle Schnittstelle zum Arduino schickt
* AADC_All_Sensors: Dieses Plugin enth�lt mehrere Filter. Alle Filter sind dazu da, die einzelnen vom AADC_ArduinoDemux zusammengefassten Sensorarten in einzelne Signale aufzul�sen, die dann als gewoehnliche Floats verarbeitet werden koennen.

Files:
* CMakeLists.txt:	Datei der CMake kette
* Readme.txt:		Diese Datei