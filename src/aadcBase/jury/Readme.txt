In diesem Ordner sind die Sourcen fuer die Jury und Driver Module enthalten

Folders:
* AADC_DriverFilter:	                Dieser Filter dient zur Vorlage f�r die  Teams (Prototyp). Er veranschaulicht den Empfang vom Jury Modul und das Senden des eigenen Status
* AADC_JuryModule:	                    Dieser Filter wird von der Jury zur �berwachung und Steuerung der Driver verwendet.
* AADC_JuryModule_connectionlib:	    Dies ist eine eigenst�ndige GUI Applikation auf Basis der connectionlib und wird von der Jury zur Steuerung des Fahrzeugs verwendet. 
                                        Um die Applikation zu bauen ist zus�tzlich die connectionlib notwendig. Sie ist nicht in den "normalen" AADC-Buildablauf eingebunden, 
                                        wird aber vorkompiliert unter bin f�r alle g�ngigen Plattformen ausgeliefert.


Files:
* CMakeLists.txt:	Datei der CMake Kette
* Readme.txt:		Diese Datei