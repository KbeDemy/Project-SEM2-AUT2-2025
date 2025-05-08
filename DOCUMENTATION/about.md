# Projectdocumentatie

Deze map bevat alle documentatie en kennis die tijdens het project is verzameld.  
Het doel is om toekomstige ontwikkelaars een duidelijk en compleet overzicht te bieden, zodat zij het project efficiënt kunnen voortzetten en verbeteren.

---

## 1. Schema

Het bijgevoegde schema toont de eerste werkende opstelling.  
Deze bestaat uit:
- 1 Arduino (met het programma geüpload vanuit `MAIN`)
- Een joystick-shield dat rechtstreeks op de Arduino geplaatst is

---

## 2. Pinout-diagrammen

Zie de bijgevoegde schermafbeeldingen voor het overzicht van de pinout van de gebruikte hardware.

---

## 3. Datalogging van het device

**Stappen om datalogging op te zetten via de seriële monitor:**

1. Verbind de computer met de Arduino via een USB-kabel.
2. Open of installeer [PuTTY](https://www.putty.org/).
3. Selecteer *Serial*-communicatie via de juiste COM-poort.
4. Activeer logging naar een bestand (`putty.log`).
5. Controleer in het `MAIN`-programma of de volgende regel aanwezig is:

   ```cpp
   #define DEBUG 0.
   
6. Als alles correct is ingesteld, wordt de data gelogd in een .log-bestand.
7. De gelogde data kan daarna worden verwerkt in Excel of met een Python-script.

---

## 4. Problemen met de motor en oplossing en nog meer problemen
We ondervonden problemen om de juiste kracht over te brengen op de motor.
Na uitgebreid onderzoek tijdens de ACCURACY-test en analyse, hebben we de volgende stappen ondernomen:

Gebruik van de DRV8833 motordriver.

Een PID-regelaar geprobeerd, maar dit had weinig effect.

Besloten om een nieuwe motordriver te gebruiken.

Eenheid veranderd van Newton naar gram, wat eenvoudiger werkt.

Het systeem stopt nu op het juiste moment.

Een proportionele regelaar werd toegevoegd voor extra nauwkeurigheid.

Huidig probleem:
Na de controle blijft de motor soms verder draaien of stopt hij niet onmiddellijk.
Bovendien geeft het systeem extreem hoge gramwaardes na het stoppen, waardoor we niet weten wat de echte waarde is.
