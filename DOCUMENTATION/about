Deze map bevat alle documentatie en kennis die tijdens het project is verzameld.
Het doel is om toekomstige ontwikkelaars een duidelijk en compleet overzicht te bieden, 
zodat zij het project efficiënt kunnen voortzetten en verbeteren!

#1 schema
    het bijgevoegde schema is een shema van de eerste opstelling, 
    die werkt dit maakt gebruik van 1 arduino (met het geuploade programma in MAIN) en een joystick-shield op elkaar 
        
#2 pinout diagrammen
    (schermafbeeldingen)

#3 hoe zou je datalogging van de device kunnen doen 
        - verbind de computer met de arduino via USB kabel
        - open/instaleer PUTTY 
        - selecteer serial-communicatie via de juiste COM poort 
        - enable logging naar een putty.log file
        - in de MAIN programma zou er moeten staan " #Define DEBUG 0 " dit zorgt ervoor dat alleen de loadcell waardes op de seriele bus komen
        - als alles correct is zou het mogelijk moeten zijn om de data op te slaan in .log file
        - deze kun je dan verwerken in exel of python 

#4 wij hadden problemen met de motor de juiste kracht op de motor te duwen , 
   veel onderzoek gedaan naar probleem in ACCURACY test en dan geanaliseerd, toen maakten we gebruik van een DRV8833
   geprobeerd met PID-regelaar, had niet zoveel impact 
   dan hebben we besloten om een nieuwe motordriver te gebruiken EN in plaats van NEWTON -> gram te gebruiken wat het eenvoudiger maakt
   nu stopt hij om de juiste moment()
   ook een Proportionele regelaar ingevoegd om wat nauwkeuriger te werken 
   wat nu het probleem is dat hij na deze controle niet direct of niet stopt want na dit worden supergroote gramwaarden gegeven!
   ik weet nu niet wat de echte gram waarde is

    
