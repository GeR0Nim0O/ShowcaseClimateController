# Showcase Climate Controller - Nederlandse Projectbeschrijving

## Vraag

Voor dit project was de opgave om een geavanceerd klimaatcontrolesysteem te ontwikkelen dat in staat is om temperatuur en vochtigheid in een vitrine automatisch te regelen met behulp van PID-controle, sensoren en actuatoren, terwijl het gebruik maakt van een herbruikbare en uitbreidbare softwarearchitectuur. De keuze is gemaakt om de ESP32-S3 microcontroller te gebruiken omdat deze reeds bepaald was in het Casekeeper project en er veel raakvlakken waren tussen beide projecten, waardoor efficiënte hergebruik van bestaande componenten en kennis mogelijk werd. Er is bewust voor gekozen om de Casekeeper projectstructuur als basis te gebruiken vanwege deze raakvlakken, met de intentie om beide projecten te combineren.

## Aanpak

Het project is ontwikkeld met een object-georiënteerde aanpak die zorgt voor hergebruik van bestaande componenten. De software architectuur gebruikt een Device base class met DeviceRegistry singleton patroon voor het beheren van alle I2C-apparaten via een PCA9548A multiplexer. De configuratie wordt beheerd via JSON-bestanden op SD-kaart met fallback naar SPIFFS, en device discovery gebeurt automatisch via I2C scanning.

De ClimateController implementeert onafhankelijke PID-controllers voor temperatuur en vochtigheid met configureerbare parameters, safety monitoring met automatische noodstop functionaliteit, en fan control voor luchtcirculatie. Het systeem ondersteunt zowel digitale als analoge uitgangen voor actuators, waarbij de DAC wordt gebruikt voor variabele vermogensregeling (0-100%). Een rotary encoder met display zorgt voor gebruikersinteractie en real-time status updates.

De ClimateController werkt met een cyclische update-methode die elke 5 seconden de sensorwaarden inleest van de SHT31 sensor, deze doorgeeft aan de PID-controllers, en op basis van de PID-output de juiste actuatoren aanstuurt. Voor temperatuurcontrole bepaalt de PID-controller de benodigde actie en activeert via digitale uitgangen ofwel de verwarmings- ofwel de koelingsfunctie van de temperatuurcontroller, terwijl vochtigheidscontrole werkt met hysterese-logica voor stabiele aan/uit-schakeling van de membranen.

## Resultaat

Het resultaat is een volledig functioneel klimaatcontrolesysteem dat automatisch temperatuur en vochtigheid in een vitrine kan regelen binnen instelbare grenzen. Het systeem beschikt over een modulaire library organisatie met Device, GPIO, Sensors, Display, DAC en ClimateController libraries. De hardware configuratie maakt gebruik van I2C device mapping via PCA9548A multiplexer en PCF8574 GPIO expander voor actuator controle binnen de vitrine.

De PCF8574 GPIO expander fungeert als het centrale aansturingspunt voor alle klimaatsystemen in de vitrine. Via de 8 digitale uitgangen worden verschillende actuatoren aangestuurd: een temperatuurcontroller, twee membranen voor luchtvochtigheidsregeling, en ventilatoren. De GPIO expander ontvangt commands van de ClimateController en zet deze om naar fysieke schakelingen die de gewenste klimaatomstandigheden realiseren binnen de vitrine.

Het aansturingssysteem werkt via specifieke pin-toewijzingen: pin 0 en 1 voor interior/exterior ventilatoren (automatisch geactiveerd bij klimaatcontrole), pin 2 en 3 voor twee omgekeerd gemonteerde membranen die samen de luchtvochtigheid regelen, en pin 4, 5 en 6 voor de temperatuurcontroller waarbij pin 4 de enable functie verzorgt en pin 5 en 6 respectievelijk koeling en verwarming aansturen van hetzelfde temperatuurregelingsapparaat. De GP8403 DAC biedt aanvullende analoge controle voor variabele vermogensregeling van verwarmings- en koelelementen, waarbij het voltage (0-5V) proportioneel is aan het gewenste vermogen (0-100%).

## Validatie

Het testen van de hardware elementen is gedaan met de mobiele testopstelling van het prototype. Het systeem is gevalideerd door middel van uitgebreide testing van alle hardware componenten en verificatie van de PID-controle algorithmes onder verschillende omstandigheden. De modulariteit en herbruikbaarheid van de code is bevestigd door de succesvolle integratie van de Casekeeper projectstructuur. Het systeem toont robuuste prestaties met automatische device discovery, EEPROM-gebaseerde persistente opslag van klimaatinstellingen, en betrouwbare MQTT communicatie voor remote monitoring. De flexibele architectuur maakt toekomstige uitbreidingen en integratie met het Casekeeper project mogelijk.
