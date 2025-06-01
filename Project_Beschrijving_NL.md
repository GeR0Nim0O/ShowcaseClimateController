# Showcase Climate Controller - Nederlandse Projectbeschrijving

## Vraag

Voor dit project was de opgave om een geavanceerd klimaatcontrolesysteem te ontwikkelen dat in staat is om temperatuur en vochtigheid in een vitrine automatisch te regelen met behulp van PID-controle, sensoren en actuatoren, terwijl het gebruik maakt van een herbruikbare en uitbreidbare softwarearchitectuur. De keuze is gemaakt om de ESP32-S3 microcontroller te gebruiken omdat deze reeds bepaald was in het Casekeeper project en er veel raakvlakken waren tussen beide projecten, waardoor efficiënte hergebruik van bestaande componenten en kennis mogelijk werd. Er is bewust voor gekozen om de Casekeeper projectstructuur als basis te gebruiken vanwege deze raakvlakken, met de intentie om beide projecten te combineren.

## Aanpak

Het project is ontwikkeld met een object-georiënteerde aanpak die zorgt voor hergebruik van bestaande componenten. De software architectuur gebruikt een Device base class met DeviceRegistry singleton patroon voor het beheren van alle I2C-apparaten via een PCA9548A multiplexer. De configuratie wordt beheerd via JSON-bestanden op SD-kaart met fallback naar SPIFFS, en device discovery gebeurt automatisch via I2C scanning.

De ClimateController implementeert onafhankelijke PID-controllers voor temperatuur en vochtigheid met configureerbare parameters, safety monitoring met automatische noodstop functionaliteit, en fan control voor luchtcirculatie. Het systeem ondersteunt zowel digitale als analoge uitgangen voor actuators, waarbij de DAC wordt gebruikt voor variabele vermogensregeling (0-100%). Een rotary encoder met display zorgt voor gebruikersinteractie en real-time status updates.

## Resultaat

Het resultaat is een volledig functioneel klimaatcontrolesysteem dat automatisch temperatuur en vochtigheid in een vitrine kan regelen binnen instelbare grenzen. Het systeem beschikt over een modulaire library organisatie met Device, GPIO, Sensors, Display, DAC en ClimateController libraries. De hardware configuratie maakt gebruik van I2C device mapping via PCA9548A multiplexer en PCF8574 GPIO expander voor actuator controle binnen de vitrine.

De PCF8574 GPIO expander fungeert als het centrale aansturingspunt voor alle klimaatsystemen in de vitrine. Via de 8 digitale uitgangen worden verschillende actuatoren aangestuurd: verwarmingselementen, koelelementen, ventilatoren en luchtontvochtiger. De GPIO expander ontvangt commands van de ClimateController en zet deze om naar fysieke schakelingen die de gewenste klimaatomstandigheden realiseren binnen de vitrine.

## Validatie

Het testen van de hardware elementen is gedaan met de mobiele testopstelling van het prototype. Het systeem is gevalideerd door middel van uitgebreide testing van alle hardware componenten en verificatie van de PID-controle algorithmes onder verschillende omstandigheden. De modulariteit en herbruikbaarheid van de code is bevestigd door de succesvolle integratie van de Casekeeper projectstructuur. Het systeem toont robuuste prestaties met automatische device discovery, EEPROM-gebaseerde persistente opslag van klimaatinstellingen, en betrouwbare MQTT communicatie voor remote monitoring. De flexibele architectuur maakt toekomstige uitbreidingen en integratie met het Casekeeper project mogelijk.
