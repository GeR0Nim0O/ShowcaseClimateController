# Showcase CDe ClimateController implementeert onafhankelijke PID-controllers voor temperatuur en vochtigheid met configureerbare parameters, safety monitoring met automatische noodstop functionaliteit, en fan control voor luchtcirculatie. Het systeem ondersteunt zowel digitale als analoge uitgangen voor actuators, waarbij de DAC wordt gebruikt voor variabele vermogensregeling (0-100%). Een rotary encoder met display zorgt voor gebruikersinteractie en real-time status updates.imate Controller - Nederlandse Projectbeschrijving

## Vraag

Voor dit project was de opgave om een geavanceerd klimaatcontrolesysteem te ontwikkelen dat in staat is om temperatuur en vochtigheid in een vitrine automatisch te regelen met behulp van PID-controle, sensoren en actuatoren, terwijl het gebruik maakt van een herbruikbare en uitbreidbare softwarearchitectuur. De keuze is gemaakt om de ESP32-S3 microcontroller te gebruiken omdat deze reeds bepaald was in het Casekeeper project en er veel raakvlakken waren tussen beide projecten, waardoor efficiënte hergebruik van bestaande componenten en kennis mogelijk werd. Er is bewust voor gekozen om de Casekeeper projectstructuur als basis te gebruiken vanwege deze raakvlakken, met de intentie om beide projecten te combineren.

## Aanpak

Het project is ontwikkeld met een object-georiënteerde aanpak die zorgt voor hergebruik van bestaande componenten en een efficiënte ontwikkelingscyclus. De belangrijkste toevoegingen zijn de `ClimateController` klasse voor PID-gebaseerde klimaatregeling, de `ClimateConfig` klasse voor persistente configuratieopslag in EEPROM, en uitgebreide DAC-ondersteuning voor analoge vermogensregeling.

De software architectuur gebruikt een Device base class met een DeviceRegistry singleton patroon voor het beheren van alle I2C-apparaten via een PCA9548A multiplexer. Alle apparaten (PCF8574 GPIO expander, SHT31 temperatuur/vochtigheid sensoren, GP8403 DAC, DS3231 RTC, en Display) zijn geïmplementeerd als afgeleide klassen van de Device base class. De configuratie wordt beheerd via JSON-bestanden op SD-kaart met fallback naar SPIFFS, en device discovery gebeurt automatisch via I2C scanning met positionele indexering.

De ClimateController implementeert onafhankelijke PID-controllers voor temperatuur en vochtigheid met configureerbare parameters, safety monitoring met automatische noodstop functionaliteit, en fan control voor luchtcirculatie. Het systeem ondersteunt zowel digitale als analoge uitgangen voor actuators, waarbij de DAC wordt gebruikt voor variabele vermogensregeling (0-100%). Een rotary encoder met display zorgt voor gebruikersinteractie en real-time status updates.

## Resultaat

Het resultaat is een volledig functioneel klimaatcontrolesysteem dat automatisch temperatuur en vochtigheid in een vitrine kan regelen binnen instelbare grenzen. Het systeem beschikt over een uitgebreide device hiërarchie met Device base class, PCF8574_GPIO voor GPIO expansion, SHT31_Sensor voor temperatuur/vochtigheid meting, Display voor weergave, GP8403dac voor digital-to-analog conversie, en RotaryEncoder voor gebruikersinvoer.

De library organisatie is gestructureerd in modulaire componenten: Device library voor de base class en DeviceRegistry, GPIO library voor PCF8574 GPIO expander, Sensors library voor verschillende sensortypes (SHT31, BH1705, SCALES), Display library voor display management, DAC library voor GP8403 power control, en ClimateController library voor de hoofdregellogica met PID-controle. Een Config library beheert de ClimateConfig met EEPROM-gebaseerde settings opslag.

Het hardware configuratie maakt gebruik van I2C device mapping via PCA9548A multiplexer met verschillende kanalen voor elk apparaat type, GPIO pin assignments voor rotary encoder en I2C communicatie, en PCF8574 GPIO expander pin mapping voor actuator controle binnen de vitrine. De software features omvatten I2C bus scanning, automatische device discovery, PID-gebaseerde klimaatregeling voor vitrine omstandigheden, persistente configuratie opslag, en veiligheidsmonitoring met noodstop functionaliteit.

## Validatie

Het testen van de hardware elementen is gedaan met de mobiele testopstelling van het prototype. Het systeem is gevalideerd door middel van uitgebreide testing van alle hardware componenten, verificatie van de PID-controle algorithmes onder verschillende omstandigheden, en validatie van de veiligheidsfuncties inclusief sensor failure detection en emergency shutdown procedures. De modulariteit en herbruikbaarheid van de code is bevestigd door de succesvolle integratie van de Casekeeper projectstructuur en de mogelijkheid om eenvoudig nieuwe device types toe te voegen via het DeviceRegistry pattern.

De automatische device discovery en configuratie via JSON-bestanden is getest en werkt betrouwbaar, terwijl de EEPROM-gebaseerde persistente opslag van klimaatinstellingen zorgt voor behoud van configuraties na power cycles. Het systeem toont robuuste prestaties met real-time monitoring, gebruiksvriendelijke interface via rotary encoder en OLED display, en betrouwbare MQTT communicatie voor remote monitoring en data logging. De flexibele architectuur maakt toekomstige uitbreidingen en integratie met het Casekeeper project mogelijk, waarbij beide systemen kunnen samenwerken voor uitgebreide showcase functionaliteit.
