# Showcase Climate Controller - Nederlandse Projectbeschrijving

## Vraag

Voor dit project was de opgave om een geavanceerd klimaatcontrolesysteem te ontwikkelen dat in staat is om temperatuur en vochtigheid in een vitrine automatisch te regelen met behulp van PID-controle, sensoren en actuatoren, terwijl het gebruik maakt van een herbruikbare en uitbreidbare softwarearchitectuur. De keuze is gemaakt om de ESP32-S3 microcontroller te gebruiken omdat deze reeds bepaald was in het Casekeeper project en er veel raakvlakken waren tussen beide projecten, waardoor efficiënte hergebruik van bestaande componenten en kennis mogelijk werd. Er is bewust voor gekozen om de Casekeeper projectstructuur als basis te gebruiken vanwege deze raakvlakken, met de intentie om beide projecten te combineren.

## Aanpak

Het project is ontwikkeld met een object-georiënteerde aanpak die zorgt voor hergebruik van bestaande componenten. De software architectuur gebruikt een Device base class met DeviceRegistry singleton patroon (één centrale instantie voor apparaatbeheer) voor het beheren van alle I2C-apparaten via een PCA9548A multiplexer. De configuratie wordt beheerd via JSON-bestanden op SD-kaart met fallback naar SPIFFS.

De ClimateController werkt met een cyclische update-methode die elke 5 seconden de sensorwaarden inleest van de SHT sensor en op basis van PID-output de juiste actuatoren aanstuurt. Voor temperatuurcontrole activeert de PID-controller via digitale uitgangen de verwarmings- of koelingsfunctie, terwijl vochtigheidscontrole werkt met hysterese-logica voor de membranen.

## Resultaat

Het resultaat is een volledig functioneel klimaatcontrolesysteem dat automatisch temperatuur en vochtigheid in een vitrine kan regelen. De PCF8574 GPIO expander stuurt via digitale uitgangen een temperatuurcontroller, twee omgekeerd gemonteerde membranen voor luchtvochtigheidsregeling, en interior/exterior ventilatoren aan. De GP8403 DAC biedt analoge vermogensregeling (0-5V = 0-100% vermogen) voor verwarmings- en koelelementen.

## Validatie

Het systeem is gevalideerd door middel van testing van alle hardware componenten en verificatie van de PID-controle algorithmes met de mobiele testopstelling van het prototype. De modulariteit is bevestigd door succesvolle integratie van de Casekeeper projectstructuur. Het systeem toont robuuste prestaties met automatische device discovery, EEPROM-gebaseerde configuratieopslag, en MQTT communicatie voor remote monitoring.
