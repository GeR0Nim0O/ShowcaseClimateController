# Showcase Climate Controller - Nederlandse Projectbeschrijving

## Vraag

Dit project ontwikkelt een automatisch klimaatcontrolesysteem voor vitrines dat temperatuur en luchtvochtigheid regelt via PID-controle, sensoren en actuatoren. De ESP32-S3 microcontroller werd gekozen omdat deze al bepaald was in het Casekeeper project, waardoor hergebruik van componenten en kennis mogelijk werd. De Casekeeper projectstructuur dient als basis met de intentie beide projecten te combineren.

## Aanpak

Het systeem gebruikt object-georiënteerde architectuur met een Device base class en DeviceRegistry singleton voor I2C-apparaatbeheer via multiplexer. Configuratie wordt opgeslagen in JSON-bestanden op SD-kaart (ClimateConfig.json, config.json) met SPIFFS fallback. De ClimateController leest periodiek sensorwaarden en stuurt via PID-output actuatoren aan: digitale uitgangen voor temperatuurcontrole en hysterese-logica voor luchtvochtigheidsregeling.

## Resultaat

Een functioneel klimaatcontrolesysteem dat automatisch temperatuur en luchtvochtigheid in vitrines regelt. De ClimateController stuurt via GPIO expander verschillende actuatoren aan: temperatuurcontroller met drie modi (Enable, Cool, Heat) en analoog vermogensignaal, twee omgekeerd gemonteerde membranen voor luchtvochtigheid (Humidify/Dehumidify met hysterese-logica), en interior/exterior ventilatoren voor luchtcirculatie via interne en externe radiatoren. Interne en externe sensoren leveren continue metingen voor nauwkeurige regeling.

## Validatie

Het systeem is gevalideerd door hardware testing en PID-verificatie met de mobiele testopstelling. Debugging via seriële monitor werd gebruikt voor real-time monitoring tijdens ontwikkeling. De modulariteit is bevestigd door succesvolle Casekeeper integratie. Het systeem beschikt over automatische device discovery, EEPROM configuratieopslag en MQTT remote monitoring.

---

# Showcase Climate Controller - Uitgebreide Projectbeschrijving

## Vraag

Voor dit project was de opgave om een geavanceerd klimaatcontrolesysteem te ontwikkelen dat in staat is om temperatuur en luchtvochtigheid in een vitrine automatisch te regelen met behulp van PID-controle, sensoren en actuatoren, terwijl het gebruik maakt van een herbruikbare en uitbreidbare softwarearchitectuur. De keuze is gemaakt om de ESP32-S3 microcontroller te gebruiken omdat deze reeds bepaald was in het Casekeeper project en er veel raakvlakken waren tussen beide projecten, waardoor efficiënte hergebruik van bestaande componenten en kennis mogelijk werd. Er is bewust voor gekozen om de Casekeeper projectstructuur als basis te gebruiken vanwege deze raakvlakken, met de intentie om beide projecten te combineren.

## Aanpak

Het project is ontwikkeld met een object-georiënteerde aanpak die zorgt voor hergebruik van bestaande componenten. De software architectuur gebruikt een Device base class met DeviceRegistry singleton patroon (één centrale instantie voor apparaatbeheer) voor het beheren van alle I2C-apparaten (Inter-Integrated Circuit communicatieprotocol) via een I2C multiplexer. Het systeem gebruikt een dubbele configuratiemethode waarbij de primaire configuratie wordt opgeslagen in JSON-bestanden op SD-kaart (ClimateConfig.json voor klimaatinstellingen en config.json voor algemene systeeminstellingen) met automatische fallback naar SPIFFS (SPI Flash File System - ingebouwd flashgeheugen voor bestandsopslag) wanneer de SD-kaart niet beschikbaar is. Deze aanpak zorgt voor flexibele configuratie-aanpassingen via de SD-kaart terwijl systeem betrouwbaarheid gegarandeerd blijft. De ClimateController werkt met een cyclische update-methode die periodiek de sensorwaarden inleest van de interne en externe temperatuur en luchtvochtigheid sensoren en op basis van PID-output (Proportional-Integral-Derivative regelalgoritme) de juiste actuatoren aanstuurt. Voor temperatuurcontrole activeert de PID-controller via digitale uitgangen de verwarmings- of koelingsfunctie, terwijl luchtvochtigheidscontrole werkt met hysterese-logica voor de membranen.

## Resultaat

Het resultaat is een volledig functioneel klimaatcontrolesysteem dat automatisch temperatuur en luchtvochtigheid in een vitrine kan regelen. De ClimateController is het hoofddoel van dit project en werkt volgens een geavanceerde regelstrategie die verschillende actuatoren intelligent aanstuurt. Voor temperatuurregeling gebruikt het systeem een geïntegreerde temperatuurcontroller die drie digitale inputs heeft voor modus-selectie (TemperatureEnable, TemperatureCool, TemperatureHeat) en waarvan het vermogen wordt geregeld met een analoog 0-5V signaal (TemperaturePower). De PID-algoritme berekent de benodigde output en bepaalt via de PCF8574 GPIO expander uitgangen welke modus actief moet zijn. De luchtvochtigheidsregeling gebeurt door middel van twee omgekeerd gemonteerde membranen die door de GPIO expander worden aangestuurd (Humidify, Dehumidify), waarbij slechts één membraan tegelijk kan worden ingeschakeld. Deze membranen werken volgens hysterese-logica: wanneer de luchtvochtigheid in de vitrine te hoog wordt, activeert het dehumidify membraan om vocht af te voeren, en wanneer deze te laag wordt, activeert het humidify membraan om vocht toe te voegen aan de lucht in de vitrine. Deze aanpak zorgt voor stabiele luchtvochtigheidscontrole zonder constante aan/uit-schakeling. Daarnaast regelt het systeem de luchtstroom via interior en exterior ventilatoren die eveneens door de GPIO expander worden aangestuurd (FanExterior, FanInterior). De interne ventilatoren verdelen de temperatuur-geregelde lucht door de vitrine, door circulatie via de interne radiator. De externe ventilatoren voeren de warme of koele lucht (afhankelijk van de modus) naar de buitenlucht af, door verse lucht door de externe radiator te blazen. De interne en externe temperatuur en luchtvochtigheid sensoren leveren continu nauwkeurige metingen, terwijl het multiplexer systeem zorgt voor betrouwbare I2C-communicatie tussen alle componenten.

## Validatie

Het systeem is gevalideerd door middel van testing van alle hardware componenten en verificatie van de PID-controle algorithmes met de mobiele testopstelling van het prototype. Debugging via de seriële monitor werd gebruikt om real-time systeemgedrag te monitoren en problemen op te sporen tijdens ontwikkeling en testing. De modulariteit is bevestigd door succesvolle integratie van de Casekeeper projectstructuur. Het systeem beschikt over automatische device discovery, EEPROM-gebaseerde (Electrically Erasable Programmable Read-Only Memory - niet-vluchtig geheugen) configuratieopslag, en MQTT communicatie (Message Queuing Telemetry Transport - lichtgewicht berichtenprotocol) voor remote monitoring.
