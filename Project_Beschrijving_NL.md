# Showcase Climate Controller - Nederlandse Projectbeschrijving

## Vraag

Dit project ontwikkelt een automatisch klimaatcontrolesysteem voor vitrines dat temperatuur en luchtvochtigheid regelt via PID-controle, sensoren en actuatoren. De ESP32-S3 microcontroller werd gekozen omdat deze al bepaald was in het Casekeeper project, waardoor hergebruik van componenten en kennis mogelijk werd. De Casekeeper projectstructuur dient als basis met de intentie beide projecten te combineren.

## Aanpak

Het systeem gebruikt object-georiënteerde architectuur met een Device base class en DeviceRegistry singleton voor I2C-apparaatbeheer via multiplexer. Configuratie wordt opgeslagen in JSON-bestanden op SD-kaart (ClimateConfig.json, config.json) met SPIFFS fallback. De ClimateController leest periodiek sensorwaarden en stuurt via PID-output actuatoren aan: digitale uitgangen voor temperatuurcontrole en hysterese-logica voor luchtvochtigheidsregeling.

## Resultaat

Een functioneel klimaatcontrolesysteem dat automatisch temperatuur en luchtvochtigheid in vitrines regelt. De ClimateController stuurt via GPIO expander verschillende actuatoren aan: temperatuurcontroller met drie modi (Enable, Cool, Heat) en analoog vermogensignaal, twee omgekeerd gemonteerde membranen voor luchtvochtigheid (Humidify/Dehumidify met hysterese-logica), en interior/exterior ventilatoren voor luchtcirculatie via interne en externe radiatoren. Interne en externe sensoren leveren continue metingen voor nauwkeurige regeling.

## Validatie

Het systeem is gevalideerd door middel van testing van alle hardware componenten en verificatie van de PID-controle algorithmes met de mobiele testopstelling van het prototype. Debugging via de seriële monitor werd gebruikt om real-time systeemgedrag te monitoren en problemen op te sporen tijdens ontwikkeling en testing. De modulariteit is bevestigd door succesvolle integratie van de Casekeeper projectstructuur. Het systeem beschikt over automatische device discovery, EEPROM-gebaseerde (Electrically Erasable Programmable Read-Only Memory - niet-vluchtig geheugen) configuratieopslag, en MQTT communicatie (Message Queuing Telemetry Transport - lichtgewicht berichtenprotocol) voor remote monitoring.
