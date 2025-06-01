# Showcase Climate Controller - Nederlandse Projectbeschrijving

## Vraag

Dit project ontwikkelt een automatisch klimaatcontrolesysteem voor vitrines dat temperatuur en luchtvochtigheid regelt via PID-controle, sensoren en actuatoren. De ESP32-S3 microcontroller werd gekozen omdat deze al bepaald was in het Casekeeper project, waardoor hergebruik van componenten en kennis mogelijk werd. De Casekeeper projectstructuur dient als basis met de intentie beide projecten te combineren.

## Aanpak

Het systeem gebruikt object-georiënteerde architectuur met een Device base class en DeviceRegistry singleton voor I2C-apparaatbeheer via multiplexer. Configuratie wordt opgeslagen in JSON-bestanden op SD-kaart (ClimateConfig.json, config.json) met SPIFFS fallback. De ClimateController leest periodiek sensorwaarden en stuurt via PID-output actuatoren aan: digitale uitgangen voor temperatuurcontrole en hysterese-logica voor luchtvochtigheidsregeling.

## Resultaat

Een functioneel klimaatcontrolesysteem dat automatisch temperatuur en luchtvochtigheid in vitrines regelt. De ClimateController stuurt via GPIO expander verschillende actuatoren aan: temperatuurcontroller met drie modi (Enable, Cool, Heat) en analoog vermogensignaal, twee omgekeerd gemonteerde membranen voor luchtvochtigheid (Humidify/Dehumidify met hysterese-logica), en interior/exterior ventilatoren voor luchtcirculatie via interne en externe radiatoren. Interne en externe sensoren leveren continue metingen voor nauwkeurige regeling.

## Validatie

Het systeem is gevalideerd door hardware testing en PID-verificatie met de mobiele testopstelling. Debugging via seriële monitor werd gebruikt voor real-time monitoring tijdens ontwikkeling. De modulariteit is bevestigd door succesvolle Casekeeper integratie. Het systeem beschikt over automatische device discovery, EEPROM configuratieopslag en MQTT remote monitoring.
