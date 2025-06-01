# Showcase Climate Controller - Nederlandse Projectbeschrijving

## Vraag

Voor dit project was de opgave om een geavanceerd klimaatcontrolesysteem te ontwikkelen dat in staat is om temperatuur en vochtigheid in een vitrine automatisch te regelen met behulp van PID-controle, sensoren en actuatoren, terwijl het gebruik maakt van een herbruikbare en uitbreidbare softwarearchitectuur. De keuze is gemaakt om de ESP32-S3 microcontroller te gebruiken omdat deze reeds bepaald was in het Casekeeper project en er veel raakvlakken waren tussen beide projecten, waardoor efficiënte hergebruik van bestaande componenten en kennis mogelijk werd. Er is bewust voor gekozen om de Casekeeper projectstructuur als basis te gebruiken vanwege deze raakvlakken, met de intentie om beide projecten te combineren.

## Aanpak

Het project is ontwikkeld met een object-georiënteerde aanpak die zorgt voor hergebruik van bestaande componenten. De software architectuur gebruikt een Device base class met DeviceRegistry singleton patroon (één centrale instantie voor apparaatbeheer) voor het beheren van alle I2C-apparaten via een I2C multiplexer. De configuratie wordt beheerd via JSON-bestanden op SD-kaart met fallback naar SPIFFS.

De ClimateController werkt met een cyclische update-methode die elke 5 seconden de sensorwaarden inleest van de temperatuur/vochtigheids sensor en op basis van PID-output de juiste actuatoren aanstuurt. Voor temperatuurcontrole activeert de PID-controller via digitale uitgangen de verwarmings- of koelingsfunctie, terwijl vochtigheidscontrole werkt met hysterese-logica voor de membranen.

## Resultaat

Het resultaat is een volledig functioneel klimaatcontrolesysteem dat automatisch temperatuur en vochtigheid in een vitrine kan regelen. De ClimateController vormt het hart van het systeem en werkt volgens een geavanceerde regelstrategie die verschillende actuatoren intelligent aanstuurt.

Voor temperatuurregeling gebruikt het systeem een geïntegreerde temperatuurcontroller die via drie digitale uitgangen van de GPIO expander wordt aangestuurd. Deze controller kan zowel verwarmen als koelen, waarbij de digitale signalen de verwarmings- of koelingsfunctie activeren zonder dat negatieve waarden verzonden hoeven te worden. De PID-algoritme berekent de benodigde output en bepaalt via de digitale uitgangen welke modus actief moet zijn.

De vochtigheidsregeling gebeurt door middel van twee omgekeerd gemonteerde membranen die door de GPIO expander worden aangestuurd. Deze membranen werken volgens hysterese-logica: wanneer de luchtvochtigheid te hoog wordt, activeren de membranen om vocht af te voeren, en wanneer deze te laag wordt, kunnen ze vocht toevoegen aan de lucht in de vitrine. Deze aanpak zorgt voor stabiele vochtigheidscontrole zonder constante aan/uit-schakeling.

Daarnaast regelt het systeem de luchtstroom via interior en exterior ventilatoren die eveneens door de GPIO expander worden aangestuurd. Deze ventilatoren zorgen voor luchtcirculatie binnen de vitrine en kunnen indien nodig externe lucht aanzuigen voor temperatuur- en vochtregeling.

Voor meer geavanceerde regeling biedt de GP8403 DAC analoge vermogensregeling (0-5V = 0-100% vermogen) voor extra verwarmings- en koelelementen, waardoor het systeem zeer nauwkeurig kan reageren op klimaatverschillen. De SHT sensor levert continu nauwkeurige temperatuur- en vochtigheidsmeting, terwijl het PCA9548A multiplexer systeem zorgt voor betrouwbare I2C-communicatie tussen alle componenten.

## Validatie

Het systeem is gevalideerd door middel van testing van alle hardware componenten en verificatie van de PID-controle algorithmes met de mobiele testopstelling van het prototype. De modulariteit is bevestigd door succesvolle integratie van de Casekeeper projectstructuur. Het systeem toont robuuste prestaties met automatische device discovery, EEPROM-gebaseerde configuratieopslag, en MQTT communicatie voor remote monitoring.
