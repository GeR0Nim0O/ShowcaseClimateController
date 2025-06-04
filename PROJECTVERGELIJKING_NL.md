# Showcase Climate Controller - Ontwikkeling en Verbeteringen

## Casekeeper - Basis Project

**Casekeeper** is een passief monitoring systeem voor klimaatcondities, gebouwd op een ESP32-POE-ISO platform. Het systeem verzamelt sensordata van temperatuur, vochtigheid en lichtsterkte en publiceert deze via MQTT naar een centrale server.

### Kernfuncties Casekeeper
- **Hardware**: ESP32-POE-ISO (4MB Flash)
- **Sensoren**: SHT31 (temperatuur/vochtigheid), BH1705 (licht), HX711 (gewicht)
- **Connectiviteit**: WiFi + MQTT met NTP time synchronization
- **Architectuur**: Modulaire opbouw met DeviceRegistry pattern
- **Interface**: Serial debugging voor ontwikkeling
- **Opslag**: Runtime JSON configuratie zonder persistentie

### Code Structuur Casekeeper
```
Modules: Device, Sensors, GPIO, RTC
- DeviceRegistry voor automatische device discovery
- Configuration singleton voor runtime instellingen
- Basis error handling
```

---

## Showcase Climate Controller - Toevoegingen en Verbeteringen

**Showcase Climate Controller** bouwt voort op Casekeeper en transformeert het van een passief monitoring systeem naar een volledig actief klimaatregelsysteem.

### Hardware Verbeteringen
- **Platform**: Upgrade naar ESP32-S3 Box (16MB Flash + PSRAM)
- **Prestaties**: QIO flash mode voor 40% snellere data toegang
- **Debugging**: Maximum debug level (5) voor uitgebreide ontwikkelingsondersteuning
- **Geheugen**: 4x meer Flash geheugen voor complexere applicaties

### Nieuwe Functionaliteit
- **Temperatuurregeling**: PID controller voor actieve temperatuurregeling
- **Luchtvochtigheidsregeling**: Setpoint controle voor actieve luchtvochtigheidsregeling
- **Volledig Automatische Klimaat Controle**: Vermogensgestuurde regeling
  - **Temperatuur**: Cool/Heat/Off via 0-5V DAC vermogensregeling
  - **Luchtvochtigheid**: Humidify/Dehumidify/Off
- **Uitgangssignalen**: 
  - **0-5V DAC**: Vermogensregeling van temperatuur actuatoren
  - **GPIO Expander**: Aansturen van temperatuur regelaar, luchtvochtigheid regelaar, en interne/externe luchtcirculatie
- **Sensor Uitbreiding**: Rotary encoder voor interactieve bediening

### User Interface Toevoegingen
- **LCD Display**: Real-time status weergave van alle systeem parameters
- **Rotary Encoder**: Interactieve bedieningselement voor menu navigatie
- **Visuele Feedback**: Directe feedback via display interface
- **Interactive Menu**: Instelbare parameters zonder code wijzigingen
- **Permanente Opslag**: Configuratie wijzigingen worden opgeslagen

### Architectuur Verbeteringen
- **Uitgebreide Modules**: 7+ gespecialiseerde modules vs 4 basis modules
- **Smart Device Discovery**: Labels en positional indexing met fallback mechanismen
- **Enhanced DeviceRegistry**: `getDeviceByTypeAndLabel()` voor specifieke device selectie
- **Nieuwe Modules**: DAC, Display, Config modules toegevoegd

### Data Persistentie Systeem
- **Multi-level Storage**: 4-laags opslag architectuur
  - **Primary**: SD Card voor hoofdconfiguratie
  - **Fallback**: SPIFFS als backup storage
  - **Runtime**: EEPROM met checksum validatie
  - **Emergency**: Default waarden als laatste redmiddel

### Connectiviteit Verbeteringen
- **MQTT Throttling**: Intelligente 30-seconden throttling tegen spam
- **JSON Validatie**: ArduinoJson v7.2.1 met input validatie
- **Library Management**: Vastgezette versies voor stabiliteit

### Error Handling Systeem
- **Graceful Degradation**: Systeem blijft operationeel bij hardware falen
- **Network Recovery**: Retry mechanisme met exponential backoff
- **Data Integrity**: Checksum validatie voor corruptie detectie
- **Config Protection**: Fallback naar defaults bij configuratie fouten
- **Sensor Redundancy**: Continueren met werkende sensoren bij uitval

### Development Environment
- **Advanced PlatformIO**: Geoptimaliseerde build configuratie
- **Custom Build Flags**: Performance optimalisaties
- **Library Pinning**: Vastgezette versies (PID, ArduinoJson)
- **Enhanced Debugging**: Maximum debug output voor ontwikkeling

---

## Conclusie

### Transformatie
- **Van**: Passief monitoring systeem
- **Naar**: Actief klimaatregelsysteem

### Key Achievements
1. **Nieuwe functionaliteit**: Volledige klimaatregeling
2. **Professional architectuur**: SOLID principles + design patterns
3. **Enterprise features**: Multi-level config, error recovery, persistentie
4. **User experience**: Display + interactive controls
5. **Development quality**: Advanced debugging + structured codebase

### Impact
**Showcase Climate Controller** demonstreert een professionele evolutie van prototype naar production-ready embedded systeem, geschikt voor commerciële toepassing.

---

*Ron Groenen - Fontys Semester 4 - Junior Embedded Software Engineer*
