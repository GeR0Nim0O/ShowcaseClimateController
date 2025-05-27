function Decoder(topic, payload) {
    try {
        // Transform incoming payload to JSON
        payload = JSON.parse(payload);
        
        // Extract value from your payload
        var temperature = payload.value;
        
        // Forward Data to Datacake Device API using topic as device identifier
        return [
            {
                device: topic,
                field: "TEMPERATURE",
                value: temperature
            }
        ];
        
    } catch (error) {
        return [];
    }
}
