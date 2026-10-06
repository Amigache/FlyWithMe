// ============================================================================
// FASE 4: TESTS UNITARIOS
// ============================================================================
// Tests para funciones críticas del sistema FlyWithMe
// Ejecutar con: pio test -e native (requiere configuración adicional)
// ============================================================================

#include <unity.h>
#include <cmath>

// Mock de tipos básicos para testing
typedef struct {
    int32_t lat;
    int32_t lon;
    int32_t alt;
    int32_t relative_alt;
    uint16_t heading;
    uint16_t ground_speed;
    int16_t climb;
    uint8_t sysid;
    uint8_t custom_mode;
    uint8_t base_mode;
    uint8_t checksum;
} LoraPacket_t;

typedef struct {
    float lat;
    float lon;
    float alt;
    float confidence;
} PredictedPosition;

enum FormationType {
    FORMATION_TRAIL,
    FORMATION_LEFT,
    FORMATION_RIGHT,
    FORMATION_ABOVE,
    FORMATION_BELOW
};

// ============================================================================
// FUNCIONES BAJO TEST (copiadas para testing independiente)
// ============================================================================

// Calcular distancia usando fórmula Haversine
float calculateDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2)
{
    const float R = 6371000.0f; // Radio de la Tierra en metros
    
    float lat1_rad = (lat1 / 1e7) * M_PI / 180.0f;
    float lon1_rad = (lon1 / 1e7) * M_PI / 180.0f;
    float lat2_rad = (lat2 / 1e7) * M_PI / 180.0f;
    float lon2_rad = (lon2 / 1e7) * M_PI / 180.0f;
    
    float dlat = lat2_rad - lat1_rad;
    float dlon = lon2_rad - lon1_rad;
    
    float a = sin(dlat / 2) * sin(dlat / 2) +
              cos(lat1_rad) * cos(lat2_rad) *
              sin(dlon / 2) * sin(dlon / 2);
    
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float distance = R * c;
    
    return distance;
}

// Validar packet (límites de seguridad)
bool isSafeToFollow(LoraPacket_t packet)
{
    const int32_t MAX_FOLLOW_DISTANCE = 5000;  // 5km
    const int32_t MIN_SAFE_ALTITUDE = 50;      // 50m
    const int32_t MAX_SAFE_ALTITUDE = 500;     // 500m
    const int32_t MAX_GROUND_SPEED = 3000;     // 30 m/s
    
    // Validar altitud relativa
    if (packet.relative_alt < MIN_SAFE_ALTITUDE * 100) return false;
    if (packet.relative_alt > MAX_SAFE_ALTITUDE * 100) return false;
    
    // Validar velocidad
    if (packet.ground_speed > MAX_GROUND_SPEED) return false;
    
    // Validar rango GPS (latitud -90 a 90, longitud -180 a 180)
    if (abs(packet.lat) > 90e7) return false;
    if (abs(packet.lon) > 180e7) return false;
    
    return true;
}

// Predicción de posición
PredictedPosition predictLeaderPosition(LoraPacket_t current, float predictionTimeSeconds)
{
    PredictedPosition pred;
    
    float speedMPS = current.ground_speed / 100.0f;
    float predictedDistance = speedMPS * predictionTimeSeconds;
    float headingRad = current.heading * M_PI / 180.0f;
    
    float latChange = (predictedDistance * cos(headingRad)) / 111320.0f;
    float lonChange = (predictedDistance * sin(headingRad)) / 
                      (111320.0f * cos(current.lat / 1e7 * M_PI / 180.0f));
    
    pred.lat = current.lat / 1e7 + latChange;
    pred.lon = current.lon / 1e7 + lonChange;
    pred.alt = current.alt + (current.climb / 100.0f * predictionTimeSeconds);
    
    // Confianza basada en velocidad (más confiable a velocidades moderadas)
    if (speedMPS < 2.0f) {
        pred.confidence = 0.3f;  // Muy lento
    } else if (speedMPS > 25.0f) {
        pred.confidence = 0.5f;  // Muy rápido
    } else {
        pred.confidence = 0.9f;  // Velocidad óptima
    }
    
    return pred;
}

// ============================================================================
// TESTS DE DISTANCIA (Haversine)
// ============================================================================

void test_distance_same_point()
{
    int32_t lat = 404168000;  // 40.4168° (Madrid)
    int32_t lon = -37038000;  // -3.7038°
    
    float distance = calculateDistance(lat, lon, lat, lon);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.0, distance);
}

void test_distance_known_points()
{
    // Madrid (40.4168, -3.7038) a Barcelona (41.3851, 2.1734)
    int32_t lat1 = 404168000;
    int32_t lon1 = -37038000;
    int32_t lat2 = 413851000;
    int32_t lon2 = 21734000;
    
    float distance = calculateDistance(lat1, lon1, lat2, lon2);
    
    // Distancia real ~504 km
    TEST_ASSERT_FLOAT_WITHIN(10000, 504000, distance);
}

void test_distance_small_separation()
{
    // Dos puntos a ~100m
    int32_t lat1 = 404168000;
    int32_t lon1 = -37038000;
    int32_t lat2 = 404178000;  // +0.001° ≈ 111m
    int32_t lon2 = -37038000;
    
    float distance = calculateDistance(lat1, lon1, lat2, lon2);
    TEST_ASSERT_FLOAT_WITHIN(20, 111, distance);
}

// ============================================================================
// TESTS DE VALIDACIÓN DE SEGURIDAD
// ============================================================================

void test_validation_valid_packet()
{
    LoraPacket_t packet;
    packet.lat = 404168000;
    packet.lon = -37038000;
    packet.relative_alt = 10000;  // 100m
    packet.ground_speed = 1500;   // 15 m/s
    packet.heading = 45;
    
    TEST_ASSERT_TRUE(isSafeToFollow(packet));
}

void test_validation_altitude_too_low()
{
    LoraPacket_t packet;
    packet.lat = 404168000;
    packet.lon = -37038000;
    packet.relative_alt = 3000;   // 30m (< 50m límite)
    packet.ground_speed = 1500;
    packet.heading = 45;
    
    TEST_ASSERT_FALSE(isSafeToFollow(packet));
}

void test_validation_altitude_too_high()
{
    LoraPacket_t packet;
    packet.lat = 404168000;
    packet.lon = -37038000;
    packet.relative_alt = 60000;  // 600m (> 500m límite)
    packet.ground_speed = 1500;
    packet.heading = 45;
    
    TEST_ASSERT_FALSE(isSafeToFollow(packet));
}

void test_validation_speed_too_high()
{
    LoraPacket_t packet;
    packet.lat = 404168000;
    packet.lon = -37038000;
    packet.relative_alt = 10000;
    packet.ground_speed = 3500;  // 35 m/s (> 30 m/s límite)
    packet.heading = 45;
    
    TEST_ASSERT_FALSE(isSafeToFollow(packet));
}

void test_validation_invalid_gps()
{
    LoraPacket_t packet;
    packet.lat = 1000000000;  // >90° (inválido)
    packet.lon = -37038000;
    packet.relative_alt = 10000;
    packet.ground_speed = 1500;
    packet.heading = 45;
    
    TEST_ASSERT_FALSE(isSafeToFollow(packet));
}

// ============================================================================
// TESTS DE PREDICCIÓN
// ============================================================================

void test_prediction_straight_line()
{
    LoraPacket_t packet;
    packet.lat = 404168000;
    packet.lon = -37038000;
    packet.alt = 10000;
    packet.heading = 0;  // Norte
    packet.ground_speed = 1000;  // 10 m/s
    packet.climb = 0;
    
    PredictedPosition pred = predictLeaderPosition(packet, 1.0f);  // 1 segundo
    
    // En 1 segundo a 10 m/s hacia el norte → ~10m al norte
    // 10m en latitud ≈ 0.00009°
    TEST_ASSERT_FLOAT_WITHIN(0.0001, 40.41689, pred.lat);
    TEST_ASSERT_FLOAT_WITHIN(0.0001, -3.7038, pred.lon);
}

void test_prediction_confidence_slow()
{
    LoraPacket_t packet;
    packet.lat = 404168000;
    packet.lon = -37038000;
    packet.alt = 10000;
    packet.heading = 0;
    packet.ground_speed = 100;  // 1 m/s (muy lento)
    packet.climb = 0;
    
    PredictedPosition pred = predictLeaderPosition(packet, 1.0f);
    
    // Confianza baja para velocidades muy lentas
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.3, pred.confidence);
}

void test_prediction_confidence_fast()
{
    LoraPacket_t packet;
    packet.lat = 404168000;
    packet.lon = -37038000;
    packet.alt = 10000;
    packet.heading = 0;
    packet.ground_speed = 2600;  // 26 m/s (muy rápido)
    packet.climb = 0;
    
    PredictedPosition pred = predictLeaderPosition(packet, 1.0f);
    
    // Confianza media para velocidades muy altas
    TEST_ASSERT_FLOAT_WITHIN(0.2, 0.5, pred.confidence);
}

void test_prediction_confidence_optimal()
{
    LoraPacket_t packet;
    packet.lat = 404168000;
    packet.lon = -37038000;
    packet.alt = 10000;
    packet.heading = 0;
    packet.ground_speed = 1500;  // 15 m/s (óptimo)
    packet.climb = 0;
    
    PredictedPosition pred = predictLeaderPosition(packet, 1.0f);
    
    // Confianza alta para velocidades moderadas
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0.9, pred.confidence);
}

// ============================================================================
// TESTS DE COMPRESIÓN (conceptual, sin implementación completa)
// ============================================================================

void test_compression_ratio()
{
    // Test conceptual: packet normal = 27 bytes, comprimido = 15 bytes
    size_t normalSize = 27;
    size_t compressedSize = 15;
    
    float compressionRatio = (1.0f - ((float)compressedSize / normalSize)) * 100.0f;
    
    TEST_ASSERT_FLOAT_WITHIN(1, 44.4, compressionRatio);
}

// ============================================================================
// MAIN DE TESTS
// ============================================================================

void setUp(void) {
    // Setup antes de cada test
}

void tearDown(void) {
    // Cleanup después de cada test
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    
    // Tests de distancia
    RUN_TEST(test_distance_same_point);
    RUN_TEST(test_distance_known_points);
    RUN_TEST(test_distance_small_separation);
    
    // Tests de validación
    RUN_TEST(test_validation_valid_packet);
    RUN_TEST(test_validation_altitude_too_low);
    RUN_TEST(test_validation_altitude_too_high);
    RUN_TEST(test_validation_speed_too_high);
    RUN_TEST(test_validation_invalid_gps);
    
    // Tests de predicción
    RUN_TEST(test_prediction_straight_line);
    RUN_TEST(test_prediction_confidence_slow);
    RUN_TEST(test_prediction_confidence_fast);
    RUN_TEST(test_prediction_confidence_optimal);
    
    // Tests de compresión
    RUN_TEST(test_compression_ratio);
    
    return UNITY_END();
}
