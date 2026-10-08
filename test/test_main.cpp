// Pruebas unitarias de los módulos puros de FlyWithMe (sin Arduino ni hardware).
// Ejecutar con: pio test -e native
//
// Se prueba el código real de src/ (cabeceras puras). La lógica de Comm/Telem depende de
// Arduino/SPI/UART y se valida en placa y en SITL (ver docs/VALIDACION_PRE_RELEASE.md).

#include <unity.h>

#include <string.h>

#include "../src/protocol.h"
#include "../src/selftest.h"
#include "../src/status_text.h"
#include "../src/wifi_identity.h"

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------------------------
// protocol.h: trama LoRa v2
// ---------------------------------------------------------------------------------------------

static LoraPacket_t makeBeacon()
{
  LoraPacket_t p = {};
  p.version = PROTOCOL_VERSION;
  p.type = LORA_MSG_BEACON;
  p.netid = 4660;
  p.sysid = 1;
  p.mode = 15;
  p.seq = 7;
  p.lat = 370000000;
  p.lon = -65600000;
  p.alt = 200000;
  p.relative_alt = 150000;
  p.ground_speed = 2200;
  p.hdg = 18000;
  p.flags = LORA_FLAG_POSITION_VALID;
  p.checksum = loraPacketChecksum(p);
  return p;
}

void test_frame_size_is_wire_format(void)
{
  TEST_ASSERT_EQUAL_UINT32(40, sizeof(LoraPacket_t));
  TEST_ASSERT_EQUAL_UINT32(sizeof(LoraPacket_t) - 1, offsetof(LoraPacket_t, checksum));
}

void test_checksum_accepts_valid_frame(void)
{
  LoraPacket_t p = makeBeacon();
  TEST_ASSERT_TRUE(loraChecksumOk(p));
  TEST_ASSERT_TRUE(loraPositionOk(p));
}

void test_checksum_rejects_any_corrupted_covered_byte(void)
{
  LoraPacket_t p = makeBeacon();
  uint8_t wire[sizeof(LoraPacket_t)];
  memcpy(wire, &p, sizeof(p));
  for (size_t i = 0; i < sizeof(wire) - 1; ++i)
  {
    wire[i] ^= 0x01;
    LoraPacket_t corrupted;
    memcpy(&corrupted, wire, sizeof(corrupted));
    TEST_ASSERT_FALSE_MESSAGE(loraChecksumOk(corrupted), "un byte alterado debe invalidar la trama");
    wire[i] ^= 0x01;
  }
}

void test_netid_and_version_filters(void)
{
  LoraPacket_t p = makeBeacon();
  TEST_ASSERT_TRUE(loraNetidOk(p, 4660));
  TEST_ASSERT_FALSE(loraNetidOk(p, 4661));
  TEST_ASSERT_TRUE(loraVersionOk(p));
  p.version = PROTOCOL_VERSION + 1;
  TEST_ASSERT_FALSE(loraVersionOk(p));
}

void test_sequence_is_modular_and_rejects_duplicates(void)
{
  TEST_ASSERT_TRUE(loraSeqIsNewer(11, 10));
  TEST_ASSERT_TRUE(loraSeqIsNewer(0, 65535)); // wrap 65535 -> 0
  TEST_ASSERT_FALSE(loraSeqIsNewer(10, 10));  // duplicado
  TEST_ASSERT_FALSE(loraSeqIsNewer(9, 10));   // atrasado
}

void test_sync_word_depends_on_netid(void)
{
  TEST_ASSERT_EQUAL_UINT8(loraSyncWordFor(0x1234), loraSyncWordFor(0x1234));
  TEST_ASSERT_NOT_EQUAL(loraSyncWordFor(0x1234), loraSyncWordFor(0x1235));
}

// ---------------------------------------------------------------------------------------------
// status_text.h: deduplicación de STATUSTEXT
// ---------------------------------------------------------------------------------------------

void test_statustext_dedupe_exact_repeats(void)
{
  StatusTextDeduper d;
  TEST_ASSERT_TRUE(d.shouldSend("FWM: Follower 48m", 4));
  TEST_ASSERT_FALSE(d.shouldSend("FWM: Follower 48m", 4));
  TEST_ASSERT_TRUE(d.shouldSend("FWM: Follower 49m", 4));
  TEST_ASSERT_TRUE(d.shouldSend("FWM: Follower 49m", 6)); // cambio de severidad
  TEST_ASSERT_FALSE(d.shouldSend(nullptr, 6));
}

// ---------------------------------------------------------------------------------------------
// wifi_identity.h: SSID desde la MAC y clave WPA2
// ---------------------------------------------------------------------------------------------

void test_wifi_identity_from_mac(void)
{
  const uint8_t mac[6] = {0x30, 0xAE, 0xA4, 0x07, 0x0D, 0x64};
  char macText[18] = {};
  char ssid[11] = {};
  TEST_ASSERT_TRUE(fwmWifiIdentityFromMac(mac, macText, sizeof(macText), ssid, sizeof(ssid)));
  TEST_ASSERT_EQUAL_STRING("30:AE:A4:07:0D:64", macText);
  TEST_ASSERT_EQUAL_STRING("FWM 070D64", ssid);
}

void test_wifi_identity_rejects_bad_buffers(void)
{
  const uint8_t mac[6] = {0};
  char macText[18] = {};
  char ssid[11] = {};
  TEST_ASSERT_FALSE(fwmWifiIdentityFromMac(nullptr, macText, sizeof(macText), ssid, sizeof(ssid)));
  TEST_ASSERT_FALSE(fwmWifiIdentityFromMac(mac, macText, 10, ssid, sizeof(ssid)));
  TEST_ASSERT_FALSE(fwmWifiIdentityFromMac(mac, macText, sizeof(macText), ssid, 5));
}

void test_passphrase_length_limits(void)
{
  TEST_ASSERT_TRUE(fwmApPassphraseValid("12345678"));
  TEST_ASSERT_FALSE(fwmApPassphraseValid("1234567"));
  TEST_ASSERT_FALSE(fwmApPassphraseValid(""));
  TEST_ASSERT_FALSE(fwmApPassphraseValid(nullptr));

  char max[FWM_AP_PASS_MAX_LEN + 1];
  memset(max, 'a', FWM_AP_PASS_MAX_LEN);
  max[FWM_AP_PASS_MAX_LEN] = '\0';
  TEST_ASSERT_TRUE(fwmApPassphraseValid(max));

  char tooLong[FWM_AP_PASS_MAX_LEN + 2];
  memset(tooLong, 'a', FWM_AP_PASS_MAX_LEN + 1);
  tooLong[FWM_AP_PASS_MAX_LEN + 1] = '\0';
  TEST_ASSERT_FALSE(fwmApPassphraseValid(tooLong));
}

void test_passphrase_rejects_non_printable_ascii(void)
{
  TEST_ASSERT_FALSE(fwmApPassphraseValid("12345678\x01"));
  TEST_ASSERT_FALSE(fwmApPassphraseValid("1234567\x7F"));
  TEST_ASSERT_FALSE(fwmApPassphraseValid("12345678\xC3\xB1")); // ñ en UTF-8
  TEST_ASSERT_TRUE(fwmApPassphraseValid("clave segura !#$"));
}

// ---------------------------------------------------------------------------------------------
// selftest.h: el propio auto-test del protocolo debe pasar en host
// ---------------------------------------------------------------------------------------------

void test_protocol_selftest_passes(void)
{
  char summary[96] = {};
  TEST_ASSERT_EQUAL_INT(0, protocolSelfTest(summary, sizeof(summary)));
  TEST_ASSERT_NOT_NULL(strstr(summary, "0 fails"));
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_frame_size_is_wire_format);
  RUN_TEST(test_checksum_accepts_valid_frame);
  RUN_TEST(test_checksum_rejects_any_corrupted_covered_byte);
  RUN_TEST(test_netid_and_version_filters);
  RUN_TEST(test_sequence_is_modular_and_rejects_duplicates);
  RUN_TEST(test_sync_word_depends_on_netid);
  RUN_TEST(test_statustext_dedupe_exact_repeats);
  RUN_TEST(test_wifi_identity_from_mac);
  RUN_TEST(test_wifi_identity_rejects_bad_buffers);
  RUN_TEST(test_passphrase_length_limits);
  RUN_TEST(test_passphrase_rejects_non_printable_ascii);
  RUN_TEST(test_protocol_selftest_passes);
  return UNITY_END();
}
