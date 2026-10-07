#pragma once
// Deduplicación pura de STATUSTEXT para poder probarla sin Arduino/FC.
#include <stdint.h>
#include <string.h>

class StatusTextDeduper
{
public:
  bool shouldSend(const char *wireText, uint8_t severity)
  {
    if (wireText == nullptr)
      return false;
    if (hasLast_ && severity == lastSeverity_ && strcmp(wireText, lastText_) == 0)
      return false;
    strncpy(lastText_, wireText, sizeof(lastText_) - 1);
    lastText_[sizeof(lastText_) - 1] = '\0';
    lastSeverity_ = severity;
    hasLast_ = true;
    return true;
  }

private:
  char lastText_[50] = {};
  uint8_t lastSeverity_ = 0xff;
  bool hasLast_ = false;
};
