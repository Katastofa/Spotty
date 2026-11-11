#pragma once
struct MDNSResponder { bool begin(const char *) { return true; } void end() {} void addService(const char *, const char *, int) {} };
extern MDNSResponder MDNS;
