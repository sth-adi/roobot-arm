#pragma once
#include <Arduino.h>
#include <WiFiClientSecure.h>
#define HTTP_CODE_OK 200
#define HTTP_CODE_NO_CONTENT 204
// The test replaces the controller packet through fakeControllerBody().
inline std::string& fakeControllerBody() { static std::string b; return b; }
class HTTPClient {
public:
    bool begin(WiFiClientSecure&, const String&) { return true; }
    void setReuse(bool) {}
    void collectHeaders(const char**, int) {}
    int GET() { return 200; }
    int POST(const String&) { return 204; }
    String header(const char*) { return "Normal"; }
    String getString() { return String(fakeControllerBody()); }
    void addHeader(const char*, const char*) {}
    void end() {}
    static String errorToString(int) { return "error"; }
};
