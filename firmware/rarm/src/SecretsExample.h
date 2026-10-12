// Copy to Secrets.h (git-ignored) and fill in.
const char* WIFI_SSID = "";
const char* WIFI_PASSWORD = "";
// Server host name only, no https:// and no path. The firmware talks HTTPS (port 443), so this must be a server
// behind TLS, e.g. the roobot-arm server deployed with its Dockerfile.
#define HOST "your-server.example.com"
