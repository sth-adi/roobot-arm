#pragma once
// HTTPClient on real sockets (plain HTTP/1.1, blocking, 5 s timeouts like the ESP32 library), so the firmware's requests
// really cross the network and really block its loop() while they wait. https:// is accepted and treated as http://:
// the cost of a TLS handshake is added by the network proxy, not computed here.
#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <map>
#include <vector>

#define HTTP_CODE_OK 200
#define HTTP_CODE_NO_CONTENT 204
#define HTTPC_ERROR_CONNECTION_REFUSED (-1)
#define HTTPC_ERROR_SEND_PAYLOAD_FAILED (-3)
#define HTTPC_ERROR_NOT_CONNECTED (-4)
#define HTTPC_ERROR_CONNECTION_LOST (-5)
#define HTTPC_ERROR_READ_TIMEOUT (-11)

// Timing the emulator reports: how long each kind of request blocked the firmware.
struct EmuNetStats {
    unsigned polls = 0, pollFails = 0, connects = 0, others = 0, otherFails = 0;
    double pollMsSum = 0, pollMsMax = 0, otherMsMax = 0, connectMsMax = 0;
    std::vector<float> pollMs;       // every /controller GET, for percentiles
};
inline EmuNetStats& emuNet() { static EmuNetStats s; return s; }

class HTTPClient {
    std::string host_, path_;
    int port_ = 80, fd_ = -1;
    bool reuse_ = false;
    std::map<std::string, std::string> reqHeaders_, respHeaders_;
    std::vector<std::string> collect_;
    std::string body_;

    static double nowMs() { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

    void closeSocket() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }

    int connectNow()
    {
        struct addrinfo hints = {}, *res = nullptr;
        hints.ai_family = AF_INET; hints.ai_socktype = SOCK_STREAM;
        if (getaddrinfo(host_.c_str(), std::to_string(port_).c_str(), &hints, &res) != 0 || !res) return HTTPC_ERROR_CONNECTION_REFUSED;
        int fd = ::socket(res->ai_family, res->ai_socktype, 0);
        struct timeval tv = {5, 0};                     // the ESP32 HTTPClient default timeout is 5 s
        setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
        int one = 1; setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        const double t0 = nowMs();
        const int rc = ::connect(fd, res->ai_addr, res->ai_addrlen);
        freeaddrinfo(res);
        if (rc != 0) { ::close(fd); return errno == EINPROGRESS || errno == EAGAIN ? HTTPC_ERROR_READ_TIMEOUT : HTTPC_ERROR_CONNECTION_REFUSED; }
        EmuNetStats& st = emuNet();
        ++st.connects; st.connectMsMax = std::max(st.connectMsMax, nowMs() - t0);
        fd_ = fd;
        return 0;
    }

    bool sendAll(const std::string& s)
    {
        size_t off = 0;
        while (off < s.size()) {
            const ssize_t n = ::send(fd_, s.data() + off, s.size() - off, MSG_NOSIGNAL);
            if (n <= 0) return false;
            off += (size_t)n;
        }
        return true;
    }

    // Read the status line, headers and body. Returns the status code or a negative error.
    int readResponse(bool& serverCloses)
    {
        std::string buf;
        char tmp[2048];
        size_t headEnd = std::string::npos;
        while ((headEnd = buf.find("\r\n\r\n")) == std::string::npos) {
            const ssize_t n = ::recv(fd_, tmp, sizeof tmp, 0);
            if (n == 0) return HTTPC_ERROR_CONNECTION_LOST;
            if (n < 0) return (errno == EAGAIN || errno == EWOULDBLOCK) ? HTTPC_ERROR_READ_TIMEOUT : HTTPC_ERROR_CONNECTION_LOST;
            buf.append(tmp, (size_t)n);
        }
        const std::string head = buf.substr(0, headEnd);
        std::string rest = buf.substr(headEnd + 4);
        int status = 0;
        sscanf(head.c_str(), "HTTP/%*s %d", &status);
        respHeaders_.clear();
        size_t pos = head.find("\r\n");
        long contentLength = -1;
        serverCloses = head.compare(0, 8, "HTTP/1.0") == 0;
        while (pos != std::string::npos) {
            const size_t next = head.find("\r\n", pos + 2);
            const std::string line = head.substr(pos + 2, next == std::string::npos ? std::string::npos : next - pos - 2);
            pos = next;
            const size_t c = line.find(':');
            if (c == std::string::npos) continue;
            std::string k = line.substr(0, c), v = line.substr(c + 1);
            while (!v.empty() && v[0] == ' ') v.erase(0, 1);
            for (auto& ch : k) ch = (char)tolower(ch);
            respHeaders_[k] = v;
            if (k == "content-length") contentLength = atol(v.c_str());
            if (k == "connection") {
                std::string lv = v; for (auto& ch : lv) ch = (char)tolower(ch);
                if (lv.find("close") != std::string::npos) serverCloses = true;
                if (lv.find("keep-alive") != std::string::npos) serverCloses = false;
            }
        }
        body_ = rest;
        if (status == 204 || status == 304) contentLength = 0;
        while (contentLength >= 0 ? (long)body_.size() < contentLength : true) {
            const ssize_t n = ::recv(fd_, tmp, sizeof tmp, 0);
            if (n == 0) { serverCloses = true; break; }
            if (n < 0) return HTTPC_ERROR_READ_TIMEOUT;
            body_.append(tmp, (size_t)n);
        }
        return status;
    }

    int request(const char* method, const std::string& payload)
    {
        const double t0 = nowMs();
        const bool isPoll = path_ == "/controller" && std::string(method) == "GET";
        int status = HTTPC_ERROR_NOT_CONNECTED;
        for (int attempt = 0; attempt < 2; ++attempt) {
            const bool reusedSocket = fd_ >= 0;
            if (fd_ < 0) {
                status = connectNow();
                if (status != 0) break;
            }
            std::string req = std::string(method) + " " + path_ + " HTTP/1.1\r\nHost: " + host_ + ":" + std::to_string(port_) +
                              "\r\nUser-Agent: ESP32HTTPClient\r\nConnection: " + (reuse_ ? "keep-alive" : "close") + "\r\n";
            for (auto& h : reqHeaders_) req += h.first + ": " + h.second + "\r\n";
            if (std::string(method) == "POST") req += "Content-Length: " + std::to_string(payload.size()) + "\r\n";
            req += "\r\n" + payload;
            bool serverCloses = true;
            if (sendAll(req)) status = readResponse(serverCloses);
            else status = HTTPC_ERROR_SEND_PAYLOAD_FAILED;
            if (status > 0) {
                if (serverCloses || !reuse_) closeSocket();
                break;
            }
            closeSocket();
            if (!reusedSocket) break;            // a stale kept-alive socket gets one retry on a fresh connection
        }
        const double ms = nowMs() - t0;
        EmuNetStats& st = emuNet();
        if (isPoll) {
            ++st.polls; if (status <= 0) ++st.pollFails;
            st.pollMsSum += ms; st.pollMsMax = std::max(st.pollMsMax, ms); st.pollMs.push_back((float)ms);
        } else {
            ++st.others; if (status <= 0) ++st.otherFails;
            st.otherMsMax = std::max(st.otherMsMax, ms);
        }
        return status;
    }

public:
    ~HTTPClient() { closeSocket(); }
    bool begin(WiFiClientSecure&, const String& url)
    {
        std::string u = url.s;
        const size_t scheme = u.find("://");
        if (scheme != std::string::npos) u = u.substr(scheme + 3);
        const size_t slash = u.find('/');
        std::string hostport = slash == std::string::npos ? u : u.substr(0, slash);
        const std::string newPath = slash == std::string::npos ? "/" : u.substr(slash);
        std::string h = hostport; int p = 80;
        const size_t colon = hostport.rfind(':');
        if (colon != std::string::npos) { h = hostport.substr(0, colon); p = atoi(hostport.c_str() + colon + 1); }
        if (h != host_ || p != port_) closeSocket();       // a different server: the kept-alive socket is no use
        host_ = h; port_ = p; path_ = newPath;
        return true;
    }
    void setReuse(bool r) { reuse_ = r; }
    void collectHeaders(const char** keys, int n) { collect_.clear(); for (int i = 0; i < n; ++i) collect_.push_back(keys[i]); }
    int GET() { return request("GET", ""); }
    int POST(const String& payload) { return request("POST", payload.s); }
    String header(const char* name) { std::string k = name; for (auto& c : k) c = (char)tolower(c); auto it = respHeaders_.find(k); return String(it == respHeaders_.end() ? "" : it->second); }
    String getString() { return String(body_); }
    void addHeader(const char* k, const char* v) { reqHeaders_[k] = v; }
    void end() { closeSocket(); reqHeaders_.clear(); }
    static String errorToString(int e)
    {
        switch (e) {
            case HTTPC_ERROR_CONNECTION_REFUSED: return "connection refused";
            case HTTPC_ERROR_SEND_PAYLOAD_FAILED: return "send payload failed";
            case HTTPC_ERROR_NOT_CONNECTED: return "not connected";
            case HTTPC_ERROR_CONNECTION_LOST: return "connection lost";
            case HTTPC_ERROR_READ_TIMEOUT: return "read timeout";
            default: return "error " + std::to_string(e);
        }
    }
};
