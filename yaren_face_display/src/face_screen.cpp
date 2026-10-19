#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>
#include <map>
#include <cstdlib>
#include <atomic>
#include <sstream>
#include <ctime>
#include <array>
#include <functional>
#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>
#include <unordered_map>
#include <queue>

namespace fs = std::filesystem;

#include <cstdint>
// =============================================================================
//  txu: texto con acentos y ñ para las fuentes Hershey de OpenCV (solo ASCII)
//  Dibuja la letra base y le pinta encima la tilde / acento / dieresis.
// =============================================================================
namespace txu {
struct Mark { size_t pos; int kind; bool upper; };   // kind: 1 acento, 2 dieresis, 3 tilde (ñ)
struct Map  { uint32_t cp; char base; int kind; bool upper; };
static const Map kMap[] = {
    {0xE1,'a',1,false},{0xE9,'e',1,false},{0xED,'i',1,false},{0xF3,'o',1,false},{0xFA,'u',1,false},
    {0xC1,'A',1,true}, {0xC9,'E',1,true}, {0xCD,'I',1,true}, {0xD3,'O',1,true}, {0xDA,'U',1,true},
    {0xF1,'n',3,false},{0xD1,'N',3,true}, {0xFC,'u',2,false},{0xDC,'U',2,true},
    {0xBF,'?',0,false},{0xA1,'!',0,false},{0x2014,'-',0,false},{0x2013,'-',0,false},{0x2026,'.',0,false}
};
inline bool hasNonAscii(const std::string& s) {
    for (unsigned char c : s) if (c >= 0x80) return true;
    return false;
}
inline std::string fold(const std::string& s, std::vector<Mark>* marks) {
    std::string out;
    for (size_t i = 0; i < s.size();) {
        unsigned char c = (unsigned char)s[i];
        if (c < 0x80) { out.push_back((char)c); ++i; continue; }
        uint32_t cp = 0x1FFFFF; size_t len = 1;
        if ((c & 0xE0) == 0xC0 && i + 1 < s.size()) {
            cp = ((c & 0x1Fu) << 6) | ((unsigned char)s[i + 1] & 0x3Fu); len = 2;
        } else if ((c & 0xF0) == 0xE0 && i + 2 < s.size()) {
            cp = ((c & 0x0Fu) << 12) | (((unsigned char)s[i + 1] & 0x3Fu) << 6) | ((unsigned char)s[i + 2] & 0x3Fu); len = 3;
        } else if ((c & 0xF8) == 0xF0 && i + 3 < s.size()) { len = 4; }
        i += len;
        char base = '?'; int kind = 0; bool up = false;
        for (const auto& m : kMap) if (m.cp == cp) { base = m.base; kind = m.kind; up = m.upper; break; }
        if (marks && kind) marks->push_back({out.size(), kind, up});
        out.push_back(base);
    }
    return out;
}
inline cv::Size size(const std::string& text, int font, double scale, int thickness, int* baseLine) {
    if (!hasNonAscii(text)) return cv::getTextSize(text, font, scale, thickness, baseLine);
    return cv::getTextSize(fold(text, nullptr), font, scale, thickness, baseLine);
}
inline void put(cv::InputOutputArray img, const std::string& text, cv::Point org, int font, double scale,
                const cv::Scalar& color, int thickness = 1, int lineType = cv::LINE_8, bool bl = false) {
    if (!hasNonAscii(text)) { cv::putText(img, text, org, font, scale, color, thickness, lineType, bl); return; }
    std::vector<Mark> marks;
    std::string f = fold(text, &marks);
    cv::putText(img, f, org, font, scale, color, thickness, lineType, bl);
    int b0 = 0;
    const int capH = cv::getTextSize("A", font, scale, thickness, &b0).height;
    const int xH   = cv::getTextSize("x", font, scale, thickness, &b0).height;
    const int unit = std::max(2, (int)std::lround(capH * 0.20));
    const int t = std::max(1, thickness);
    for (const auto& m : marks) {
        int px = cv::getTextSize(f.substr(0, m.pos), font, scale, thickness, &b0).width;
        char ch = f[m.pos];
        int w = cv::getTextSize(std::string(1, ch), font, scale, thickness, &b0).width;
        int cx = org.x + px + w / 2;
        int top = org.y - ((m.upper || ch == 'i') ? capH : xH);
        if (m.kind == 1) {            // acento
            int y = top - std::max(2, unit / 2);
            cv::line(img, cv::Point(cx - unit / 2, y), cv::Point(cx + unit, y - (int)(unit * 1.3)), color, t, lineType);
        } else if (m.kind == 2) {     // dieresis
            int dx = std::max(unit, w / 5), y = top - unit;
            cv::circle(img, cv::Point(cx - dx, y), std::max(1, t), color, cv::FILLED, lineType);
            cv::circle(img, cv::Point(cx + dx, y), std::max(1, t), color, cv::FILLED, lineType);
        } else if (m.kind == 3) {     // tilde de la ñ
            int half = std::max(unit * 3 / 2, w * 3 / 10), y0 = top - unit - 1;
            std::vector<cv::Point> pts;
            for (int i = 0; i <= 12; ++i) {
                double u = i / 12.0;
                pts.push_back(cv::Point(cx - half + (int)std::lround(2.0 * half * u),
                                        y0 - (int)std::lround(unit * 0.55 * std::sin(u * 2.0 * CV_PI))));
            }
            cv::polylines(img, pts, false, color, t, lineType);
        }
    }
}
} // namespace txu


struct AlsaDevice {
    std::string hwId;
    std::string label;
};

static std::vector<AlsaDevice> parsePulseDevices(bool isSink) {
    std::vector<AlsaDevice> result;
    std::string cmd = isSink
        ? "pactl list sinks 2>/dev/null"
        : "pactl list sources 2>/dev/null";
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return result;
    char buf[512];
    std::string curName, curDesc;
    auto flush = [&]() {
        if (curName.empty() || curDesc.empty()) return;
        std::string low     = curDesc;
        std::string lowName = curName;
        std::transform(low.begin(),     low.end(),     low.begin(),     ::tolower);
        std::transform(lowName.begin(), lowName.end(), lowName.begin(), ::tolower);
        bool isMonitor = low.find(".monitor")     != std::string::npos ||
                         lowName.find(".monitor")  != std::string::npos;
        bool isVirtual = low.find("null")  != std::string::npos ||
                         low.find("dummy") != std::string::npos;
        bool incluir = false;
        if (isSink) {
            bool isHDMI      = low.find("hdmi")       != std::string::npos ||
                               low.find("displayport") != std::string::npos;
            bool isSPDIF     = low.find("s/pdif")      != std::string::npos ||
                               low.find("spdif")       != std::string::npos ||
                               low.find("digital")     != std::string::npos;
            bool isAnalog    = low.find("analog")      != std::string::npos;
            bool isHeadphone = low.find("headphone")   != std::string::npos;
            incluir = (isHDMI || isSPDIF || isAnalog || isHeadphone) && !isVirtual && !isMonitor;
        } else {
            bool isNvidia = low.find("nvidia") != std::string::npos;
            incluir = !isNvidia && !isVirtual && !isMonitor;
        }
        if (incluir) {
            AlsaDevice d;
            d.hwId  = curName;
            d.label = curDesc;
            result.push_back(d);
        }
        curName.clear(); curDesc.clear();
    };
    while (fgets(buf, sizeof(buf), pipe)) {
        std::string line(buf);
        while (!line.empty() && (line.back()=='\n'||line.back()=='\r')) line.pop_back();
        size_t s = line.find_first_not_of(" \t");
        std::string trimmed = (s != std::string::npos) ? line.substr(s) : "";
        if (trimmed.rfind("Name:", 0) == 0) {
            flush();
            curName = trimmed.substr(5);
            s = curName.find_first_not_of(" \t");
            if (s != std::string::npos) curName = curName.substr(s);
        } else if (trimmed.rfind("Description:", 0) == 0) {
            curDesc = trimmed.substr(12);
            s = curDesc.find_first_not_of(" \t");
            if (s != std::string::npos) curDesc = curDesc.substr(s);
        }
    }
    flush();
    pclose(pipe);
    return result;
}

static std::string getPulseDefault(bool isSink) {
    std::string cmd = isSink
        ? "pactl get-default-sink 2>/dev/null"
        : "pactl get-default-source 2>/dev/null";
    FILE* f = popen(cmd.c_str(), "r");
    if (!f) return "";
    char buf[256] = {};
    fgets(buf, sizeof(buf), f);
    pclose(f);
    std::string r(buf);
    while (!r.empty() && (r.back()=='\n'||r.back()=='\r'||r.back()==' ')) r.pop_back();
    return r;
}
static std::string getLocalIP() {
    FILE* pipe = popen("hostname -I | awk '{print $1}'", "r");
    if (!pipe) return "Unknown IP";
    char buf[128];
    std::string ip = "Offline";
    if (fgets(buf, sizeof(buf), pipe) != nullptr) {
        ip = buf;
        ip.erase(std::remove(ip.begin(), ip.end(), '\n'), ip.end());
        ip.erase(std::remove(ip.begin(), ip.end(), '\r'), ip.end());
        ip.erase(std::remove(ip.begin(), ip.end(), ' '), ip.end());
    }
    pclose(pipe);
    if (ip.empty()) return "Offline";
    return ip;
}

static void drawCenteredText(cv::Mat& frame, const std::string& txt,
                             int totalW, int y,
                             int font, double scale,
                             const cv::Scalar& color, int thick = 1) {
    int bl = 0;
    cv::Size s = txu::size(txt, font, scale, thick, &bl);
    txu::put(frame, txt, {(totalW - s.width) / 2, y},
                font, scale, color, thick, cv::LINE_AA);
}

static void drawTextInRect(cv::Mat& frame, const std::string& txt,
                           const cv::Rect& r,
                           int font, double scale,
                           const cv::Scalar& color, int thick = 1) {
    int bl = 0;
    cv::Size s = txu::size(txt, font, scale, thick, &bl);
    txu::put(frame, txt,
                {r.x + (r.width  - s.width)  / 2,
                 r.y + (r.height + s.height) / 2 - 2},
                font, scale, color, thick, cv::LINE_AA);
}

static void drawArrow(cv::Mat& frame, cv::Point center, bool up,
                      const cv::Scalar& color, int size = 7) {
    std::vector<cv::Point> pts(3);
    if (up) {
        pts[0] = {center.x,        center.y - size};
        pts[1] = {center.x - size, center.y + size};
        pts[2] = {center.x + size, center.y + size};
    } else {
        pts[0] = {center.x,        center.y + size};
        pts[1] = {center.x - size, center.y - size};
        pts[2] = {center.x + size, center.y - size};
    }
    cv::fillPoly(frame, pts, color);
}

// =============================================================================
//  SettingsMenu
// =============================================================================

// =============================================================================
//  kd:: estilo infantil (fondo crema, tarjetas blancas, botones de colores)
// =============================================================================
namespace kd {
inline cv::Scalar cream() { return cv::Scalar(231, 246, 248); }
inline cv::Scalar ink()   { return cv::Scalar(52, 48, 46); }
inline cv::Scalar gray()  { return cv::Scalar(125, 130, 135); }
inline cv::Scalar shade() { return cv::Scalar(205, 218, 222); }
inline cv::Scalar white() { return cv::Scalar(255, 255, 255); }
inline cv::Scalar mix(const cv::Scalar& a, const cv::Scalar& b, double k) { return a * (1.0 - k) + b * k; }

// paleta: coral, naranja, amarillo, verde, azul, morado (BGR)
inline cv::Scalar pal(int i) {
    static const cv::Scalar P[6] = {
        cv::Scalar(80, 90, 250), cv::Scalar(40, 160, 255), cv::Scalar(30, 205, 255),
        cv::Scalar(90, 205, 90), cv::Scalar(250, 170, 60), cv::Scalar(230, 110, 200)};
    return P[((i % 6) + 6) % 6];
}
inline cv::Scalar rainbow(float k) {
    k = std::max(0.f, std::min(1.f, k));
    float pos = k * 5.f;
    int i = (int)pos;
    float f = pos - i;
    return mix(pal(i), pal(std::min(i + 1, 5)), f);
}

inline void roundPath(std::vector<cv::Point>& pts, const cv::Rect& r, int rad) {
    rad = std::max(1, std::min(rad, std::min(r.width, r.height) / 2));
    auto arc = [&](int cx, int cy, int a0) {
        for (int a = a0; a <= a0 + 90; a += 10)
            pts.push_back(cv::Point(cx + (int)std::lround(rad * std::cos(a * CV_PI / 180.0)),
                                    cy + (int)std::lround(rad * std::sin(a * CV_PI / 180.0))));
    };
    arc(r.x + r.width - rad - 1, r.y + rad,                270);
    arc(r.x + r.width - rad - 1, r.y + r.height - rad - 1, 0);
    arc(r.x + rad,               r.y + r.height - rad - 1, 90);
    arc(r.x + rad,               r.y + rad,                180);
}
inline void fillRound(cv::Mat& f, const cv::Rect& r, int rad, const cv::Scalar& c, double alpha = 1.0) {
    cv::Rect rr = r & cv::Rect(0, 0, f.cols, f.rows);
    if (rr.area() <= 0) return;
    std::vector<cv::Point> p; roundPath(p, r, rad);
    if (alpha >= 0.99) { cv::fillPoly(f, p, c, cv::LINE_AA); return; }
    cv::Mat roi = f(rr), ov = roi.clone();
    for (auto& pt : p) { pt.x -= rr.x; pt.y -= rr.y; }
    cv::fillPoly(ov, p, c, cv::LINE_AA);
    cv::addWeighted(ov, alpha, roi, 1.0 - alpha, 0, roi);
}
inline void strokeRound(cv::Mat& f, const cv::Rect& r, int rad, const cv::Scalar& c, int thick = 1) {
    std::vector<cv::Point> p; roundPath(p, r, rad);
    cv::polylines(f, p, true, c, thick, cv::LINE_AA);
}
// tarjeta blanca con sombra y borde de color
inline void card(cv::Mat& f, const cv::Rect& r, int rad, const cv::Scalar& border, int thick = 2) {
    fillRound(f, cv::Rect(r.x, r.y + 4, r.width, r.height), rad, shade());
    fillRound(f, r, rad, white());
    strokeRound(f, r, rad, border, thick);
}
inline void text(cv::Mat& f, const std::string& s, cv::Point org, int font, double sc,
                 const cv::Scalar& col, const cv::Scalar& outline, int th = 1) {
    txu::put(f, s, org, font, sc, outline, th + 3, cv::LINE_AA);
    txu::put(f, s, org, font, sc, col, th, cv::LINE_AA);
}
inline void centered(cv::Mat& f, const std::string& s, int totalW, int y, int font, double sc,
                     const cv::Scalar& col, int th = 1) {
    int bl = 0;
    cv::Size ts = txu::size(s, font, sc, th, &bl);
    txu::put(f, s, cv::Point((totalW - ts.width) / 2, y), font, sc, col, th, cv::LINE_AA);
}
inline void rainbowTitle(cv::Mat& f, const std::string& s, int W, int y, double sc, int th = 2) {
    int bl = 0;
    cv::Size full = txu::size(s, cv::FONT_HERSHEY_DUPLEX, sc, th, &bl);
    int x0 = (W - full.width) / 2;
    for (int pass = 0; pass < 2; ++pass) {
        std::string prefix;
        int ci = 0;
        for (size_t i = 0; i < s.size(); ++i) {
            int pw = prefix.empty() ? 0 : txu::size(prefix, cv::FONT_HERSHEY_DUPLEX, sc, th, &bl).width;
            if (s[i] != ' ') {
                std::string ch(1, s[i]);
                cv::Scalar c = pal(ci++);
                if (pass == 0) txu::put(f, ch, cv::Point(x0 + pw, y), cv::FONT_HERSHEY_DUPLEX, sc, c * 0.5, th + 3, cv::LINE_AA);
                else           txu::put(f, ch, cv::Point(x0 + pw, y), cv::FONT_HERSHEY_DUPLEX, sc, c, th, cv::LINE_AA);
            }
            prefix += s[i];
        }
    }
}
inline void sparkle(cv::Mat& f, cv::Point c, int r, const cv::Scalar& col) {
    std::vector<cv::Point> p = {
        {c.x, c.y - r}, {c.x + r / 3, c.y - r / 3}, {c.x + r, c.y}, {c.x + r / 3, c.y + r / 3},
        {c.x, c.y + r}, {c.x - r / 3, c.y + r / 3}, {c.x - r, c.y}, {c.x - r / 3, c.y - r / 3}};
    cv::fillPoly(f, p, col, cv::LINE_AA);
}
inline void background(cv::Mat& f) {
    f.setTo(cream());
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    int W = f.cols, H = f.rows;
    for (int i = 0; i < 10; ++i) {
        double x = std::fmod(i * 83.0 + 20.0 * std::sin(t * 0.5 + i), (double)W);
        if (x < 0) x += W;
        double y = H - std::fmod(i * 47.0 + t * (6.0 + (i % 4) * 2.5), H + 30.0) + 15.0;
        cv::Scalar c = mix(pal(i), white(), 0.55);
        if (i % 3 == 0) sparkle(f, cv::Point((int)x, (int)y), 5 + (i % 2) * 3, c);
        else cv::circle(f, cv::Point((int)x, (int)y), 2 + (i % 3) * 2, c, cv::FILLED, cv::LINE_AA);
    }
}
inline void button(cv::Mat& f, const cv::Rect& r, const std::string& txt, const cv::Scalar& accent,
                   bool hov, bool enabled = true, double maxScale = 0.5) {
    int rad = r.height / 2;
    cv::Scalar a = enabled ? accent : cv::Scalar(185, 192, 196);
    fillRound(f, cv::Rect(r.x, r.y + 4, r.width, r.height), rad, shade());
    fillRound(f, r, rad, (hov && enabled) ? mix(a, white(), 0.22) : a);
    strokeRound(f, r, rad, white(), (hov && enabled) ? 3 : 2);
    double sc = maxScale; int bl = 0;
    while (sc > 0.28 && txu::size(txt, cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl).width > r.width - 20) sc -= 0.03;
    cv::Size ts = txu::size(txt, cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl);
    txu::put(f, txt, cv::Point(r.x + (r.width - ts.width) / 2, r.y + (r.height + ts.height) / 2 - 1),
                cv::FONT_HERSHEY_DUPLEX, sc, white(), 1, cv::LINE_AA);
}
inline void buttonThin(cv::Mat& f, const cv::Rect& r, const std::string& txt, const cv::Scalar& accent,
                       bool hov, bool enabled = true, double maxScale = 0.5) {
    int rad = r.height / 2;
    cv::Scalar a = enabled ? accent : cv::Scalar(185, 192, 196);
    fillRound(f, cv::Rect(r.x, r.y + 4, r.width, r.height), rad, shade());
    fillRound(f, r, rad, (hov && enabled) ? mix(a, white(), 0.22) : a);
    strokeRound(f, r, rad, white(), (hov && enabled) ? 3 : 2);
    double sc = maxScale; int bl = 0;
    while (sc > 0.28 && txu::size(txt, cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl).width > r.width - 20) sc -= 0.03;
    cv::Size ts = txu::size(txt, cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl);
    txu::put(f, txt, cv::Point(r.x + (r.width - ts.width) / 2, r.y + (r.height + ts.height) / 2 - 1),
                cv::FONT_HERSHEY_DUPLEX, sc, white(), 1, cv::LINE_AA);
}
inline void roundArrow(cv::Mat& f, const cv::Rect& r, bool left, const cv::Scalar& accent, bool hov) {
    if (r.area() <= 0) return;
    cv::Point c(r.x + r.width / 2, r.y + r.height / 2);
    int R = std::min(r.width, r.height) / 2;
    cv::circle(f, cv::Point(c.x, c.y + 3), R, shade(), cv::FILLED, cv::LINE_AA);
    cv::circle(f, c, R, hov ? accent : white(), cv::FILLED, cv::LINE_AA);
    cv::circle(f, c, R, accent, 3, cv::LINE_AA);
    int s = left ? -1 : 1;
    std::vector<cv::Point> ch = { {c.x - s * 4, c.y - 10}, {c.x + s * 6, c.y}, {c.x - s * 4, c.y + 10} };
    cv::polylines(f, ch, false, hov ? white() : accent, 4, cv::LINE_AA);
}
inline void wifiGlyph(cv::Mat& f, cv::Point c, const cv::Scalar& col) {
    cv::Point p(c.x, c.y + 6);
    for (int rr : {5, 9, 13}) cv::ellipse(f, p, cv::Size(rr, rr), 0, 225, 315, col, 2, cv::LINE_AA);
    cv::circle(f, p, 2, col, cv::FILLED, cv::LINE_AA);
}
inline void lockGlyph(cv::Mat& f, cv::Point c, const cv::Scalar& col) {
    cv::rectangle(f, cv::Rect(c.x - 5, c.y - 1, 10, 8), col, cv::FILLED, cv::LINE_AA);
    cv::ellipse(f, cv::Point(c.x, c.y - 1), cv::Size(4, 5), 0, 180, 360, col, 2, cv::LINE_AA);
}
inline void bouncingDots(cv::Mat& f, cv::Point c, double t, const cv::Scalar* mono = nullptr) {
    for (int k = 0; k < 5; ++k)
        cv::circle(f, cv::Point(c.x - 40 + k * 20, c.y - (int)(8.0 * std::fabs(std::sin(t * 4.0 + k * 0.6)))),
                   6, mono ? *mono : pal(k), cv::FILLED, cv::LINE_AA);
}
inline double now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
} // namespace kd

class SettingsMenu {
public:
    int volumeLevel = 64;
    int previousVolume = 64;
    bool isMuted = false;
    bool isDraggingVolume = false;
    cv::Rect muteBtnRect {0, 0, 0, 0};
    cv::Rect sliderTrackRect {0, 0, 0, 0};
    bool* isEnglish = nullptr;
    std::function<void()> onLanguageChanged;
    cv::Rect btnLang {0, 0, 0, 0};
    std::function<void(const std::string&)> onMicSelected;
    std::function<void(const std::string&)> onSpkSelected;
    std::string selectedMicId;
    std::function<void()> onWifi;
    std::string selectedSpkId;
    std::string localIP;
    std::function<void()> onBack;
    std::string feedbackMsg;
    std::chrono::steady_clock::time_point feedbackTime;
    cv::Rect powerButtonRect {0, 0, 0, 0};
    bool hoveredPower {false};
    bool eng() const { return isEnglish != nullptr && *isEnglish; }

    SettingsMenu() { refresh(); }

    void setAudioMutex(std::mutex* m) { audioMutex_ = m; }

    void setFeedback(const std::string& msg) {
        feedbackMsg  = msg;
        feedbackTime = std::chrono::steady_clock::now();
    }

    void refresh() {
        mics = parsePulseDevices(false);
        spks = parsePulseDevices(true);
        if (mics.empty()) mics.push_back({"", eng() ? "(no microphones detected)" : "(sin micrófonos detectados)"});
        if (spks.empty()) spks.push_back({"", eng() ? "(no speakers detected)"    : "(sin parlantes detectados)"});
        std::string defSpk = getPulseDefault(true);
        std::string defMic = getPulseDefault(false);
        selSpk = 0;
        for (int i = 0; i < (int)spks.size(); ++i)
            if (spks[i].hwId == defSpk) { selSpk = i; break; }
        selectedSpkId = spks[selSpk].hwId;
        selMic = 0;
        for (int i = 0; i < (int)mics.size(); ++i)
            if (mics[i].hwId == defMic) { selMic = i; break; }
        selectedMicId = mics[selMic].hwId;
        micScroll = std::max(0, std::min(selMic, (int)mics.size() - kRows));
        spkScroll = std::max(0, std::min(selSpk, (int)spks.size() - kRows));
        std::time_t t = std::time(nullptr);
        std::tm* tm   = std::localtime(&t);
        editDay  = tm->tm_mday;
        editMon  = tm->tm_mon + 1;
        editYear = tm->tm_year + 1900;
        editHour = tm->tm_hour;
        editMin  = tm->tm_min;
        initialMicId = selectedMicId;
        initialSpkId = selectedSpkId;
        initDay = editDay; initMon = editMon; editYear_init = editYear;
        initHour = editHour; initMin = editMin;
        localIP = getLocalIP();
    }

    // ------------------------------------------------------------------ render
    void render(cv::Mat& frame) {
        if (frame.empty()) return;
        W = frame.cols;
        H = frame.rows;
        const cv::Scalar white = kd::white();
        frame.setTo(kd::cream());

        // Cabecera
        txu::put(frame, eng() ? "SETTINGS" : "CONFIGURACIÓN", {20, 38},
                    cv::FONT_HERSHEY_DUPLEX, 0.9, orangeDk(), 1, cv::LINE_AA);
        if (!feedbackMsg.empty()) {
            double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - feedbackTime).count();
            if (el < 3.5) {
                double al = (el < 3.0) ? 1.0 : (3.5 - el) / 0.5;
                int bl = 0;
                cv::Size ts = txu::size(feedbackMsg, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &bl);
                txu::put(frame, feedbackMsg, {W - 20 - ts.width, 36}, cv::FONT_HERSHEY_DUPLEX, 0.6,
                            kd::mix(kd::cream(), orangeDk(), al), 1, cv::LINE_AA);
            } else {
                feedbackMsg.clear();
            }
        }

        const std::string labels[5] = {
            eng() ? "MICROPHONE" : "MICRÓFONO",
            eng() ? "SPEAKER"    : "PARLANTE",
            eng() ? "VOLUME"     : "VOLUMEN",
            eng() ? "DATE"       : "FECHA",
            "GENERAL"
        };

        // Panel derecho (solo muestra la pestaña elegida)
        cv::Rect P(224, 62, W - 224 - 14, 340);
        kd::card(frame, P, 26, tan(), 2);
        txu::put(frame, labels[tab_], {P.x + 26, P.y + 40}, cv::FONT_HERSHEY_DUPLEX, 0.85, kd::ink(), 1, cv::LINE_AA);
        cv::line(frame, {P.x + 20, P.y + 56}, {P.x + P.width - 20, P.y + 56}, kd::shade(), 2, cv::LINE_AA);

        switch (tab_) {
            case 0: pageDevices(frame, P, true);  break;
            case 1: pageDevices(frame, P, false); break;
            case 2: pageVolume(frame, P);         break;
            case 3: pageDate(frame, P);           break;
            default: pageGeneral(frame, P);       break;
        }

        // Pestañas a la izquierda
        for (int i = 0; i < 5; ++i) {
            cv::Rect hit(14, 62 + i * 70, 196, 60);
            tabRects_[i] = hit;
            bool sel = (i == tab_);
            bool hov = hit.contains(hov_);
            cv::Rect r = hit;
            if (sel) r.width += 22;                       // la pestaña activa sale hacia el panel
            kd::fillRound(frame, cv::Rect(r.x, r.y + 4, r.width, r.height), 24, kd::shade());
            kd::fillRound(frame, r, 24, sel ? orange() : (hov ? kd::mix(orange(), white, 0.88) : white));
            kd::strokeRound(frame, r, 24, sel ? orangeDk() : tan(), 2);
            cv::Point ic(r.x + 34, r.y + r.height / 2);
            cv::circle(frame, ic, 21, sel ? kd::mix(orange(), white, 0.25) : kd::mix(orange(), white, 0.85),
                       cv::FILLED, cv::LINE_AA);
            drawGlyph(frame, i, ic, sel ? white : orange());
            double sc = fitScale(labels[i], cv::FONT_HERSHEY_DUPLEX, 0.62, 0.34, 196 - 76);
            int bl = 0;
            cv::Size ts = txu::size(labels[i], cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl);
            txu::put(frame, labels[i], {r.x + 66, r.y + (r.height + ts.height) / 2},
                        cv::FONT_HERSHEY_DUPLEX, sc, sel ? white : kd::ink(), 1, cv::LINE_AA);
        }

        // Barra inferior
        btnBack = cv::Rect(14, 418, 196, 52);
        kd::button(frame, btnBack, eng() ? "BACK" : "VOLVER", caramel(), btnBack.contains(hov_), true, 0.6);
        btnSaveAll = cv::Rect(W - 14 - 260, 418, 260, 52);
        kd::button(frame, btnSaveAll, eng() ? "SAVE ALL" : "GUARDAR TODO", orange(), btnSaveAll.contains(hov_), true, 0.6);
    }

    // ------------------------------------------------------------------ mouse
    void handleMouse(int event, int x, int y) {
        cv::Point pt{x, y};
        if (event == cv::EVENT_MOUSEMOVE) {
            hov_ = pt;
            if (isDraggingVolume) dragTo(x);
            return;
        }
        if (event == cv::EVENT_LBUTTONUP) {
            if (isDraggingVolume) { isDraggingVolume = false; applySystemVolume(); }
            return;
        }
        if (event != cv::EVENT_LBUTTONDOWN) return;
        hov_ = pt;

        if (btnBack.contains(pt))    { if (onBack) onBack(); return; }
        if (btnSaveAll.contains(pt)) { doSave(); return; }
        for (int i = 0; i < 5; ++i)
            if (tabRects_[i].contains(pt)) { tab_ = i; return; }

        switch (tab_) {
            case 0:
            case 1: {
                bool isMic = (tab_ == 0);
                auto& devs    = isMic ? mics : spks;
                auto& scroll  = isMic ? micScroll : spkScroll;
                auto& selIdx  = isMic ? selMic : selSpk;
                auto& rowsBuf = isMic ? micRows : spkRows;
                cv::Rect& upBtn = isMic ? btnMicUp : btnSpkUp;
                cv::Rect& dnBtn = isMic ? btnMicDown : btnSpkDown;
                if (btnRefresh.contains(pt)) { refresh(); return; }
                if (upBtn.contains(pt)) { if (scroll > 0) scroll--; return; }
                if (dnBtn.contains(pt)) { if (scroll + kRows < (int)devs.size()) scroll++; return; }
                for (int i = 0; i < (int)rowsBuf.size(); ++i) {
                    if (rowsBuf[i].contains(pt)) {
                        selIdx = scroll + i;
                        if (isMic) { selectedMicId = mics[selIdx].hwId; if (onMicSelected) onMicSelected(selectedMicId); }
                        else       { selectedSpkId = spks[selIdx].hwId; if (onSpkSelected) onSpkSelected(selectedSpkId); }
                        return;
                    }
                }
                break;
            }
            case 2: {
                if (muteBtnRect.contains(pt)) {
                    isMuted = !isMuted;
                    if (isMuted) { previousVolume = volumeLevel; volumeLevel = 0; }
                    else         { volumeLevel = (previousVolume == 0) ? 64 : previousVolume; }
                    applyMix();
                    return;
                }
                if (btnVolDown_.contains(pt)) { setVolume(volumeLevel - 8); applySystemVolume(); return; }
                if (btnVolUp_.contains(pt))   { setVolume(volumeLevel + 8); applySystemVolume(); return; }
                cv::Rect hit(sliderTrackRect.x - 24, sliderTrackRect.y - 30, sliderTrackRect.width + 48, sliderTrackRect.height + 60);
                if (hit.contains(pt)) { isDraggingVolume = true; dragTo(x); return; }
                break;
            }
            case 3: {
                if (btnDayDown.contains(pt)) { dayDelta(-1); return; }
                if (btnDayUp.contains(pt))   { dayDelta(+1); return; }
                if (btnMonDown.contains(pt)) { monDelta(-1); return; }
                if (btnMonUp.contains(pt))   { monDelta(+1); return; }
                break;
            }
            default: {
                if (btnLang.contains(pt)) {
                    if (isEnglish != nullptr) {
                        *isEnglish = !(*isEnglish);
                        RCLCPP_INFO(rclcpp::get_logger("SettingsMenu"), "Idioma cambiado a: %s", *isEnglish ? "English" : "Español");
                    }
                    if (onLanguageChanged) onLanguageChanged();
                    return;
                }
                if (btnWifi.contains(pt)) { if (onWifi) onWifi(); return; }
                break;
            }
        }
    }

private:
    static constexpr int kRows = 3;
    int W = 800, H = 480;
    int tab_ = 0;
    cv::Point hov_ {-1, -1};
    std::array<cv::Rect, 5> tabRects_ {};
    std::vector<AlsaDevice> mics, spks;
    int selMic = 0, selSpk = 0;
    int micScroll = 0, spkScroll = 0;
    int editDay = 1, editMon = 1, editYear = 2024;
    int editHour = 0, editMin = 0;
    std::string initialMicId, initialSpkId;
    int initDay = 1, initMon = 1, editYear_init = 2024, initHour = 0, initMin = 0;
    cv::Rect btnMicUp, btnMicDown, btnSpkUp, btnSpkDown;
    std::vector<cv::Rect> micRows, spkRows;
    cv::Rect btnDayUp, btnDayDown, btnMonUp, btnMonDown;
    cv::Rect btnRefresh, btnSaveAll, btnBack, btnWifi;
    cv::Rect btnVolUp_, btnVolDown_;
    std::mutex* audioMutex_ {nullptr};

    // Paleta: un solo acento (naranja) + caramelo para lo secundario
    static cv::Scalar orange()   { return cv::Scalar(40, 160, 255); }
    static cv::Scalar orangeDk() { return cv::Scalar(20, 105, 205); }
    static cv::Scalar caramel()  { return cv::Scalar(95, 140, 190); }
    static cv::Scalar tan()      { return cv::Scalar(150, 190, 225); }

    static double fitScale(const std::string& s, int font, double sc, double mn, int maxW) {
        int bl = 0;
        while (sc > mn && txu::size(s, font, sc, 1, &bl).width > maxW) sc -= 0.03;
        return sc;
    }
    static void putC(cv::Mat& f, const std::string& s, int cx, int y, int font, double sc,
                     const cv::Scalar& col, int th = 1) {
        int bl = 0;
        cv::Size ts = txu::size(s, font, sc, th, &bl);
        txu::put(f, s, {cx - ts.width / 2, y}, font, sc, col, th, cv::LINE_AA);
    }

    // Boton circular grande: kind 0 = menos, 1 = mas, 2 = arriba, 3 = abajo
    void roundBtn(cv::Mat& f, const cv::Rect& r, int kind, bool enabled = true) {
        bool hov = r.contains(hov_) && enabled;
        cv::Point c(r.x + r.width / 2, r.y + r.height / 2);
        int R = std::min(r.width, r.height) / 2;
        cv::Scalar a = enabled ? orange() : cv::Scalar(205, 210, 214);
        cv::circle(f, {c.x, c.y + 4}, R, kd::shade(), cv::FILLED, cv::LINE_AA);
        cv::circle(f, c, R, hov ? a : kd::white(), cv::FILLED, cv::LINE_AA);
        cv::circle(f, c, R, a, 3, cv::LINE_AA);
        cv::Scalar sym = hov ? kd::white() : a;
        int s = R / 3 + 3;
        if (kind == 0 || kind == 1) {
            cv::line(f, {c.x - s, c.y}, {c.x + s, c.y}, sym, 4, cv::LINE_AA);
            if (kind == 1) cv::line(f, {c.x, c.y - s}, {c.x, c.y + s}, sym, 4, cv::LINE_AA);
        } else if (kind == 2) {
            std::vector<cv::Point> ch = {{c.x - s, c.y + s / 2}, {c.x, c.y - s / 2}, {c.x + s, c.y + s / 2}};
            cv::polylines(f, ch, false, sym, 4, cv::LINE_AA);
        } else {
            std::vector<cv::Point> ch = {{c.x - s, c.y - s / 2}, {c.x, c.y + s / 2}, {c.x + s, c.y - s / 2}};
            cv::polylines(f, ch, false, sym, 4, cv::LINE_AA);
        }
    }

    void drawGlyph(cv::Mat& f, int kind, cv::Point c, const cv::Scalar& col) {
        switch (kind) {
            case 0: {   // microfono
                kd::fillRound(f, cv::Rect(c.x - 5, c.y - 11, 10, 16), 5, col);
                cv::ellipse(f, cv::Point(c.x, c.y + 1), cv::Size(10, 9), 0, 0, 180, col, 2, cv::LINE_AA);
                cv::line(f, {c.x, c.y + 10}, {c.x, c.y + 14}, col, 2, cv::LINE_AA);
                cv::line(f, {c.x - 6, c.y + 14}, {c.x + 6, c.y + 14}, col, 2, cv::LINE_AA);
                break;
            }
            case 1: {   // parlante
                std::vector<cv::Point> sp = {{c.x - 11, c.y - 4}, {c.x - 5, c.y - 4}, {c.x + 2, c.y - 10},
                                             {c.x + 2, c.y + 10}, {c.x - 5, c.y + 4}, {c.x - 11, c.y + 4}};
                cv::fillPoly(f, sp, col, cv::LINE_AA);
                cv::ellipse(f, {c.x + 3, c.y}, {7, 7}, 0, -50, 50, col, 2, cv::LINE_AA);
                cv::ellipse(f, {c.x + 3, c.y}, {12, 12}, 0, -50, 50, col, 2, cv::LINE_AA);
                break;
            }
            case 2: {   // nota musical
                cv::ellipse(f, {c.x - 4, c.y + 7}, {6, 4}, -20, 0, 360, col, cv::FILLED, cv::LINE_AA);
                cv::line(f, {c.x + 1, c.y + 6}, {c.x + 1, c.y - 10}, col, 2, cv::LINE_AA);
                cv::line(f, {c.x + 1, c.y - 10}, {c.x + 8, c.y - 4}, col, 2, cv::LINE_AA);
                break;
            }
            case 3: {   // calendario
                kd::strokeRound(f, cv::Rect(c.x - 10, c.y - 8, 20, 19), 4, col, 2);
                cv::rectangle(f, cv::Rect(c.x - 10, c.y - 8, 20, 6), col, cv::FILLED, cv::LINE_AA);
                cv::line(f, {c.x - 5, c.y - 12}, {c.x - 5, c.y - 6}, col, 2, cv::LINE_AA);
                cv::line(f, {c.x + 5, c.y - 12}, {c.x + 5, c.y - 6}, col, 2, cv::LINE_AA);
                break;
            }
            default: {  // engranaje
                for (int k = 0; k < 8; ++k) {
                    double ang = k * CV_PI / 4.0;
                    cv::line(f, {(int)(c.x + 8 * std::cos(ang)), (int)(c.y + 8 * std::sin(ang))},
                                {(int)(c.x + 12 * std::cos(ang)), (int)(c.y + 12 * std::sin(ang))}, col, 3, cv::LINE_AA);
                }
                cv::circle(f, c, 8, col, cv::FILLED, cv::LINE_AA);
                cv::circle(f, c, 3, kd::white(), cv::FILLED, cv::LINE_AA);
                break;
            }
        }
    }

    // ---------------------------------------------------------------- paginas
    void pageDevices(cv::Mat& f, const cv::Rect& P, bool isMic) {
        const cv::Scalar white = kd::white();
        auto& devs    = isMic ? mics : spks;
        auto& scroll  = isMic ? micScroll : spkScroll;
        auto& selIdx  = isMic ? selMic : selSpk;
        auto& rowsBuf = isMic ? micRows : spkRows;
        cv::Rect& upBtn = isMic ? btnMicUp : btnSpkUp;
        cv::Rect& dnBtn = isMic ? btnMicDown : btnSpkDown;

        btnRefresh = cv::Rect(P.x + P.width - 20 - 150, P.y + 10, 150, 38);
        kd::button(f, btnRefresh, eng() ? "REFRESH" : "REFRESCAR", caramel(), btnRefresh.contains(hov_), true, 0.5);

        int n = (int)devs.size();
        scroll = std::max(0, std::min(scroll, n - kRows));
        const int rowH = 72, rowGap = 12, btnSz = 64;
        int listX = P.x + 20, listY = P.y + 74;
        int listW = P.width - 40 - btnSz - 14;

        rowsBuf.clear();
        for (int i = 0; i < kRows; ++i) {
            int di = scroll + i;
            if (di >= n) break;
            cv::Rect r(listX, listY + i * (rowH + rowGap), listW, rowH);
            rowsBuf.push_back(r);
            bool sel = (di == selIdx);
            bool hov = r.contains(hov_);
            kd::fillRound(f, cv::Rect(r.x, r.y + 3, r.width, r.height), 18, kd::shade());
            kd::fillRound(f, r, 18, sel ? kd::mix(orange(), white, 0.88) : (hov ? cv::Scalar(246, 250, 252) : white));
            kd::strokeRound(f, r, 18, sel ? orange() : tan(), sel ? 3 : 2);
            cv::Point rc(r.x + 34, r.y + r.height / 2);
            cv::circle(f, rc, 14, sel ? orange() : cv::Scalar(200, 206, 210), 3, cv::LINE_AA);
            if (sel) cv::circle(f, rc, 7, orange(), cv::FILLED, cv::LINE_AA);

            std::string label = devs[di].label;
            int maxW = r.width - 80, bl = 0;
            if (txu::size(label, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &bl).width > maxW) {
                while (label.size() > 3 &&
                       txu::size(label + "..", cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &bl).width > maxW)
                    label.pop_back();
                label += "..";
            }
            cv::Size ts = txu::size(label, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &bl);
            txu::put(f, label, {r.x + 62, r.y + (r.height + ts.height) / 2}, cv::FONT_HERSHEY_DUPLEX, 0.6,
                        sel ? kd::ink() : cv::Scalar(90, 95, 100), 1, cv::LINE_AA);
        }
        int bx = listX + listW + 14;
        int listH = kRows * rowH + (kRows - 1) * rowGap;
        upBtn = cv::Rect(bx, listY, btnSz, btnSz);
        dnBtn = cv::Rect(bx, listY + listH - btnSz, btnSz, btnSz);
        roundBtn(f, upBtn, 2, scroll > 0);
        roundBtn(f, dnBtn, 3, scroll + kRows < n);
    }

    void applyMix() {
        if (audioMutex_) { std::lock_guard<std::mutex> lk(*audioMutex_); Mix_VolumeMusic(volumeLevel); }
        else Mix_VolumeMusic(volumeLevel);
    }
    void applySystemVolume() {
        int pct = ((isMuted ? 0 : volumeLevel) * 100) / 128;
        std::string cmd = "pactl set-sink-volume @DEFAULT_SINK@ " + std::to_string(pct) + "% 2>/dev/null &";
        std::system(cmd.c_str());
    }
    void setVolume(int v) {
        v = std::max(0, std::min(128, v));
        volumeLevel = v;
        isMuted = (v == 0);
        if (v > 0) previousVolume = v;
        applyMix();
    }
    void dragTo(int x) {
        if (sliderTrackRect.width <= 0) return;
        int relX = std::max(0, std::min(sliderTrackRect.width, x - sliderTrackRect.x));
        setVolume((relX * 128) / sliderTrackRect.width);
    }

    void pageVolume(cv::Mat& f, const cv::Rect& P) {
        const cv::Scalar white = kd::white();
        int pct = (isMuted ? 0 : volumeLevel) * 100 / 128;
        putC(f, std::to_string(pct) + "%", P.x + P.width / 2, P.y + 130, cv::FONT_HERSHEY_DUPLEX, 1.8, kd::ink(), 1);

        int sy = P.y + 196;
        muteBtnRect = cv::Rect(P.x + 24, sy - 36, 72, 72);
        bool hm = muteBtnRect.contains(hov_);
        cv::Point mc(muteBtnRect.x + 36, muteBtnRect.y + 36);
        cv::Scalar mf = isMuted ? cv::Scalar(150, 150, 158) : (hm ? kd::mix(orange(), white, 0.2) : orange());
        cv::circle(f, {mc.x, mc.y + 4}, 36, kd::shade(), cv::FILLED, cv::LINE_AA);
        cv::circle(f, mc, 36, mf, cv::FILLED, cv::LINE_AA);
        cv::circle(f, mc, 36, white, 3, cv::LINE_AA);
        auto P2 = [&](double dx, double dy) { return cv::Point(mc.x + (int)(dx * 1.8), mc.y + (int)(dy * 1.8)); };
        std::vector<cv::Point> sp = {P2(-11, -4), P2(-5, -4), P2(2, -10), P2(2, 10), P2(-5, 4), P2(-11, 4)};
        cv::fillPoly(f, sp, white, cv::LINE_AA);
        if (isMuted) {
            cv::line(f, P2(5, -7), P2(14, 7), white, 4, cv::LINE_AA);
            cv::line(f, P2(14, -7), P2(5, 7), white, 4, cv::LINE_AA);
        } else {
            cv::ellipse(f, P2(3, 0), cv::Size(13, 13), 0, -50, 50, white, 3, cv::LINE_AA);
            cv::ellipse(f, P2(3, 0), cv::Size(22, 22), 0, -50, 50, white, 3, cv::LINE_AA);
        }

        int tx = muteBtnRect.x + muteBtnRect.width + 30;
        int tw = P.x + P.width - 40 - tx;
        sliderTrackRect = cv::Rect(tx, sy - 9, tw, 18);
        kd::fillRound(f, sliderTrackRect, 9, cv::Scalar(225, 232, 236));
        int fillW = (isMuted ? 0 : volumeLevel) * tw / 128;
        if (fillW > 4) kd::fillRound(f, cv::Rect(tx, sy - 9, fillW, 18), 9, orange());
        cv::Rect hit(tx - 24, sy - 30, tw + 48, 60);
        bool hs = hit.contains(hov_) || isDraggingVolume;
        cv::Point kn(tx + fillW, sy);
        cv::circle(f, {kn.x, kn.y + 3}, 22, kd::shade(), cv::FILLED, cv::LINE_AA);
        cv::circle(f, kn, 22, white, cv::FILLED, cv::LINE_AA);
        cv::circle(f, kn, 22, hs ? kd::mix(orange(), white, 0.2) : orange(), 4, cv::LINE_AA);

        int cx = P.x + P.width / 2;
        btnVolDown_ = cv::Rect(cx - 90 - 34, P.y + 250, 68, 68);
        btnVolUp_   = cv::Rect(cx + 90 - 34, P.y + 250, 68, 68);
        roundBtn(f, btnVolDown_, 0);
        roundBtn(f, btnVolUp_, 1);
    }

    int daysIn(int m, int y) const {
        static const int d[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
        int v = d[(std::max(1, std::min(12, m)) - 1)];
        if (m == 2 && (y % 400 == 0 || (y % 4 == 0 && y % 100 != 0))) v = 29;
        return v;
    }
    void dayDelta(int d) {
        int md = daysIn(editMon, editYear);
        editDay += d;
        if (editDay > md) editDay = 1;
        if (editDay < 1)  editDay = md;
    }
    void monDelta(int d) {
        editMon = ((editMon - 1 + d + 12) % 12) + 1;
        editDay = std::min(editDay, daysIn(editMon, editYear));
    }

    void pageDate(cv::Mat& f, const cv::Rect& P) {
        const cv::Scalar white = kd::white();
        std::time_t tn = std::time(nullptr);
        char sb[64];
        std::strftime(sb, sizeof(sb), eng() ? "System: %d/%m/%Y" : "Sistema: %d/%m/%Y", std::localtime(&tn));
        int bl = 0;
        cv::Size ss = txu::size(sb, cv::FONT_HERSHEY_DUPLEX, 0.55, 1, &bl);
        txu::put(f, sb, {P.x + P.width - 24 - ss.width, P.y + 38}, cv::FONT_HERSHEY_DUPLEX, 0.55,
                    kd::gray(), 1, cv::LINE_AA);

        static const char* mEs[12] = {"ENERO","FEBRERO","MARZO","ABRIL","MAYO","JUNIO",
                                      "JULIO","AGOSTO","SEPTIEMBRE","OCTUBRE","NOVIEMBRE","DICIEMBRE"};
        static const char* mEn[12] = {"JANUARY","FEBRUARY","MARCH","APRIL","MAY","JUNE",
                                      "JULY","AUGUST","SEPTEMBER","OCTOBER","NOVEMBER","DECEMBER"};
        const int btn = 80, boxW = 260, g = 16;
        int total = btn + g + boxW + g + btn;
        int x0 = P.x + (P.width - total) / 2;
        for (int r = 0; r < 2; ++r) {
            int y = P.y + 76 + r * 108;
            cv::Rect dn(x0, y, btn, btn);
            cv::Rect box(x0 + btn + g, y, boxW, btn);
            cv::Rect up(box.x + boxW + g, y, btn, btn);
            if (r == 0) { btnDayDown = dn; btnDayUp = up; } else { btnMonDown = dn; btnMonUp = up; }
            roundBtn(f, dn, 0);
            roundBtn(f, up, 1);
            kd::fillRound(f, box, 24, kd::mix(orange(), white, 0.9));
            kd::strokeRound(f, box, 24, tan(), 2);
            std::string lbl = (r == 0) ? (eng() ? "DAY" : "DÍA") : (eng() ? "MONTH" : "MES");
            putC(f, lbl, box.x + box.width / 2, box.y + 25, cv::FONT_HERSHEY_DUPLEX, 0.5, kd::gray(), 1);
            char vb[32];
            if (r == 0) snprintf(vb, sizeof(vb), "%02d", editDay);
            else        snprintf(vb, sizeof(vb), "%s", (eng() ? mEn : mEs)[(editMon - 1) % 12]);
            putC(f, vb, box.x + box.width / 2, box.y + 64, cv::FONT_HERSHEY_DUPLEX, 0.95, kd::ink(), 1);
        }
    }

    void pageGeneral(cv::Mat& f, const cv::Rect& P) {
        txu::put(f, eng() ? "LANGUAGE" : "IDIOMA", {P.x + 26, P.y + 84},
                    cv::FONT_HERSHEY_DUPLEX, 0.55, kd::gray(), 1, cv::LINE_AA);
        btnLang = cv::Rect(P.x + 24, P.y + 94, P.width - 48, 64);
        kd::button(f, btnLang, eng() ? "ENGLISH   >   ESPAÑOL" : "ESPAÑOL   >   ENGLISH",
                   caramel(), btnLang.contains(hov_), true, 0.62);

        txu::put(f, eng() ? "NETWORK" : "RED", {P.x + 26, P.y + 190},
                    cv::FONT_HERSHEY_DUPLEX, 0.55, kd::gray(), 1, cv::LINE_AA);
        btnWifi = cv::Rect(P.x + 24, P.y + 200, P.width - 48, 64);
        kd::button(f, btnWifi, "WIFI", caramel(), btnWifi.contains(hov_), true, 0.62);

        putC(f, "IP: " + localIP, P.x + P.width / 2, P.y + 308, cv::FONT_HERSHEY_DUPLEX, 0.6, kd::gray(), 1);
    }

    // ---------------------------------------------------------------- guardado
    void doSave() {
        bool audioChanged = (selectedMicId != initialMicId || selectedSpkId != initialSpkId);
        bool dateChanged  = (editDay != initDay || editMon != initMon ||
                             editYear != editYear_init || editHour != initHour || editMin != initMin);
        if (audioChanged && dateChanged) {
            saveAudioConfig();
            applyDateTime();
            setFeedback(eng() ? "Audio & Date saved" : "Se guardó Audio y Fecha");
        } else if (audioChanged) {
            saveAudioConfig();
            setFeedback(eng() ? "Only Audio updated" : "Solo Audio actualizado");
        } else if (dateChanged) {
            applyDateTime();
            setFeedback(eng() ? "Only Date/Time updated" : "Solo Fecha/Hora actualizada");
        } else {
            setFeedback(eng() ? "Nothing has changed" : "Nada ha cambiado");
        }
        initialMicId = selectedMicId;
        initialSpkId = selectedSpkId;
        initDay = editDay; initMon = editMon; editYear_init = editYear;
        initHour = editHour; initMin = editMin;
    }

    void clampDate() {
        editMon = std::max(1, std::min(12, editMon));
        editDay = std::max(1, std::min(daysIn(editMon, editYear), editDay));
    }
    void applyDateTime() {
        clampDate();
        char buf[128];
        snprintf(buf, sizeof(buf),
                 "sudo timedatectl set-time '%04d-%02d-%02d %02d:%02d:00' 2>/dev/null",
                 editYear, editMon, editDay, editHour, editMin);
        int ret = std::system(buf);
        if (ret == 0) setFeedback((eng() ? "Date applied: " : "Fecha aplicada: ") +
                                  std::to_string(editDay) + "/" + std::to_string(editMon) + "/" + std::to_string(editYear));
        else setFeedback(eng() ? "Error: check sudo NOPASSWD for timedatectl" : "Error: verifica sudo NOPASSWD para timedatectl");
    }
    void saveAudioConfig() {
        bool ok = true;
        int volPorcentaje = ((isMuted ? 0 : volumeLevel) * 100) / 128;
        std::string volCmd = "pactl set-sink-volume @DEFAULT_SINK@ " + std::to_string(volPorcentaje) + "% 2>/dev/null";
        std::system(volCmd.c_str());

        if (!selectedSpkId.empty()) {
            std::string cmd = "pactl set-default-sink '" + selectedSpkId + "' 2>/dev/null";
            if (std::system(cmd.c_str()) != 0) ok = false;
            std::system(
                ("for i in $(pactl list sink-inputs short 2>/dev/null | awk '{print $1}'); "
                 "do pactl move-sink-input $i '" + selectedSpkId + "' 2>/dev/null; done").c_str());
        }
        if (!selectedMicId.empty()) {
            std::string cmd = "pactl set-default-source '" + selectedMicId + "' 2>/dev/null";
            if (std::system(cmd.c_str()) != 0) ok = false;
            std::system(
                ("for i in $(pactl list source-outputs short 2>/dev/null | awk '{print $1}'); "
                 "do pactl move-source-output $i '" + selectedMicId + "' 2>/dev/null; done").c_str());
        }
        auto writeFile = [](const char* path, const std::string& val) {
            FILE* fp = fopen(path, "w");
            if (fp) { fprintf(fp, "%s\n", val.c_str()); fclose(fp); }
        };
        writeFile("/tmp/yaren_mic_device.txt", selectedMicId);
        writeFile("/tmp/yaren_spk_device.txt", selectedSpkId);
        if (ok) setFeedback(eng() ? "Audio applied correctly" : "Audio aplicado correctamente");
        else    setFeedback(eng() ? "Warning: a device failed"  : "Advertencia: algún dispositivo falló");
    }
};


// =============================================================================
//  RadioApp
// =============================================================================
struct SongInfo {
    std::string id;
    std::string title;
    std::string subtitle;
    std::string audioFile;
    cv::Scalar  color;
    double      beatHz;
};
enum class RadioState { SELECTING, PLAYING };

class RadioApp {
public:
    std::function<void()> onBack;
    std::vector<SongInfo> songs;
    RadioState state { RadioState::SELECTING };
    int currentSong { -1 };
    bool* isEnglish = nullptr;
    bool eng() const { return isEnglish != nullptr && *isEnglish; }

    // FIX-E: puntero a mutex de audio compartido
    void setAudioMutex(std::mutex* m) { audioMutex_ = m; }

    void drawNavArrow(cv::Mat& frame, const cv::Rect& r, bool left, bool hovered) {
        cv::Scalar color = hovered ? cv::Scalar(255, 100, 255) : cv::Scalar(150, 80, 180);
        cv::rectangle(frame, r, cv::Scalar(20, 15, 40), cv::FILLED);
        cv::rectangle(frame, r, color, hovered ? 2 : 1, cv::LINE_AA);
        int cx = r.x + r.width/2;
        int cy = r.y + r.height/2;
        int size = 15;
        std::vector<cv::Point> pts(3);
        if (left) {
            pts[0] = {cx + size/2, cy - size};
            pts[1] = {cx + size/2, cy + size};
            pts[2] = {cx - size/2, cy};
        } else {
            pts[0] = {cx - size/2, cy - size};
            pts[1] = {cx - size/2, cy + size};
            pts[2] = {cx + size/2, cy};
        }
        cv::fillPoly(frame, pts, color);
    }

    void render(cv::Mat& frame) {
        if (state == RadioState::SELECTING)
            renderSelector(frame);
        else
            renderNowPlaying(frame);
    }

    void init(const std::string& pkg, rclcpp::Logger log) {
        pkgDir = pkg;
        {
            const std::string dir = pkgDir + "/faces/nuevas_caras";
            faces_.clear();
            cv::Mat openM, closedM;
            std::vector<cv::Mat> extras;
            if (fs::exists(dir)) {
                std::vector<std::string> fl;
                for (const auto& e : fs::directory_iterator(dir)) {
                    std::string ext = e.path().extension().string();
                    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") fl.push_back(e.path().string());
                }
                std::sort(fl.begin(), fl.end());
                for (const auto& f : fl) {
                    cv::Mat img = cv::imread(f, cv::IMREAD_UNCHANGED);
                    if (img.empty()) continue;
                    std::string n = fs::path(f).filename().string();
                    std::transform(n.begin(), n.end(), n.begin(), ::tolower);
                    auto has = [&](const char* k) { return n.find(k) != std::string::npos; };
                    if (has("cerr") || has("close") || has("blink")) closedM = img;
                    else if (has("abier") || has("open"))            openM   = img;
                    else                                             extras.push_back(img);
                }
            }
            if (!openM.empty()) faces_.push_back({openM, closedM});
            for (auto& m : extras) faces_.push_back({m, cv::Mat()});
            if (faces_.empty())
                RCLCPP_WARN(rclcpp::get_logger("RadioApp"), "No se encontraron caras en %s", dir.c_str());
        }
        logger_ = log;
        currentPage = 0;
        songs = {
            // Canciones base originales
            { "aramsamsam",    "ARA RAM SAM SAM",         "Luli Pampín",         "Luli Pampín - ARAM SAM SAM 2021.mp3",                                           {220, 100, 255}, 3.0 },
            { "gorila",        "BAILE DE GORILA",         "CantaJuego",          "CantaJuego - El Baile del Gorila.mp3",                                          {180,  60, 255}, 2.5 },
            { "barney",        "BARNEY ES UN DINOSAURIO", "Barney y sus amigos", "Intro de Barney y sus amigos.mp3",                                              {200, 100, 100}, 2.0 },
            { "chipichapa",    "CHOPI CHOPI",             "Christell",           "Chipi chipi chapa chapa dubi dubi daba daba Christell - Dubidubidu Subtitulada en español.mp3", { 50, 200, 255}, 3.5 },
            { "libresoy",      "LIBRE SOY",               "Martina Stoessel",    "Martina Stoessel_ Libre Soy - Frozen_ Una Aventura Congelada.mp3",              {255, 180,  50}, 1.8 },
            { "sasa",          "SA SA",                   "Luli Pampín",         "Luli Pampín - SASA LA SERPIENTE (Official Video).mp3",                          { 80, 255, 180}, 2.8 },
            { "sitienesganas", "SI TIENES GANAS",         "Luli Pampín",         "Luli Pampín - SI TÚ TIENES MUCHAS GANAS DE APLAUDIR - Official Video.mp3",      { 60, 180, 255}, 3.2 },

            // Nuevas canciones añadidas desde el directorio audios
            { "superestrella", "SUPERESTRELLA",           "Aitana",              "Aitana - SUPERESTRELLA.mp3",              {255, 100, 150}, 2.6 },
            { "balada",        "BALADA",                  "Desconocido",         "Balada.mp3",                              {100, 150, 255}, 2.0 },
            { "rosapastel",    "ROSA PASTEL",             "Belanova",            "Belanova - Rosa Pastel.mp3",              {255, 182, 193}, 2.5 },
            { "bringmetolife", "BRING ME TO LIFE",        "Evanescence",         "bringmetolife.mp3",                       {105, 105, 105}, 3.0 },
            { "Wavin' Flag",          "WAVIN' FLAG",      "K'NAAN",             "cancionmundial2010.mp3",                  {255, 215,   0}, 2.8 },
            { "cantstop",      "CANT STOP THE FEELING",   "Justin Timberlake",   "CantStopTheFeeling.mp3",                  {255, 140,   0}, 2.7 },
            { "getlucky",      "GET LUCKY",               "Daft Punk",           "GetLucky.mp3",                            { 50, 205,  50}, 2.4 },
            { "ghost",         "GHOST",                   "Justin Bieber",       "Ghost.mp3",                               {200, 200, 200}, 2.2 },
            { "gotaslluvia",   "GOTAS DE LLUVIA",         "Grupo Niche",         "Gotas De Lluvia, Grupo Niche.mp3",        {  0, 191, 255}, 2.6 },
            { "iris",          "IRIS",                    "Goo Goo Dolls",       "IRIS.mp3",                                {138,  43, 226}, 2.0 },
            { "beautyandabeat","BEAUTY AND A BEAT",       "Justin Bieber",       "Justin Bieber - Beauty And A Beat.mp3",   {255,  20, 147}, 2.9 },
            { "justtheway",    "JUST THE WAY YOU ARE",    "Bruno Mars",          "justthewayuare.mp3",                      {220,  20,  60}, 2.3 },
            { "labicicleta",   "LA BICICLETA",            "Carlos Vives",        "LaBicicleta.mp3",                         { 34, 139,  34}, 2.7 },
            { "ladespedida",   "LA DESPEDIDA",            "Daddy Yankee",        "La Despedida.mp3",                        {255,  69,   0}, 2.8 },
            { "lagozadera",    "LA GOZADERA",             "Gente de Zona",       "LaGozadera.mp3",                          {255, 215,   0}, 3.0 },
            { "lovemeagain",   "LOVE ME AGAIN",           "John Newman",         "Love Me Again.mp3",                       {255,   0,   0}, 2.6 },
            { "migente",       "MI GENTE",                "J Balvin",            "MiGente.mp3",                             {124, 252,   0}, 2.8 },
            { "picky",         "PICKY",                   "Joey Montana",        "Picky.mp3",                               {255, 105, 180}, 2.7 },
            { "lacarretera",   "LA CARRETERA",            "Prince Royce",        "Prince Royce - La Carretera.mp3",         { 70, 130, 180}, 2.2 },
            { "dontstop",      "DONT STOP THE MUSIC",     "Rihanna",             "Rihanna - Don't Stop The Music.mp3",      {148,   0, 211}, 2.9 },
            { "spiderman",     "SPIDERMAN THEME",         "Marvel",              "Spiderman.mp3",                           {220,  20,  60}, 2.5 },
            { "suerte",        "SUERTE",                  "Shakira",             "SUERTE.mp3",                              {218, 165,  32}, 2.4 },
            { "sunflower",     "SUNFLOWER",               "Post Malone",         "SunFlower.mp3",                           {255, 215,   0}, 2.3 },
            { "theonethat",    "THE ONE THAT GOT AWAY",   "Katy Perry",          "The One That Got Away.mp3",               {238, 130, 238}, 2.1 },
            { "winner",        "THE WINNER TAKES IT ALL", "ABBA",                "The Winner Takes It All.mp3",             {192, 192, 192}, 2.0 },
            { "tucarcel",      "TU CÁRCEL",               "Los Bukis",           "TuCarcel.mp3",                            {139,  69,  19}, 2.2 },
            { "unbeso",        "UN BESO Y UNA FLOR",      "Nino Bravo",          "Un Beso Y Una Flor.mp3",                  {255, 160, 122}, 2.0 },
            { "yourlove",      "YOUR LOVE",               "The Outfield",        "YourLove.mp3",                            {  0, 191, 255}, 2.5 }
        };
        if (SDL_Init(SDL_INIT_AUDIO) < 0) {
            RCLCPP_ERROR(logger_, "No se pudo inicializar SDL: %s", SDL_GetError());
        } else if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0) {
            RCLCPP_ERROR(logger_, "No se pudo inicializar SDL_mixer: %s", Mix_GetError());
            SDL_Quit();
        } else {
            audioInitialized = true;
        }
        lastBlinkTime   = std::chrono::steady_clock::now();
        lastFaceSwitch  = std::chrono::steady_clock::now();
        altFaceStartTime = std::chrono::steady_clock::now();
        showingAltFace      = false;
        currentAltFaceIndex = 0;
        isBlinking          = false;
        currentSongIndex    = -1;
    }

    ~RadioApp() {
        killAudio();
        if (audioInitialized) {
            Mix_CloseAudio();
            SDL_Quit();
        }
    }

    // FIX-E: killAudio protegido con audioMutex_
    void killAudio() {
        if (audioMutex_) {
            std::lock_guard<std::mutex> lk(*audioMutex_);
            killAudioImpl();
        } else {
            killAudioImpl();
        }
    }

    void reset() {
        killAudio();
        viewMode_ = 0;
        state = RadioState::SELECTING;
        currentSong = -1;
        hovSong = -1;
        hovStop = false;
        hovBack = false;
        currentSongIndex = -1;
    }

    void playSong(int idx) {
        if (idx < 0 || idx >= (int)songs.size()) return;
        killAudio();
        currentSongIndex = idx;
        currentSong = idx;
        showFace = true;
        state = RadioState::PLAYING;
        playStart = std::chrono::steady_clock::now(); paused_ = false; pausedTotal_ = 0.0;
        currentFaceIndex = 0;
        lastFaceSwitch = std::chrono::steady_clock::now();
        isBlinking = false;
	std::string absCwd = std::filesystem::current_path().string();
	std::string audioPath = absCwd + "/src/YAREN2/yaren_radio/audios/" + songs[idx].audioFile;
        if (!fs::exists(audioPath)) {
            state = RadioState::SELECTING;
            currentSong = -1;
            currentSongIndex = -1;
            return;
        }
        songDuration = probeDuration(audioPath);
        if (audioInitialized) {
            Mix_Music* music = Mix_LoadMUS(audioPath.c_str());
            if (music) {
                // FIX-E: proteger operaciones SDL con mutex
                bool started = false;
                if (audioMutex_) {
                    std::lock_guard<std::mutex> lk(*audioMutex_);
                    currentMusic = music;
                    started = (Mix_FadeInMusic(currentMusic, -1, 500) == 0);
                } else {
                    currentMusic = music;
                    started = (Mix_FadeInMusic(currentMusic, -1, 500) == 0);
                }
                if (started) {
                    std::system("python3 src/YAREN2/yaren_movements/yaren_movements/yaren_dance_radio.py &");
                    return;
                }
                if (audioMutex_) {
                    std::lock_guard<std::mutex> lk(*audioMutex_);
                    Mix_FreeMusic(currentMusic);
                    currentMusic = nullptr;
                } else {
                    Mix_FreeMusic(currentMusic);
                    currentMusic = nullptr;
                }
            } else {
                RCLCPP_ERROR(logger_, "Mix_LoadMUS falló para: %s — %s", audioPath.c_str(), Mix_GetError());
            }
        }
        std::string cmd = "mpg123 -q \"" + audioPath + "\" &";
        std::thread([cmd]() { std::system(cmd.c_str()); }).detach();
        std::system("python3 src/YAREN2/yaren_movements/yaren_movements/yaren_dance_radio.py &");
    }

    void handleMouse(int ev, int x, int y) {
        mousePos = {x, y, 1, 1};
        const cv::Point pt(x, y);
        hovPt_ = pt;
        if (ev != cv::EVENT_LBUTTONDOWN) return;
        auto in = [&](const cv::Rect& r) { return r.area() > 0 && r.contains(pt); };
        if (fav_.size() != songs.size()) fav_.assign(songs.size(), false);

        if (state == RadioState::SELECTING) {
            if (in(backBtn))     { killAudio(); if (onBack) onBack(); return; }
            if (in(tabMusic_))   { favTab_ = false; return; }
            if (in(tabFav_))     { favTab_ = true;  return; }
            if (in(prevSongBtn)) { stepSong(-1); return; }
            if (in(nextSongBtn)) { stepSong(+1); return; }
            if (in(playBtn))     { if (selSong >= 0) playSong(selSong); return; }
            if (in(queueBtn_))   { randomSong(false); return; }
            for (int i = 0; i < (int)cardRects.size() && i < (int)cardSong_.size(); ++i) {
                if (cardRects[i].contains(pt)) {
                    int idx = cardSong_[i];
                    if (idx == selSong) playSong(idx);
                    else selSong = idx;
                    return;
                }
            }
        } else if (state == RadioState::PLAYING) {
            if (viewMode_ != 0) { viewMode_ = (viewMode_ + 1) % 3; return; }
            if (in(backBtn) || in(chevBtn_) || in(stopBtn)) { stopToList(); return; }
            if (in(ppBtn_))      { togglePause(); return; }
            if (in(prevSongBtn)) { playStep(-1); return; }
            if (in(nextSongBtn)) { playStep(+1); return; }
            if (in(heartBtn_))   { if (currentSong >= 0 && currentSong < (int)fav_.size()) fav_[currentSong] = !fav_[currentSong]; return; }
            if (in(repeatBtn_))  { repeatOn_ = !repeatOn_; return; }
            if (in(queueBtn_))   { randomSong(true); return; }
            if (in(volBtn_))     { toggleMute(); return; }
            if (in(dotsBtn_) || in(viewBtn)) { viewMode_ = 1; return; }
        }
    }


private:
    std::string pkgDir;
    rclcpp::Logger logger_ = rclcpp::get_logger("RadioApp");
    int currentPage { 0 };
    cv::Rect prevPageBtn {0,0,0,0};
    cv::Rect nextPageBtn {0,0,0,0};
    bool hoveredPrevPage { false };
    bool hoveredNextPage { false };
    std::vector<cv::Mat> altFaces;
    size_t currentAltFaceIndex { 0 };
    bool showingAltFace { false };
    int currentFaceIndex { 0 };
    std::chrono::steady_clock::time_point altFaceStartTime;
    std::chrono::steady_clock::time_point lastFaceSwitch;
    double faceSwitchInterval { 8.0 };
    bool audioInitialized { false };
    Mix_Music* currentMusic { nullptr };
    int currentSongIndex { -1 };
    cv::Mat eyesOpen, eyesClosed, mouthOpen, mouthClosed;
    cv::Mat faceOpenNew_, faceClosedNew_;
    struct FaceSet { cv::Mat open; cv::Mat closed; };
    std::vector<FaceSet> faces_;
    int hovSong { -1 };
    bool hovStop { false };
    bool hovBack { false };
    bool isBlinking { false };
    std::chrono::steady_clock::time_point lastBlinkTime, blinkStart;
    std::chrono::steady_clock::time_point playStart;
    std::vector<cv::Rect> cardRects;
    cv::Rect stopBtn {0,0,0,0};
    cv::Rect backBtn {0,0,0,0};
    cv::Rect mousePos {0,0,1,1};
    int  selSong { 0 };
    bool showFace { true };
    cv::Rect viewBtn {0,0,0,0};
    bool hovView { false };
    double songDuration { 0.0 };
    cv::Rect playBtn {0,0,0,0};
    bool hovPlay { false };
    cv::Rect prevSongBtn {0,0,0,0}, nextSongBtn {0,0,0,0};
    bool hovPrevSong { false }, hovNextSong { false };
    // FIX-E: mutex de audio compartido
    std::mutex* audioMutex_ {nullptr};

    // FIX-E: implementación interna de killAudio (sin lock, el caller lo gestiona)
    void killAudioImpl() {
        if (audioInitialized && Mix_PlayingMusic()) {
            Mix_HaltMusic();
        }
        if (currentMusic) {
            Mix_FreeMusic(currentMusic);
            currentMusic = nullptr;
        }
        currentSongIndex = -1;
        std::system("for pid in $(ps aux | grep -E 'yaren_dance_radio.py' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done");
        std::system("ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\" > /dev/null 2>&1 &");
    }

    double probeDuration(const std::string& path) {
        std::string cmd = "ffprobe -v error -show_entries format=duration "
                          "-of default=noprint_wrappers=1:nokey=1 \"" + path + "\" 2>/dev/null";
        FILE* f = popen(cmd.c_str(), "r");
        if (!f) return 0.0;
        char buf[64] = {};
        double d = 0.0;
        if (fgets(buf, sizeof(buf), f)) d = std::atof(buf);
        pclose(f);
        return d > 0.0 ? d : 0.0;
    }

    void drawVideoView(cv::Mat& f, const cv::Rect& card, const SongInfo& s, double t, double beat) {
        const cv::Scalar white = kd::white();
        cv::Rect inner(card.x + 10, card.y + 10, card.width - 20, card.height - 20);
        kd::fillRound(f, inner, 20, kd::mix(s.color, white, 0.82));
        cv::Mat roi = f(inner);
        cv::Point c(inner.width / 2, inner.height / 2 - 18);
        for (int i = 0; i < 3; ++i) {
            double ph = std::fmod(t * beat * 0.25 + i / 3.0, 1.0);
            int rad = (int)(100 + ph * 55);
            double al = (1.0 - ph) * 0.55;
            cv::Mat ov = roi.clone();
            cv::circle(ov, c, rad, kd::pal(i * 2 + 1), 5, cv::LINE_AA);
            cv::addWeighted(ov, al, roi, 1.0 - al, 0, roi);
        }
        cv::Point cf(inner.x + c.x, inner.y + c.y);
        drawVinyl(f, cf, 92, s.color, t, true);
        for (int k = 0; k < 4; ++k) {
            double ang = t * 1.2 + k * 1.57;
            kd::sparkle(f, {(int)(cf.x + 150 * std::cos(ang)), (int)(cf.y + 110 * std::sin(ang))},
                        5 + (k % 2) * 3, kd::pal(k + 1));
        }
        const int BARS = 24, bw = 14, gap = 6;
        int ex = inner.x + (inner.width - (BARS * bw + (BARS - 1) * gap)) / 2;
        int baseY = inner.y + inner.height - 14;
        for (int i = 0; i < BARS; ++i) {
            double amp = std::abs(std::sin(t * beat * CV_PI + i * 0.45) * 0.5 +
                                  std::sin(t * beat * CV_PI * 0.6 + i * 0.22) * 0.3 +
                                  std::sin(t * 1.1 + i * 0.15) * 0.2);
            int h = 8 + (int)(52 * amp);
            kd::fillRound(f, cv::Rect(ex + i * (bw + gap), baseY - h, bw, h), 6, kd::pal(i));
        }
    }

    void drawFitCentered(cv::Mat& f, const std::string& s, int cx, int y, int font,
                         double sc, double minSc, int maxW,
                         const cv::Scalar& col, const cv::Scalar& outline, int th = 1) {
        int bl = 0;
        while (sc > minSc && txu::size(s, font, sc, th, &bl).width > maxW) sc -= 0.03;
        cv::Size ts = txu::size(s, font, sc, th, &bl);
        kd::text(f, s, cv::Point(cx - ts.width / 2, y), font, sc, col, outline, th);
    }

    void drawVinyl(cv::Mat& f, cv::Point c, int R, const cv::Scalar& a, double t, bool spin) {
        const cv::Scalar white = kd::white();
        cv::circle(f, {c.x, c.y + 4}, R, kd::shade(), cv::FILLED, cv::LINE_AA);
        cv::circle(f, c, R, cv::Scalar(58, 52, 50), cv::FILLED, cv::LINE_AA);
        for (int g = R - 6; g > (int)(R * 0.45); g -= 6)
            cv::circle(f, c, g, cv::Scalar(80, 74, 72), 1, cv::LINE_AA);
        double ang = spin ? t * 140.0 : 20.0;
        cv::ellipse(f, c, cv::Size(R - 4, R - 4), 0, ang,       ang + 35,  cv::Scalar(235,235,235), 3, cv::LINE_AA);
        cv::ellipse(f, c, cv::Size(R - 4, R - 4), 0, ang + 180, ang + 215, cv::Scalar(235,235,235), 3, cv::LINE_AA);
        int lr = (int)(R * 0.40);
        cv::circle(f, c, lr, a, cv::FILLED, cv::LINE_AA);
        cv::circle(f, c, lr, white, 3, cv::LINE_AA);
        drawMusicNote(f, {c.x - 4, c.y + 6}, white, std::max(0.5f, R / 70.f));
        cv::circle(f, c, 3, kd::cream(), cv::FILLED, cv::LINE_AA);
    }

    void drawTile(cv::Mat& f, const cv::Rect& base, const SongInfo& s, int idx,
                  bool sel, bool hov, double t) {
        const cv::Scalar white = kd::white();
        int bob = sel ? (int)std::lround(std::sin(t * 3.0 + idx) * 2.0) : 0;
        cv::Rect r(base.x, base.y + bob - (hov ? 3 : 0), base.width, base.height);
        cv::Scalar body = kd::mix(s.color, white, 0.18);

        kd::fillRound(f, cv::Rect(r.x, r.y + 5, r.width, r.height), 22, kd::shade());
        kd::fillRound(f, r, 22, body);
        kd::fillRound(f, cv::Rect(r.x + r.width - 50, r.y + 6, 44, 44), 22, kd::mix(s.color, white, 0.5), 0.5);
        kd::fillRound(f, cv::Rect(r.x + 6, r.y + r.height - 70, 40, 40), 20, kd::mix(s.color, white, 0.5), 0.4);

        cv::Point c(r.x + r.width / 2, r.y + 62);
        cv::circle(f, c, 46, kd::mix(s.color, white, 0.65), cv::FILLED, cv::LINE_AA);
        drawVinyl(f, c, 38, s.color, t, sel);

        cv::Scalar ol = s.color * 0.5;
        drawFitCentered(f, s.title, r.x + r.width / 2, r.y + r.height - 34,
                        cv::FONT_HERSHEY_DUPLEX, 0.5, 0.28, r.width - 14, white, ol, 1);
        drawFitCentered(f, s.subtitle, r.x + r.width / 2, r.y + r.height - 14,
                        cv::FONT_HERSHEY_PLAIN, 0.85, 0.55, r.width - 14, white, ol, 1);

        kd::strokeRound(f, r, 22, white, sel ? 5 : 2);
        if (sel) kd::sparkle(f, {r.x + r.width - 16, r.y + 16}, 8, kd::pal(2));
    }

    void renderPreviewPanel(cv::Mat& f, const cv::Rect& r, const SongInfo& s, double t) {
        kd::card(f, r, 26, s.color, 3);
        cv::Point c(r.x + r.width / 2, r.y + 92);
        cv::circle(f, c, 80, kd::mix(s.color, kd::white(), 0.8), cv::FILLED, cv::LINE_AA);
        drawVinyl(f, c, 66, s.color, t, true);
        for (int k = 0; k < 3; ++k) {
            double ang = t * 1.1 + k * 2.094;
            kd::sparkle(f, {(int)(c.x + 88 * std::cos(ang)), (int)(c.y + 80 * std::sin(ang))},
                        5 + (k % 2) * 3, kd::pal(k * 2 + 1));
        }
        drawFitCentered(f, s.title, c.x, r.y + 202, cv::FONT_HERSHEY_DUPLEX, 0.62, 0.32,
                        r.width - 24, kd::ink(), kd::white(), 1);
        drawFitCentered(f, s.subtitle, c.x, r.y + 226, cv::FONT_HERSHEY_PLAIN, 1.0, 0.6,
                        r.width - 24, kd::gray(), kd::white(), 1);
        const int ay = r.y + r.height - 62;
        prevSongBtn = {r.x + 14, ay, 46, 46};
        nextSongBtn = {r.x + r.width - 14 - 46, ay, 46, 46};
        kd::roundArrow(f, prevSongBtn, true,  kd::pal(4), hovPrevSong);
        kd::roundArrow(f, nextSongBtn, false, kd::pal(4), hovNextSong);
        playBtn = {r.x + 14 + 46 + 10, ay, r.width - 2 * (14 + 46 + 10), 46};
        kd::button(f, playBtn, eng() ? "PLAY" : "REPRODUCIR", kd::pal(3), hovPlay, true, 0.6);
    }

    // ======================= YAREN RADIO: diseno claro neumorfico =======================
    std::vector<bool> fav_;
    bool   favTab_ { false };
    bool   paused_ { false };
    double pausedTotal_ { 0.0 };
    std::chrono::steady_clock::time_point pauseStart_;
    bool   repeatOn_ { true };
    int    preMuteVol_ { 64 };
    cv::Point hovPt_ { -1, -1 };
    std::vector<int> cardSong_;
    cv::Rect tabMusic_, tabFav_, heartBtn_, repeatBtn_, queueBtn_, volBtn_, dotsBtn_, chevBtn_, artRect_, ppBtn_;

    static inline cv::Scalar themeC_ {140.0, 90.0, 250.0};
    static void updateTheme(const cv::Scalar& t) { themeC_ = themeC_ * 0.88 + t * 0.12; }
    static cv::Scalar nBg()    { return kd::mix(themeC_, cv::Scalar(255, 255, 255), 0.60); }
    static cv::Scalar nPanel() { return kd::mix(themeC_, cv::Scalar(255, 255, 255), 0.86); }
    static cv::Scalar nDark()  { return kd::mix(themeC_, cv::Scalar(255, 255, 255), 0.40); }
    static cv::Scalar nLight() { return kd::mix(themeC_, cv::Scalar(255, 255, 255), 0.97); }
    static cv::Scalar nInk()   { return themeC_ * 0.14; }
    static cv::Scalar nGray()  { return kd::mix(themeC_, cv::Scalar(0, 0, 0), 0.35); }
    static cv::Scalar nPink()  { return themeC_ * 0.85; }

    static void blendCircle(cv::Mat& f, cv::Point c, int r, const cv::Scalar& col, double a, int thick = -1) {
        int m = r + std::max(0, thick) + 3;
        cv::Rect rc(c.x - m, c.y - m, 2 * m, 2 * m);
        rc &= cv::Rect(0, 0, f.cols, f.rows);
        if (rc.area() <= 0) return;
        cv::Mat roi = f(rc), ov = roi.clone();
        cv::circle(ov, c - rc.tl(), r, col, thick, cv::LINE_AA);
        cv::addWeighted(ov, a, roi, 1.0 - a, 0, roi);
    }
    static void neuCircle(cv::Mat& f, cv::Point c, int R, bool hov) {
        for (int i = 5; i >= 1; --i) {
            blendCircle(f, cv::Point(c.x + 4, c.y + 5), R + i * 2 - 2, nDark(), 0.16);
            blendCircle(f, cv::Point(c.x - 4, c.y - 4), R + i * 2 - 2, nLight(), 0.22);
        }
        cv::circle(f, c, R, hov ? kd::mix(nPanel(), nLight(), 0.5) : nPanel(), cv::FILLED, cv::LINE_AA);
    }
    static void softShadow(cv::Mat& f, const cv::Rect& r, int rad) {
        for (int i = 6; i >= 1; --i)
            kd::fillRound(f, cv::Rect(r.x + 6 - i, r.y + 12 - i, r.width - 12 + 2 * i, r.height - 12 + 2 * i),
                          rad, nDark(), 0.12);
    }
    static void gradRoundR(cv::Mat& f, const cv::Rect& r, int rad, const cv::Scalar& top, const cv::Scalar& bot) {
        cv::Rect rr = r & cv::Rect(0, 0, f.cols, f.rows);
        if (rr.area() <= 0) return;
        cv::Mat grad(rr.size(), CV_8UC3);
        for (int y = 0; y < rr.height; ++y) {
            double k = (double)(y + rr.y - r.y) / std::max(1, r.height - 1);
            k = std::max(0.0, std::min(1.0, k));
            grad.row(y).setTo(top * (1.0 - k) + bot * k);
        }
        std::vector<cv::Point> p;
        kd::roundPath(p, r, rad);
        for (auto& q : p) { q.x -= rr.x; q.y -= rr.y; }
        cv::Mat mask = cv::Mat::zeros(rr.size(), CV_8UC1);
        cv::fillPoly(mask, p, cv::Scalar(255), cv::LINE_AA);
        grad.copyTo(f(rr), mask);
    }

    // ---- iconos ----
    static void icoPrev(cv::Mat& f, cv::Point c, int s, const cv::Scalar& col) {
        cv::line(f, {c.x - s, c.y - s}, {c.x - s, c.y + s}, col, 3, cv::LINE_AA);
        std::vector<cv::Point> t = {{c.x + s, c.y - s}, {c.x + s, c.y + s}, {c.x - s + 4, c.y}};
        cv::fillPoly(f, t, col, cv::LINE_AA);
    }
    static void icoNext(cv::Mat& f, cv::Point c, int s, const cv::Scalar& col) {
        cv::line(f, {c.x + s, c.y - s}, {c.x + s, c.y + s}, col, 3, cv::LINE_AA);
        std::vector<cv::Point> t = {{c.x - s, c.y - s}, {c.x - s, c.y + s}, {c.x + s - 4, c.y}};
        cv::fillPoly(f, t, col, cv::LINE_AA);
    }
    static void icoPause(cv::Mat& f, cv::Point c, int s, const cv::Scalar& col) {
        cv::rectangle(f, cv::Rect(c.x - s, c.y - s, s * 2 / 3 + 1, 2 * s), col, cv::FILLED, cv::LINE_AA);
        cv::rectangle(f, cv::Rect(c.x + s / 3 + 1, c.y - s, s * 2 / 3 + 1, 2 * s), col, cv::FILLED, cv::LINE_AA);
    }
    static void icoPlay(cv::Mat& f, cv::Point c, int s, const cv::Scalar& col) {
        std::vector<cv::Point> t = {{c.x - s * 2 / 3, c.y - s}, {c.x - s * 2 / 3, c.y + s}, {c.x + s, c.y}};
        cv::fillPoly(f, t, col, cv::LINE_AA);
    }
    static void heartShape(cv::Mat& f, cv::Point c, double s, const cv::Scalar& col) {
        int r = std::max(1, (int)std::lround(s * 0.56));
        cv::circle(f, cv::Point((int)(c.x - s * 0.5), (int)(c.y - s * 0.3)), r, col, cv::FILLED, cv::LINE_AA);
        cv::circle(f, cv::Point((int)(c.x + s * 0.5), (int)(c.y - s * 0.3)), r, col, cv::FILLED, cv::LINE_AA);
        std::vector<cv::Point> tri = {
            {(int)(c.x - s), (int)(c.y - s * 0.05)}, {(int)(c.x + s), (int)(c.y - s * 0.05)}, {c.x, (int)(c.y + s * 1.1)}};
        cv::fillPoly(f, tri, col, cv::LINE_AA);
    }
    static void icoHeart(cv::Mat& f, cv::Point c, bool on) {
        if (on) { heartShape(f, c, 11, nPink()); return; }
        heartShape(f, c, 11, nInk());
        heartShape(f, cv::Point(c.x, c.y + 1), 8, nPanel());
    }
    static void icoRepeat(cv::Mat& f, cv::Point c, const cv::Scalar& col) {
        const int s = 11;
        std::vector<cv::Point> a = {{c.x - s, c.y + s / 3}, {c.x - s, c.y - s / 2}, {c.x + s - 5, c.y - s / 2}};
        std::vector<cv::Point> b = {{c.x + s, c.y - s / 3}, {c.x + s, c.y + s / 2}, {c.x - s + 5, c.y + s / 2}};
        cv::polylines(f, a, false, col, 2, cv::LINE_AA);
        cv::polylines(f, b, false, col, 2, cv::LINE_AA);
        std::vector<cv::Point> h1 = {{c.x + s - 6, c.y - s / 2 - 5}, {c.x + s - 6, c.y - s / 2 + 5}, {c.x + s + 1, c.y - s / 2}};
        std::vector<cv::Point> h2 = {{c.x - s + 6, c.y + s / 2 - 5}, {c.x - s + 6, c.y + s / 2 + 5}, {c.x - s - 1, c.y + s / 2}};
        cv::fillPoly(f, h1, col, cv::LINE_AA);
        cv::fillPoly(f, h2, col, cv::LINE_AA);
    }
    static void icoQueue(cv::Mat& f, cv::Point c, const cv::Scalar& col) {
        cv::line(f, {c.x - 11, c.y - 8}, {c.x + 4, c.y - 8}, col, 2, cv::LINE_AA);
        cv::line(f, {c.x - 11, c.y - 1}, {c.x + 4, c.y - 1}, col, 2, cv::LINE_AA);
        cv::line(f, {c.x - 11, c.y + 6}, {c.x - 2, c.y + 6}, col, 2, cv::LINE_AA);
        std::vector<cv::Point> t = {{c.x + 4, c.y + 1}, {c.x + 4, c.y + 11}, {c.x + 12, c.y + 6}};
        cv::fillPoly(f, t, col, cv::LINE_AA);
    }
    static void icoVolume(cv::Mat& f, cv::Point c, const cv::Scalar& col, bool muted) {
        std::vector<cv::Point> sp = {{c.x - 10, c.y - 4}, {c.x - 4, c.y - 4}, {c.x + 3, c.y - 10},
                                     {c.x + 3, c.y + 10}, {c.x - 4, c.y + 4}, {c.x - 10, c.y + 4}};
        cv::fillPoly(f, sp, col, cv::LINE_AA);
        if (muted) {
            cv::line(f, {c.x + 7, c.y - 5}, {c.x + 15, c.y + 5}, col, 2, cv::LINE_AA);
            cv::line(f, {c.x + 15, c.y - 5}, {c.x + 7, c.y + 5}, col, 2, cv::LINE_AA);
        } else {
            cv::ellipse(f, cv::Point(c.x + 4, c.y), cv::Size(7, 7), 0, -50, 50, col, 2, cv::LINE_AA);
            cv::ellipse(f, cv::Point(c.x + 4, c.y), cv::Size(12, 12), 0, -50, 50, col, 2, cv::LINE_AA);
        }
    }
    static void icoChevron(cv::Mat& f, cv::Point c, int s, const cv::Scalar& col, bool left) {
        int a = left ? s / 2 : -s / 2;
        std::vector<cv::Point> p = {{c.x + a, c.y - s}, {c.x - a, c.y}, {c.x + a, c.y + s}};
        cv::polylines(f, p, false, col, 3, cv::LINE_AA);
    }

    static std::string fmtTime(int sec) {
        if (sec < 0) sec = 0;
        char b[16];
        snprintf(b, sizeof(b), "%d:%02d", sec / 60, sec % 60);
        return std::string(b);
    }

    void fitLeft(cv::Mat& f, const std::string& s, int x, int y, int font, double sc, double mn,
                 int maxW, const cv::Scalar& col, int th = 1) {
        int bl = 0;
        while (sc > mn && txu::size(s, font, sc, th, &bl).width > maxW) sc -= 0.03;
        std::string q = s;
        bool cut = false;
        while (q.size() > 1 && txu::size(cut ? q + ".." : q, font, sc, th, &bl).width > maxW) {
            size_t i = q.size() - 1;
            while (i > 0 && (((unsigned char)q[i]) & 0xC0) == 0x80) --i;
            q.erase(i);
            cut = true;
        }
        if (cut) q += "..";
        txu::put(f, q, cv::Point(x, y), font, sc, col, th, cv::LINE_AA);
    }

    // ---- estado de reproduccion ----
    double elapsedSec() const {
        auto now = std::chrono::steady_clock::now();
        double e = std::chrono::duration<double>(now - playStart).count() - pausedTotal_;
        if (paused_) e -= std::chrono::duration<double>(now - pauseStart_).count();
        return std::max(0.0, e);
    }
    std::vector<int> buildVisible() const {
        std::vector<int> v;
        for (int i = 0; i < (int)songs.size(); ++i)
            if (!favTab_ || (i < (int)fav_.size() && fav_[i])) v.push_back(i);
        return v;
    }
    void stepSong(int d) {
        std::vector<int> v = buildVisible();
        if (v.empty()) return;
        int p = 0, n = (int)v.size();
        for (int i = 0; i < n; ++i) if (v[i] == selSong) { p = i; break; }
        selSong = v[((p + d) % n + n) % n];
    }
    void playStep(int d) {
        int n = (int)songs.size();
        if (n == 0) return;
        int base = (currentSong >= 0) ? currentSong : selSong;
        int idx = ((base + d) % n + n) % n;
        selSong = idx;
        playSong(idx);
    }
    void randomSong(bool play) {
        int n = (int)songs.size();
        if (n == 0) return;
        int idx = std::rand() % n;
        selSong = idx;
        if (play) playSong(idx);
    }
    void stopToList() {
        killAudio();
        viewMode_ = 0;
        paused_ = false;
        state = RadioState::SELECTING;
        currentSong = -1;
        currentSongIndex = -1;
    }
    void togglePause() {
        if (audioInitialized) {
            if (audioMutex_) {
                std::lock_guard<std::mutex> lk(*audioMutex_);
                if (paused_) Mix_ResumeMusic(); else Mix_PauseMusic();
            } else {
                if (paused_) Mix_ResumeMusic(); else Mix_PauseMusic();
            }
        }
        if (paused_) {
            pausedTotal_ += std::chrono::duration<double>(std::chrono::steady_clock::now() - pauseStart_).count();
            paused_ = false;
        } else {
            pauseStart_ = std::chrono::steady_clock::now();
            paused_ = true;
        }
    }
    void toggleMute() {
        if (!audioInitialized) return;
        auto doIt = [&]() {
            int v = Mix_VolumeMusic(-1);
            if (v > 0) { preMuteVol_ = v; Mix_VolumeMusic(0); }
            else Mix_VolumeMusic(preMuteVol_ > 0 ? preMuteVol_ : 64);
        };
        if (audioMutex_) { std::lock_guard<std::mutex> lk(*audioMutex_); doIt(); }
        else doIt();
    }

    // ---- marco comun: panel + barra lateral ----
    void drawPanelBase(cv::Mat& f) {
        cv::Rect P(58, 14, 732, 452);
        for (int i = 6; i >= 1; --i)
            kd::fillRound(f, cv::Rect(P.x + 2 - i, P.y + 8 - i, P.width + 2 * i - 4, P.height + 2 * i - 4),
                          30 + i, nDark(), 0.10);
        kd::fillRound(f, cv::Rect(10, 14, 120, 452), 30, nInk());
        kd::fillRound(f, P, 30, nPanel());
    }
    void drawSidebar(cv::Mat& f, bool inPlayer) {
        const cv::Scalar white(255, 255, 255), dim(110, 104, 104);
        std::time_t tn = std::time(nullptr);
        std::tm* tmv = std::localtime(&tn);
        char hh[8], mm[8];
        snprintf(hh, sizeof(hh), "%02d", tmv->tm_hour);
        snprintf(mm, sizeof(mm), "%02d", tmv->tm_min);
        auto cen = [&](const std::string& s, int cx, int y, double sc) {
            int bl = 0;
            cv::Size ts = txu::size(s, cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl);
            txu::put(f, s, cv::Point(cx - ts.width / 2, y), cv::FONT_HERSHEY_DUPLEX, sc, white, 1, cv::LINE_AA);
        };
        cen(hh, 34, 62, 0.62);
        cen(mm, 34, 88, 0.62);

        cv::Point bc(34, 152);
        backBtn = cv::Rect(10, 128, 48, 48);
        bool hb = backBtn.contains(hovPt_);
        cv::circle(f, bc, 18, hb ? cv::Scalar(86, 80, 80) : cv::Scalar(52, 48, 48), cv::FILLED, cv::LINE_AA);
        cv::circle(f, bc, 18, dim, 1, cv::LINE_AA);
        icoChevron(f, bc, 7, white, true);

        cv::Point dc(34, 214);
        cv::circle(f, dc, 18, cv::Scalar(52, 48, 48), cv::FILLED, cv::LINE_AA);
        cv::circle(f, dc, 18, inPlayer ? white : dim, inPlayer ? 2 : 1, cv::LINE_AA);
        drawMusicNote(f, cv::Point(dc.x - 3, dc.y + 6), white, 0.9f);

        cv::Point lc(34, 276);
        cv::circle(f, lc, 18, cv::Scalar(52, 48, 48), cv::FILLED, cv::LINE_AA);
        cv::circle(f, lc, 18, dim, 1, cv::LINE_AA);
        for (int k = -1; k <= 1; ++k)
            cv::line(f, {lc.x - 7, lc.y + k * 6}, {lc.x + 7, lc.y + k * 6}, white, 2, cv::LINE_AA);
    }

    // ---- tarjetas ----
    void drawBigCard(cv::Mat& f, const cv::Rect& r, const SongInfo& s, int idx, int total, double t, bool hov) {
        const cv::Scalar white(255, 255, 255);
        softShadow(f, r, 26);
        gradRoundR(f, r, 26, kd::mix(s.color, white, 0.20), s.color * 0.45);
        kd::strokeRound(f, r, 26, white, hov ? 4 : 2);
        char nb[16];
        snprintf(nb, sizeof(nb), "%02d", idx + 1);
        txu::put(f, nb, cv::Point(r.x + 18, r.y + 52), cv::FONT_HERSHEY_DUPLEX, 1.3, white, 2, cv::LINE_AA);
        txu::put(f, "/" + std::to_string(total), cv::Point(r.x + 20, r.y + 76), cv::FONT_HERSHEY_DUPLEX, 0.5,
                 kd::mix(white, s.color, 0.3), 1, cv::LINE_AA);
        cv::Point vc(r.x + r.width / 2, r.y + 114);
        blendCircle(f, vc, 82, white, 0.18);
        drawVinyl(f, vc, 72, s.color, t, true);
        cv::Point pc(r.x + 30, r.y + r.height - 34);
        cv::circle(f, pc, 15, white, 2, cv::LINE_AA);
        icoPlay(f, cv::Point(pc.x + 1, pc.y), 6, white);
        fitLeft(f, s.title, r.x + 54, r.y + r.height - 38, cv::FONT_HERSHEY_DUPLEX, 0.5, 0.3, r.width - 64, white);
        fitLeft(f, s.subtitle, r.x + 54, r.y + r.height - 20, cv::FONT_HERSHEY_DUPLEX, 0.38, 0.26, r.width - 64,
                kd::mix(white, s.color, 0.25));
        if (idx < (int)fav_.size() && fav_[idx]) heartShape(f, cv::Point(r.x + r.width - 26, r.y + 28), 9, white);
    }
    void drawSmallCard(cv::Mat& f, const cv::Rect& base, const SongInfo& s, int idx, double t, bool hov) {
        const cv::Scalar white(255, 255, 255);
        cv::Rect r = base;
        if (hov) r.y -= 3;
        softShadow(f, r, 22);
        gradRoundR(f, r, 22, kd::mix(s.color, white, 0.62), kd::mix(s.color, white, 0.30));
        kd::strokeRound(f, r, 22, white, 2);
        fitLeft(f, s.title, r.x + 10, r.y + 28, cv::FONT_HERSHEY_DUPLEX, 0.46, 0.3, r.width - 16, nInk());
        fitLeft(f, s.subtitle, r.x + 10, r.y + 46, cv::FONT_HERSHEY_DUPLEX, 0.34, 0.26, r.width - 16, kd::mix(nInk(), white, 0.4));
        drawVinyl(f, cv::Point(r.x + r.width / 2, r.y + 126), 34, s.color, t, false);
        char nb[16];
        snprintf(nb, sizeof(nb), "%02d", idx + 1);
        txu::put(f, nb, cv::Point(r.x + 12, r.y + r.height - 16), cv::FONT_HERSHEY_DUPLEX, 0.5, nInk(), 1, cv::LINE_AA);
        if (idx < (int)fav_.size() && fav_[idx]) heartShape(f, cv::Point(r.x + r.width - 18, r.y + r.height - 20), 7, nPink());
    }

    // ---- vista lista ----
    void renderSelector(cv::Mat& frame) {
        const int W = frame.cols;
        (void)W;
        const double t = kd::now();
        if (fav_.size() != songs.size()) fav_.assign(songs.size(), false);
        if (selSong >= 0 && selSong < (int)songs.size()) updateTheme(songs[selSong].color);
        frame.setTo(nBg());
        drawPanelBase(frame);
        drawSidebar(frame, false);

        std::vector<int> vis = buildVisible();
        if (!vis.empty() && std::find(vis.begin(), vis.end(), selSong) == vis.end()) selSong = vis[0];

        // pestanas
        {
            const std::string tm = eng() ? "Music" : "Música";
            const std::string tf = eng() ? "Favorites" : "Favoritas";
            int bl = 0;
            cv::Size s1 = txu::size(tm, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &bl);
            cv::Size s2 = txu::size(tf, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &bl);
            const int x1 = 96, x2 = 96 + s1.width + 28;
            tabMusic_ = cv::Rect(x1 - 6, 24, s1.width + 12, 40);
            tabFav_   = cv::Rect(x2 - 6, 24, s2.width + 12, 40);
            txu::put(frame, tm, cv::Point(x1, 52), cv::FONT_HERSHEY_DUPLEX, 0.6, favTab_ ? nGray() : nInk(), 1, cv::LINE_AA);
            txu::put(frame, tf, cv::Point(x2, 52), cv::FONT_HERSHEY_DUPLEX, 0.6, favTab_ ? nInk() : nGray(), 1, cv::LINE_AA);
            int ux = favTab_ ? x2 : x1, uw = favTab_ ? s2.width : s1.width;
            cv::line(frame, {ux, 60}, {ux + uw, 60}, nInk(), 2, cv::LINE_AA);
        }

        if (vis.empty()) {
            cardRects.clear(); cardSong_.clear();
            prevSongBtn = nextSongBtn = playBtn = queueBtn_ = cv::Rect();
            heartShape(frame, cv::Point(424, 200), 26, nDark());
            std::string m = eng() ? "No favorites yet" : "Aún no tienes favoritas";
            int bl = 0;
            cv::Size ts = txu::size(m, cv::FONT_HERSHEY_DUPLEX, 0.8, 1, &bl);
            txu::put(frame, m, cv::Point(424 - ts.width / 2, 280), cv::FONT_HERSHEY_DUPLEX, 0.8, nInk(), 1, cv::LINE_AA);
            return;
        }

        int pos = 0;
        for (int i = 0; i < (int)vis.size(); ++i) if (vis[i] == selSong) { pos = i; break; }

        // contador
        {
            cv::Rect pill(668, 26, 102, 28);
            kd::fillRound(frame, pill, 14, kd::mix(themeC_, cv::Scalar(255, 255, 255), 0.78));
            kd::strokeRound(frame, pill, 14, nLight(), 2);
            std::string c = std::to_string(pos + 1) + " / " + std::to_string(vis.size());
            int bl = 0;
            cv::Size ts = txu::size(c, cv::FONT_HERSHEY_DUPLEX, 0.45, 1, &bl);
            txu::put(frame, c, cv::Point(pill.x + (pill.width - ts.width) / 2, pill.y + 19),
                     cv::FONT_HERSHEY_DUPLEX, 0.45, nInk(), 1, cv::LINE_AA);
        }

        // tarjetas
        const int CY = 78, CH = 240;
        cardRects.clear(); cardSong_.clear();
        int x = 96;
        for (int k = 0; k < 3 && pos + k < (int)vis.size(); ++k) {
            int si = vis[pos + k];
            int w = (k == 0) ? 224 : 180;
            if (x + w > 784) break;
            cv::Rect r(x, CY, w, CH);
            cardRects.push_back(r);
            cardSong_.push_back(si);
            bool hov = r.contains(hovPt_);
            if (k == 0) drawBigCard(frame, r, songs[si], si, (int)songs.size(), t, hov);
            else        drawSmallCard(frame, r, songs[si], si, t, hov);
            x += w + 12;
        }

        // controles
        const int cyb = 392, R = 32;
        cv::Point pc(140, cyb), qc(216, cyb), nc(292, cyb);
        prevSongBtn = cv::Rect(pc.x - R, pc.y - R, 2 * R, 2 * R);
        playBtn     = cv::Rect(qc.x - R, qc.y - R, 2 * R, 2 * R);
        nextSongBtn = cv::Rect(nc.x - R, nc.y - R, 2 * R, 2 * R);
        neuCircle(frame, pc, R, prevSongBtn.contains(hovPt_)); icoPrev(frame, pc, 9, nInk());
        neuCircle(frame, qc, R, playBtn.contains(hovPt_));     icoPlay(frame, qc, 10, nInk());
        neuCircle(frame, nc, R, nextSongBtn.contains(hovPt_)); icoNext(frame, nc, 9, nInk());

        const SongInfo& cur = songs[selSong];
        fitLeft(frame, cur.title, 350, cyb - 2, cv::FONT_HERSHEY_DUPLEX, 0.72, 0.4, 330, nInk());
        fitLeft(frame, cur.subtitle, 350, cyb + 24, cv::FONT_HERSHEY_DUPLEX, 0.48, 0.3, 330, nGray());

        cv::Point qp(730, cyb);
        queueBtn_ = cv::Rect(qp.x - 24, qp.y - 24, 48, 48);
        if (queueBtn_.contains(hovPt_)) blendCircle(frame, qp, 24, nDark(), 0.25);
        icoQueue(frame, qp, nInk());
    }

    void drawArtVinyl(cv::Mat& f, const cv::Rect& r, const SongInfo& s, double t, double beat) {
        const cv::Scalar white(255, 255, 255);
        gradRoundR(f, r, 26, kd::mix(s.color, white, 0.55), kd::mix(s.color, white, 0.20));
        cv::Point c(r.x + r.width / 2, r.y + 108);
        for (int i = 0; i < 3; ++i) {
            double ph = std::fmod(t * beat * 0.25 + i / 3.0, 1.0);
            blendCircle(f, c, (int)(78 + ph * 26), kd::pal(i * 2 + 1), (1.0 - ph) * 0.5, 4);
        }
        drawVinyl(f, c, 72, s.color, t, true);
        for (int k = 0; k < 3; ++k) {
            double ang = t * 1.2 + k * 2.094;
            kd::sparkle(f, cv::Point((int)(c.x + 100 * std::cos(ang)), (int)(c.y + 90 * std::sin(ang))),
                        5 + (k % 2) * 3, kd::pal(k * 2 + 1));
        }
        const int BARS = 14, bw = 10, gap = 6;
        int ex = r.x + (r.width - (BARS * bw + (BARS - 1) * gap)) / 2;
        int baseY = r.y + r.height - 12;
        for (int i = 0; i < BARS; ++i) {
            double amp = std::abs(std::sin(t * beat * CV_PI + i * 0.45) * 0.5 +
                                  std::sin(t * beat * CV_PI * 0.6 + i * 0.22) * 0.3 +
                                  std::sin(t * 1.1 + i * 0.15) * 0.2);
            int h = 6 + (int)(30 * amp);
            kd::fillRound(f, cv::Rect(ex + i * (bw + gap), baseY - h, bw, h), 4, kd::pal(i));
        }
    }

    // ---- vista reproduciendo ----
    void renderNowPlaying(cv::Mat& frame) {
        if (currentSong < 0 || currentSong >= (int)songs.size()) return;
        if (fav_.size() != songs.size()) fav_.assign(songs.size(), false);
        const SongInfo& s = songs[currentSong];
        const double beat = s.beatHz;
        const double t = elapsedSec();
        const cv::Scalar white(255, 255, 255);

        updateTheme(s.color);
        frame.setTo(nBg());
        if (songDuration > 0.0 && !repeatOn_ && t >= songDuration) { playStep(+1); return; }
        if (viewMode_ == 1) { renderFullFace(frame, s, t, beat); return; }
        if (viewMode_ == 2) { renderVideoPlaceholder(frame); return; }
        drawPanelBase(frame);
        drawSidebar(frame, true);

        // cabecera
        chevBtn_ = cv::Rect(92, 40, 40, 44);
        if (chevBtn_.contains(hovPt_)) blendCircle(frame, cv::Point(110, 62), 20, nDark(), 0.25);
        icoChevron(frame, cv::Point(110, 62), 8, nInk(), true);
        fitLeft(frame, s.title, 146, 66, cv::FONT_HERSHEY_DUPLEX, 0.8, 0.4, 290, nInk());
        fitLeft(frame, s.subtitle, 146, 90, cv::FONT_HERSHEY_DUPLEX, 0.45, 0.3, 290, nGray());
        dotsBtn_ = cv::Rect(436, 42, 44, 40);
        if (dotsBtn_.contains(hovPt_)) blendCircle(frame, cv::Point(458, 62), 20, nDark(), 0.25);
        for (int k = -1; k <= 1; ++k) cv::circle(frame, cv::Point(458 + k * 9, 62), 2, nInk(), cv::FILLED, cv::LINE_AA);
        cv::line(frame, {146, 108}, {480, 108}, nDark(), 2, cv::LINE_AA);

        // fila de iconos
        const int iy = 156;
        const int ixs[4] = {166, 262, 358, 454};
        heartBtn_  = cv::Rect(ixs[0] - 22, iy - 22, 44, 44);
        repeatBtn_ = cv::Rect(ixs[1] - 22, iy - 22, 44, 44);
        queueBtn_  = cv::Rect(ixs[2] - 22, iy - 22, 44, 44);
        volBtn_    = cv::Rect(ixs[3] - 22, iy - 22, 44, 44);
        cv::Rect* hr[4] = {&heartBtn_, &repeatBtn_, &queueBtn_, &volBtn_};
        for (int k = 0; k < 4; ++k)
            if (hr[k]->contains(hovPt_)) blendCircle(frame, cv::Point(ixs[k], iy), 22, nDark(), 0.25);
        icoHeart(frame, cv::Point(ixs[0], iy), fav_[currentSong]);
        icoRepeat(frame, cv::Point(ixs[1], iy), repeatOn_ ? nInk() : nGray());
        icoQueue(frame, cv::Point(ixs[2], iy), nInk());
        bool vmuted = audioInitialized && Mix_VolumeMusic(-1) == 0;
        icoVolume(frame, cv::Point(ixs[3], iy), nInk(), vmuted);

        // progreso
        const double dur = songDuration;
        const double frac = (dur > 0.0) ? std::fmod(t, dur) / dur : 0.0;
        const int tx = 146, tw = 334, ty = 224;
        kd::fillRound(frame, cv::Rect(tx, ty - 2, tw, 4), 2, nDark());
        int fw = (int)(tw * frac);
        if (fw > 3) kd::fillRound(frame, cv::Rect(tx, ty - 2, fw, 4), 2, nInk());
        cv::Point kn(tx + fw, ty);
        blendCircle(frame, cv::Point(kn.x + 1, kn.y + 3), 9, nDark(), 0.5);
        cv::circle(frame, kn, 8, white, cv::FILLED, cv::LINE_AA);
        cv::circle(frame, kn, 8, nInk(), 2, cv::LINE_AA);
        {
            int cur = (int)((dur > 0.0) ? std::fmod(t, dur) : t);
            std::string a = fmtTime(cur);
            std::string b = (dur > 0.0) ? fmtTime((int)dur) : "--:--";
            int bl = 0;
            cv::Size sb = txu::size(b, cv::FONT_HERSHEY_DUPLEX, 0.42, 1, &bl);
            txu::put(frame, a, cv::Point(tx, ty + 28), cv::FONT_HERSHEY_DUPLEX, 0.42, nGray(), 1, cv::LINE_AA);
            txu::put(frame, b, cv::Point(tx + tw - sb.width, ty + 28), cv::FONT_HERSHEY_DUPLEX, 0.42, nGray(), 1, cv::LINE_AA);
        }

        // botones grandes
        const int by = 332;
        cv::Point pc(186, by), mc(311, by), nc(436, by);
        const int Rs = 36, Rb = 42;
        prevSongBtn = cv::Rect(pc.x - Rs, pc.y - Rs, 2 * Rs, 2 * Rs);
        ppBtn_      = cv::Rect(mc.x - Rb, mc.y - Rb, 2 * Rb, 2 * Rb);
        nextSongBtn = cv::Rect(nc.x - Rs, nc.y - Rs, 2 * Rs, 2 * Rs);
        neuCircle(frame, pc, Rs, prevSongBtn.contains(hovPt_)); icoPrev(frame, pc, 10, nInk());
        neuCircle(frame, mc, Rb, ppBtn_.contains(hovPt_));
        if (paused_) icoPlay(frame, cv::Point(mc.x + 1, mc.y), 12, nInk()); else icoPause(frame, mc, 12, nInk());
        neuCircle(frame, nc, Rs, nextSongBtn.contains(hovPt_)); icoNext(frame, nc, 10, nInk());

        // caratula / cara de Yaren
        artRect_ = cv::Rect(516, 118, 244, 244);
        softShadow(frame, artRect_, 26);
        if (true) {
            cv::Rect inner = artRect_;
            int Wt = (int)((0.9 * inner.width - 40) / 0.65);
            int Ht = Wt * inner.height / inner.width;
            cv::Mat tmp(Ht, Wt, CV_8UC3, cv::Scalar(0, 0, 0));
            drawAnimatedFace(tmp, s, t, beat);
            cv::Mat face;
            cv::resize(tmp, face, inner.size(), 0, 0, cv::INTER_LINEAR);
            cv::Mat mask = cv::Mat::zeros(inner.size(), CV_8UC1);
            std::vector<cv::Point> pp;
            kd::roundPath(pp, cv::Rect(0, 0, inner.width - 1, inner.height - 1), 26);
            cv::fillPoly(mask, pp, cv::Scalar(255), cv::LINE_AA);
            face.copyTo(frame(inner), mask);
        } else {
            drawArtVinyl(frame, artRect_, s, t, beat);
        }
        kd::strokeRound(frame, artRect_, 26, white, 3);
        viewBtn = artRect_;

        stopBtn = cv::Rect(0, 0, 0, 0);   // sin botón DETENER

        // ecualizador inferior
        const int BARS = 28, bw = 12, gap = 11;
        int ex = 96 + (664 - (BARS * bw + (BARS - 1) * gap)) / 2;
        const int baseY = 452;
        for (int i = 0; i < BARS; ++i) {
            double amp = std::abs(std::sin(t * beat * CV_PI + i * 0.45) * 0.5 +
                                  std::sin(t * beat * CV_PI * 0.6 + i * 0.22) * 0.3 +
                                  std::sin(t * 1.1 + i * 0.15) * 0.2);
            int h = 6 + (int)(34 * amp);
            kd::fillRound(frame, cv::Rect(ex + i * (bw + gap), baseY - h, bw, h), 5, s.color * 0.9);
        }
    }


    int viewMode_ { 0 };   // 0 normal, 1 cara completa, 2 video (futuro)
    void renderFullFace(cv::Mat& frame, const SongInfo& s, double t, double beat) {
        frame.setTo(cv::Scalar(0, 0, 0));
        drawAnimatedFace(frame, s, t, beat);
    }
    void renderVideoPlaceholder(cv::Mat& frame) {
        frame.setTo(cv::Scalar(0, 0, 0));
        // TODO: aqui va el video futuro (dibujar el frame del video a pantalla completa)
        kd::centered(frame, eng() ? "VIDEO COMING SOON" : "VIDEO PRÓXIMAMENTE", frame.cols, frame.rows / 2,
                     cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(255, 255, 255), 1);
    }
    void drawSongInfoBar(cv::Mat& frame, const SongInfo& s, double t, double beat) {
        int W = frame.cols;
        cv::Mat ov = frame.clone();
        cv::rectangle(ov, {0, 0, W, 62}, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::addWeighted(ov, 0.65, frame, 0.35, 0, frame);
        double p = 0.5 + 0.5 * std::sin(t * beat * CV_PI);
        cv::circle(frame, {22, 22}, 7, cv::Scalar(s.color[0]*p, s.color[1]*p, s.color[2]*p), cv::FILLED, cv::LINE_AA);
        txu::put(frame, eng() ? "NOW PLAYING" : "REPRODUCIENDO", {36, 27}, cv::FONT_HERSHEY_PLAIN, 0.82, cv::Scalar(140, 140, 155), 1, cv::LINE_AA);
        drawCenteredText(frame, s.title, W, 52, cv::FONT_HERSHEY_DUPLEX, 0.78, s.color, 2);
        drawCenteredText(frame, s.subtitle, W, 72, cv::FONT_HERSHEY_PLAIN, 0.82, cv::Scalar(90, 100, 120), 1);
        cv::line(frame, {W/2-250, 80}, {W/2+250, 80}, cv::Scalar(s.color[0]*0.3, s.color[1]*0.3, s.color[2]*0.3), 1, cv::LINE_AA);
    }

    void drawAnimatedFace(cv::Mat& frame, const SongInfo& s, double t, double beat) {
        int W = frame.cols, H = frame.rows;
        if (!faces_.empty()) {
            auto nowB = std::chrono::steady_clock::now();
            if (faces_.size() > 1 &&
                std::chrono::duration<double>(nowB - lastFaceSwitch).count() >= faceSwitchInterval) {
                currentFaceIndex = (currentFaceIndex + 1) % (int)faces_.size();
                lastFaceSwitch = nowB;
                isBlinking = false;
            }
            if (currentFaceIndex >= (int)faces_.size()) currentFaceIndex = 0;
            const FaceSet& cur = faces_[currentFaceIndex];
            if (!cur.closed.empty()) {
                double sinceBlink = std::chrono::duration<double>(nowB - lastBlinkTime).count();
                if (!isBlinking && sinceBlink > (2.5 + 1.0 * std::sin(t * 0.3))) {
                    isBlinking = true; blinkStart = nowB; lastBlinkTime = nowB;
                }
                if (isBlinking && std::chrono::duration<double>(nowB - blinkStart).count() > 0.3)
                    isBlinking = false;
            } else {
                isBlinking = false;
            }
            const cv::Mat& src = (isBlinking && !cur.closed.empty()) ? cur.closed : cur.open;
            int fH = H * 80 / 100;
            int fW = fH * src.cols / std::max(1, src.rows);
            if (fW > W * 90 / 100) { fW = W * 90 / 100; fH = fW * src.rows / std::max(1, src.cols); }
            cv::Mat rs;
            cv::resize(src, rs, cv::Size(fW, fH), 0, 0, cv::INTER_AREA);
            int fx = (W - fW) / 2 + (int)(std::cos(t * beat * CV_PI * 0.5) * 3.0);
            int fy = (H - fH) / 2 + (int)(std::sin(t * beat * CV_PI) * 5.0);
            for (int y = 0; y < fH; ++y) {
                int yy = y + fy;
                if (yy < 0 || yy >= H) continue;
                for (int x = 0; x < fW; ++x) {
                    int xx = x + fx;
                    if (xx < 0 || xx >= W) continue;
                    cv::Vec3b& bg = frame.at<cv::Vec3b>(yy, xx);
                    if (rs.channels() == 4) {
                        const cv::Vec4b& px = rs.at<cv::Vec4b>(y, x);
                        float a = px[3] / 255.f;
                        if (a > 0.f)
                            for (int c = 0; c < 3; ++c)
                                bg[c] = cv::saturate_cast<uchar>(px[c] * a + bg[c] * (1.f - a));
                    } else if (rs.channels() == 3) {
                        bg = rs.at<cv::Vec3b>(y, x);
                    }
                }
            }
            return;
        }

    }

    void drawPulseRings(cv::Mat& frame, const SongInfo& s, double t, double beat) {
        int W = frame.cols, H = frame.rows;
        for (int i = 0; i < 4; ++i) {
            double phase  = t * beat * CV_PI + i * CV_PI / 2;
            double radius = 80 + i * 55 + 20 * std::sin(phase);
            double alpha  = 0.12 * std::max(0.0, std::sin(phase));
            if (alpha < 0.01) continue;
            cv::Mat ov = frame.clone();
            cv::circle(ov, {W/2, H/2}, (int)radius, cv::Scalar(s.color[0], s.color[1], s.color[2]), 2, cv::LINE_AA);
            cv::addWeighted(ov, alpha, frame, 1-alpha, 0, frame);
        }
    }

    void drawEqualizer(cv::Mat& frame, const SongInfo& s, double t, double beat) {
        int W = frame.cols, H = frame.rows;
        int eqY  = H - 88, eqH  = 32, eqX  = 30, eqW  = W - 60;
        cv::rectangle(frame, {eqX - 4, eqY - eqH - 4, eqW + 8, eqH + 8}, cv::Scalar(4, 6, 14), cv::FILLED);
        cv::rectangle(frame, {eqX - 4, eqY - eqH - 4, eqW + 8, eqH + 8}, cv::Scalar(s.color[0]*0.25, s.color[1]*0.25, s.color[2]*0.25), 1, cv::LINE_AA);
        const int BARS = 48;
        int barW = eqW / BARS - 1;
        for (int i = 0; i < BARS; ++i) {
            double f1 = t * beat * CV_PI + i * 0.35;
            double f2 = t * beat * CV_PI * 0.6 + i * 0.22;
            double f3 = t * 1.1 + i * 0.15;
            double amp = std::abs(std::sin(f1) * 0.5 + std::sin(f2) * 0.3 + std::sin(f3) * 0.2);
            int bH = (int)(4 + (eqH - 4) * amp);
            int bx = eqX + i * (barW + 1), by = eqY - bH;
            float ratio = (float)i / BARS;
            cv::Scalar c((uchar)(s.color[0] * (0.4 + 0.6*ratio)), (uchar)(s.color[1] * (0.8 - 0.4*ratio)), (uchar)(s.color[2] * (0.6 + 0.3*std::sin(ratio * CV_PI))));
            cv::rectangle(frame, {bx, by, barW, bH}, c, cv::FILLED);
            cv::line(frame, {bx, by}, {bx + barW, by}, cv::Scalar(255,255,255), 1, cv::LINE_AA);
        }
    }

    void drawFloatingNotes(cv::Mat& frame, double t) {
        struct NotePos { int x, y; double phase; float sc; };
        static const NotePos notes[] = {
            {40, 90, 0.0, 1.0f}, {740, 110, 1.3, 0.8f},
            {100, 360, 2.1, 0.7f}, {680, 310, 3.4, 0.9f},
            {60, 220, 4.2, 0.6f}, {720, 200, 5.0, 0.75f},
            {380, 460, 1.8, 0.65f},
        };
        for (const auto& n : notes) {
            float alpha = 0.25f * (float)(0.5 + 0.5 * std::sin(t * 0.7 + n.phase));
            int   oy    = (int)(12 * std::sin(t * 0.4 + n.phase));
            cv::Mat ov  = frame.clone();
            drawMusicNote(ov, {n.x, n.y + oy}, cv::Scalar(200, 60, 255), n.sc);
            cv::addWeighted(ov, alpha, frame, 1-alpha, 0, frame);
        }
    }

    void drawMusicNote(cv::Mat& f, cv::Point pos, const cv::Scalar& c, float sc = 1.0f) {
        int r  = (int)(6  * sc), sh = (int)(16 * sc), fw = (int)(10 * sc);
        cv::ellipse(f, {pos.x, pos.y}, {r, (int)(r*0.75)}, -20, 0, 360, c, cv::FILLED, cv::LINE_AA);
        cv::line(f, {pos.x + r - 1, pos.y - 1}, {pos.x + r - 1, pos.y - sh}, c, (int)(2*sc), cv::LINE_AA);
        cv::line(f, {pos.x + r - 1, pos.y - sh}, {pos.x + r + fw - 1, pos.y - sh + (int)(7*sc)}, c, (int)(2*sc), cv::LINE_AA);
    }

    void drawFlatButton(cv::Mat& frame, const cv::Rect& r, const std::string& lbl, const cv::Scalar& accent, bool hov) {
        cv::Scalar bg   = hov ? cv::Scalar(accent[0]*0.20, accent[1]*0.20, accent[2]*0.20) : cv::Scalar(16, 20, 35);
        cv::Scalar bord = hov ? accent : cv::Scalar(accent[0]*0.40, accent[1]*0.40, accent[2]*0.40);
        cv::rectangle(frame, r, bg,   cv::FILLED);
        cv::rectangle(frame, r, bord, hov ? 2 : 1, cv::LINE_AA);
        drawTextInRect(frame, lbl, r, cv::FONT_HERSHEY_DUPLEX, 0.46, hov ? cv::Scalar(255,255,255) : cv::Scalar(170,175,190));
    }

    void overlayImg(cv::Mat& bg, const cv::Mat& fg) {
        if (fg.empty() || bg.empty()) return;
        cv::Mat src;
        if (fg.size() != bg.size()) cv::resize(fg, src, bg.size(), 0, 0, cv::INTER_AREA);
        else src = fg;
        if (src.channels() == 4) {
            for (int y = 0; y < bg.rows; ++y)
                for (int x = 0; x < bg.cols; ++x) {
                    if (y >= src.rows || x >= src.cols) continue;
                    const cv::Vec4b& px = src.at<cv::Vec4b>(y,x);
                    float a = px[3] / 255.f;
                    if (a > 0.f) {
                        cv::Vec3b& b = bg.at<cv::Vec3b>(y,x);
                        for (int c = 0; c < 3; ++c)
                            b[c] = cv::saturate_cast<uchar>(px[c]*a + b[c]*(1.f-a));
                    }
                }
        } else {
            src.copyTo(bg);
        }
    }
};

// =============================================================================
//  RoutinesApp
// =============================================================================
class RoutinesApp {
public:
    std::function<void()> onBack;
    std::function<void(std::string)> onLaunchSubprocess;
    std::vector<std::string> routines;
    int currentPage {0};
    cv::Rect prevPageBtn {0,0,0,0}, nextPageBtn {0,0,0,0}, backBtn {0,0,0,0};
    cv::Rect newBtn {0,0,0,0}, continueBtn {0,0,0,0};
    RadioState state { RadioState::SELECTING };
    bool hovBack{false}, hovNew{false}, hovContinue{false};
    bool hovPrev{false}, hovNext{false};
    int hovCard{-1};
    std::vector<cv::Rect> cardRects;
    bool* isEnglish = nullptr;
    bool eng() const { return isEnglish != nullptr && *isEnglish; }

    void launchRoutine(const std::string& /*scriptPath*/, const std::string& routineName) {
        state = RadioState::PLAYING;
        std::thread([this, routineName]() {
            std::string absCwd = std::filesystem::current_path().string();
            std::string absCmd = "python3 " + absCwd +
                                 "/src/YAREN2/yaren_movements/yaren_movements/" + routineName;
            int ret = std::system(absCmd.c_str());
            if (ret != 0) {
                RCLCPP_WARN(rclcpp::get_logger("RoutinesApp"),
                            "Rutina terminó con código %d: %s", ret, absCmd.c_str());
            }
            state = RadioState::SELECTING;
            refresh();
        }).detach();
    }

    void refresh() {
        routines.clear();
        std::string absCwd = std::filesystem::current_path().string();
        std::string dirPath = absCwd + "/src/YAREN2/yaren_movements/yaren_movements";
        std::vector<std::string> exclude = {
            "__init__.py", "yaren_dance_radio.py", "yaren_fullmovement.py",
            "yaren_movement.py", "yaren_rutinanueva.py", "yaren_rutina1.py"
        };
        if (fs::exists(dirPath)) {
            for (const auto& entry : fs::directory_iterator(dirPath)) {
                if (entry.path().extension() == ".py") {
                    std::string fn = entry.path().filename().string();
                    if (std::find(exclude.begin(), exclude.end(), fn) == exclude.end()) {
                        routines.push_back(fn);
                    }
                }
            }
        }
        std::sort(routines.begin(), routines.end(), std::greater<std::string>());
        currentPage = 0;
    }

    void render(cv::Mat& frame) {
        int W = frame.cols, H = frame.rows;
        kd::background(frame);
        if (state == RadioState::PLAYING) {
            kd::rainbowTitle(frame, eng() ? "PLAYING ROUTINE..." : "REPRODUCIENDO RUTINA...", W, H / 2 - 10, 1.0);
            kd::bouncingDots(frame, cv::Point(W / 2, H / 2 + 50), kd::now());
            return;
        }
        kd::rainbowTitle(frame, eng() ? "PERSONAL ROUTINES" : "RUTINAS PERSONALES", W, 44, 0.85);
        if (routines.empty()) {
            kd::centered(frame, eng() ? "No routine created yet, create one now."
                                      : "No existe ninguna rutina creada, créala ya.",
                         W, H / 2 - 30, cv::FONT_HERSHEY_DUPLEX, 0.7, kd::ink(), 1);
            continueBtn = {W / 2 - 120, H / 2 + 30, 240, 50};
            kd::button(frame, continueBtn, eng() ? "CONTINUE" : "CONTINUAR", cv::Scalar(90, 200, 90), hovContinue, true, 0.55);
            backBtn = {W / 2 - 120, H / 2 + 100, 240, 50};
            kd::button(frame, backBtn, eng() ? "EXIT" : "SALIR", cv::Scalar(250, 170, 60), hovBack, true, 0.55);
            newBtn = {0, 0, 0, 0};
            cardRects.clear();
        } else {
            continueBtn = {0, 0, 0, 0};
            const int COLS = 2;
            const int ITEMS_PER_PAGE = 4;
            const int CW = 300, CH = 100, G = 24, SY = 80;
            int totalPages = (routines.size() + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
            if (currentPage >= totalPages) currentPage = std::max(0, totalPages - 1);
            cardRects.clear();
            int startIdx = currentPage * ITEMS_PER_PAGE;
            int endIdx = std::min(startIdx + ITEMS_PER_PAGE, (int)routines.size());
            int idx = 0;
            for (int i = startIdx; i < endIdx; ++i) {
                int row = idx / COLS, col = idx % COLS;
                int rowItems = std::min(COLS, endIdx - startIdx - row * COLS);
                int rowW = rowItems * CW + (rowItems - 1) * G;
                int rowSX = (W - rowW) / 2;
                cv::Rect r{rowSX + col * (CW + G), SY + row * (CH + G), CW, CH};
                cardRects.push_back(r);
                bool hov = (hovCard == idx);
                cv::Scalar a = kd::pal(i);
                kd::fillRound(frame, cv::Rect(r.x, r.y + 5, r.width, r.height), 22, kd::shade());
                kd::fillRound(frame, r, 22, hov ? kd::mix(a, kd::white(), 0.85) : kd::white());
                kd::strokeRound(frame, r, 22, hov ? a : kd::mix(a, kd::white(), 0.45), hov ? 4 : 3);
                cv::Point dc(r.x + 52, r.y + r.height / 2);
                cv::circle(frame, {dc.x, dc.y + 3}, 30, kd::shade(), cv::FILLED, cv::LINE_AA);
                cv::circle(frame, dc, 30, a, cv::FILLED, cv::LINE_AA);
                cv::circle(frame, dc, 30, kd::white(), 3, cv::LINE_AA);
                std::vector<cv::Point> tri = { {dc.x - 8, dc.y - 12}, {dc.x - 8, dc.y + 12}, {dc.x + 13, dc.y} };
                cv::fillPoly(frame, tri, kd::white(), cv::LINE_AA);
                std::string niceName = routines[i];
                if (niceName.size() > 3) niceName = niceName.substr(0, niceName.size() - 3);
                double sc = 0.62; int bl = 0;
                while (sc > 0.32 && txu::size(niceName, cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl).width > r.width - 110) sc -= 0.03;
                cv::Size ts = txu::size(niceName, cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl);
                txu::put(frame, niceName, {r.x + 96, r.y + (r.height + ts.height) / 2},
                            cv::FONT_HERSHEY_DUPLEX, sc, kd::ink(), 1, cv::LINE_AA);
                idx++;
            }
            if (totalPages > 1) {
                int arrowSize = 48;
                prevPageBtn = {12, H / 2 - arrowSize / 2, arrowSize, arrowSize};
                nextPageBtn = {W - 12 - arrowSize, H / 2 - arrowSize / 2, arrowSize, arrowSize};
                if (currentPage > 0) kd::roundArrow(frame, prevPageBtn, true, cv::Scalar(230, 110, 200), hovPrev);
                else prevPageBtn = {0, 0, 0, 0};
                if (currentPage < totalPages - 1) kd::roundArrow(frame, nextPageBtn, false, cv::Scalar(230, 110, 200), hovNext);
                else nextPageBtn = {0, 0, 0, 0};
            } else {
                prevPageBtn = {0, 0, 0, 0}; nextPageBtn = {0, 0, 0, 0};
            }
            newBtn = {W / 2 - 170, H - 60, 160, 42};
            kd::button(frame, newBtn, eng() ? "NEW ROUTINE" : "NUEVA RUTINA", cv::Scalar(90, 200, 90), hovNew, true, 0.48);
            backBtn = {W / 2 + 10, H - 60, 160, 42};
            kd::button(frame, backBtn, eng() ? "BACK" : "VOLVER", cv::Scalar(250, 170, 60), hovBack, true, 0.48);
        }
    }

    void handleMouse(int ev, int x, int y) {
        cv::Point pt{x, y};
        if (ev == cv::EVENT_MOUSEMOVE) {
            hovBack = backBtn.contains(pt);
            hovContinue = continueBtn.contains(pt);
            hovNew = newBtn.contains(pt);
            hovPrev = prevPageBtn.contains(pt);
            hovNext = nextPageBtn.contains(pt);
            hovCard = -1;
            for(size_t i=0; i<cardRects.size(); ++i){
                if(cardRects[i].contains(pt)) hovCard = i;
            }
            return;
        }
        if (ev == cv::EVENT_LBUTTONDOWN) {
            if (hovBack && onBack) onBack();
            if (hovContinue || hovNew) {
                if (onLaunchSubprocess) {
                    std::system("for pid in $(ps aux | grep -E 'yaren_rutinanueva' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done");
                    std::string absCwd = std::filesystem::current_path().string();
                    onLaunchSubprocess("python3 " + absCwd +
                                       "/src/YAREN2/yaren_movements/yaren_movements/yaren_rutinanueva.py &");
                }
            }
            if (hovPrev && currentPage > 0) currentPage--;
            if (hovNext) currentPage++;
            if (hovCard >= 0) {
                int realIdx = currentPage * 4 + hovCard;
                if (realIdx < (int)routines.size()) {
                    launchRoutine("...", routines[realIdx]);
                }
            }
        }
    }

    void drawNavArrow(cv::Mat& frame, const cv::Rect& r, bool left, bool hovered) {
        if(r.area() == 0) return;
        cv::Scalar color = hovered ? cv::Scalar(255, 100, 255) : cv::Scalar(150, 80, 180);
        cv::rectangle(frame, r, cv::Scalar(20, 15, 40), cv::FILLED);
        cv::rectangle(frame, r, color, hovered ? 2 : 1, cv::LINE_AA);
        int cx = r.x + r.width/2, cy = r.y + r.height/2, size = 15;
        std::vector<cv::Point> pts(3);
        if (left) pts = {{cx + size/2, cy - size}, {cx + size/2, cy + size}, {cx - size/2, cy}};
        else pts = {{cx - size/2, cy - size}, {cx - size/2, cy + size}, {cx + size/2, cy}};
        cv::fillPoly(frame, pts, color);
    }

    void drawFlatButton(cv::Mat& frame, const cv::Rect& r, const std::string& lbl, const cv::Scalar& accent, bool hov) {
        if (r.area() == 0) return;
        cv::Scalar bg   = hov ? cv::Scalar(accent[0]*0.20, accent[1]*0.20, accent[2]*0.20) : cv::Scalar(16, 20, 35);
        cv::Scalar bord = hov ? accent : cv::Scalar(accent[0]*0.40, accent[1]*0.40, accent[2]*0.40);
        cv::rectangle(frame, r, bg,   cv::FILLED);
        cv::rectangle(frame, r, bord, hov ? 2 : 1, cv::LINE_AA);
        drawTextInRect(frame, lbl, r, cv::FONT_HERSHEY_DUPLEX, 0.46, hov ? cv::Scalar(255,255,255) : cv::Scalar(170,175,190));
    }
};
// =============================================================================
//  OnScreenKeyboard — teclado QWERTY táctil para la pantalla
// =============================================================================
class OnScreenKeyboard {
public:
    std::string text;           // texto ingresado
    bool visible { false };
    std::string prompt;         // texto label encima del campo

    void show(const std::string& promptText, const std::string& initial = "") {
        prompt  = promptText;
        text    = initial;
        visible = true;
        showNumbers_ = false;
        shiftActive_ = false;
        capsLock_    = false;
        hovKey_  = -1;
    }
    void hide() { visible = false; }

    // Retorna true si el usuario presionó OK/Enter
    bool handleMouse(int ev, int x, int y) {
        if (!visible) return false;
        hovPt_ = {x, y};
        if (ev == cv::EVENT_MOUSEMOVE) {
            hovKey_ = findKey(x, y);
            return false;
        }
        if (ev != cv::EVENT_LBUTTONDOWN) return false;
        // Botón backspace
        if (btnBackspace_.contains({x,y})) {
            if (!text.empty()) text.pop_back();
            return false;
        }
        // Botón toggle numérico
        if (btnNumToggle_.contains({x,y})) { showNumbers_ = !showNumbers_; return false; }
        // Botón OK
        if (btnOk_.contains({x,y})) { visible = false; return true; }
        // Botón cancelar
        if (btnCancel_.contains({x,y})) { visible = false; text = ""; return false; }
        // Botón espacio
        if (btnSpace_.contains({x,y})) { text += ' '; return false; }
        // Botón mayúsculas
        if (btnShift_.contains({x,y})) {
            if (!shiftActive_)      { shiftActive_ = true;  capsLock_ = false; }   // 1 clic: una mayuscula
            else if (!capsLock_)    { capsLock_ = true; }                          // 2 clics: mayusculas fijas
            else                    { shiftActive_ = false; capsLock_ = false; }   // 3 clics: normal
            return false;
        }
        // Teclas normales
        int ki = findKey(x, y);
        if (ki >= 0 && ki < (int)keyRects_.size()) {
            std::string ch = getCurrentKeys()[ki];
            if (shiftActive_) {
                if (!ch.empty()) ch[0] = (char)std::toupper((unsigned char)ch[0]);
                if (!capsLock_) shiftActive_ = false;
            }
            text += ch;
        }
        return false;
    }

    void render(cv::Mat& frame) {
        if (!visible) return;
        int W = frame.cols, H = frame.rows;
        const cv::Scalar white = kd::white();

        {
            cv::Mat ov = frame.clone();
            ov.setTo(kd::cream());
            cv::addWeighted(ov, 0.93, frame, 0.07, 0, frame);
        }

        // Paleta del teclado (BGR): teclas durazno, laterales rojos, detalles rosa/cyan/verde
        const cv::Scalar panelC(255, 255, 255),  borderC(150, 190, 225);          // panel blanco + borde tan
        const cv::Scalar keyC(229, 243, 255),    keyEdge(150, 190, 225), brown(52, 48, 46); // teclas durazno claro, texto ink
        const cv::Scalar redC(95, 140, 190),     redEdge(60, 100, 150);            // shift / borrar -> caramelo
        const cv::Scalar greenC(40, 160, 255),   greenEdge(20, 105, 205);          // OK / caps -> naranja
        const cv::Scalar cyanC(95, 140, 190),    cyanEdge(60, 100, 150);           // cancelar -> caramelo
        const cv::Scalar pinkC(95, 140, 190),    pinkEdge(60, 100, 150);           // 123 -> caramelo
        const cv::Scalar goldC(90, 195, 255),    goldEdge(40, 160, 255);           // shift activo -> naranja claro

        // Panel
        const int KBW = 764, KBH = 340;
        const int KBX = (W - KBW) / 2;
        const int KBY = H - KBH - 6;
        kd::fillRound(frame, cv::Rect(KBX, KBY + 5, KBW, KBH), 28, kd::shade());
        kd::fillRound(frame, cv::Rect(KBX, KBY, KBW, KBH), 28, panelC);
        kd::strokeRound(frame, cv::Rect(KBX, KBY, KBW, KBH), 28, borderC, 4);

        const int gap = 8, keyH = 52;
        const int ix = KBX + 18, iw = KBW - 36;

        // Texto guia y campo
        txu::put(frame, prompt, {ix + 6, KBY + 28}, cv::FONT_HERSHEY_DUPLEX, 0.5, kd::gray(), 1, cv::LINE_AA);
        cv::Rect fieldRect{ix, KBY + 40, iw, 40};
        kd::fillRound(frame, fieldRect, 14, white);
        kd::strokeRound(frame, fieldRect, 14, cyanC, 2);
        double tnow = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        bool cur = std::fmod(tnow * 2.0, 1.0) < 0.5;
        int fy = fieldRect.y + fieldRect.height / 2;
        if (isPassword_) {
            int maxDots = (fieldRect.width - 50) / 22;
            int n = (int)text.size();
            int start = std::max(0, n - maxDots);
            int x = fieldRect.x + 24;
            for (int i = start; i < n; ++i) { cv::circle(frame, {x, fy}, 6, brown, cv::FILLED, cv::LINE_AA); x += 22; }
            if (cur) cv::line(frame, {x, fy - 12}, {x, fy + 12}, brown, 2, cv::LINE_AA);
        } else {
            std::string shown = text;
            int blF = 0;
            while (shown.size() > 1 && txu::size(shown, cv::FONT_HERSHEY_DUPLEX, 0.8, 1, &blF).width > fieldRect.width - 50)
                shown.erase(0, 1);
            cv::Size ts = txu::size(shown, cv::FONT_HERSHEY_DUPLEX, 0.8, 1, &blF);
            txu::put(frame, shown, {fieldRect.x + 16, fy + ts.height / 2}, cv::FONT_HERSHEY_DUPLEX, 0.8, brown, 1, cv::LINE_AA);
            if (cur) cv::line(frame, {fieldRect.x + 20 + ts.width, fy - 12}, {fieldRect.x + 20 + ts.width, fy + 12}, brown, 2, cv::LINE_AA);
        }

        const std::vector<std::vector<std::string>> ROWS_ALPHA = {
            {"q","w","e","r","t","y","u","i","o","p"},
            {"a","s","d","f","g","h","j","k","l"},
            {"z","x","c","v","b","n","m"},
            {}
        };
        const std::vector<std::vector<std::string>> ROWS_NUM = {
            {"1","2","3","4","5","6","7","8","9","0"},
            {"-","/",":",";","(",")","$","&","@","\""},
            {".","_","#","!","?","=","+","<",">"},
            {}
        };
        auto& ROWS = showNumbers_ ? ROWS_NUM : ROWS_ALPHA;

        auto drawKey = [&](const cv::Rect& r, const cv::Scalar& fill, const cv::Scalar& edge, bool hov) {
            cv::Scalar f = hov ? kd::mix(fill, white, 0.25) : fill;
            kd::fillRound(frame, cv::Rect(r.x, r.y + 5, r.width, r.height), 14, edge);
            kd::fillRound(frame, r, 14, f);
            kd::fillRound(frame, cv::Rect(r.x + 10, r.y + 5, std::max(4, r.width - 20), 7), 3, white, 0.5);
            kd::strokeRound(frame, r, 14, edge, 2);
        };
        auto label = [&](const cv::Rect& r, const std::string& s, double sc, const cv::Scalar& col) {
            int bl = 0;
            cv::Size ts = txu::size(s, cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl);
            txu::put(frame, s, {r.x + (r.width - ts.width) / 2, r.y + (r.height + ts.height) / 2 + 1},
                        cv::FONT_HERSHEY_DUPLEX, sc, col, 1, cv::LINE_AA);
        };
        auto letterKey = [&](const cv::Rect& kr, const std::string& ch) {
            keyRects_.push_back(kr);
            int idx = (int)keyRects_.size() - 1;
            drawKey(kr, keyC, keyEdge, idx == hovKey_);
            cv::circle(frame, {kr.x + 11, kr.y + 13}, 3, (idx % 2) ? greenC : pinkC, cv::FILLED, cv::LINE_AA);
            std::string lb = ch;
            if (!showNumbers_ && shiftActive_ && !lb.empty()) lb[0] = (char)std::toupper((unsigned char)lb[0]);
            label(kr, lb, 0.85, brown);
        };

        keyRects_.clear();
        const int keyW = (iw - 9 * gap) / 10;
        int y = KBY + 94;

        // Filas 1 y 2 (centradas)
        for (int r = 0; r < 2; ++r) {
            const auto& row = ROWS[r];
            int n = (int)row.size();
            int rowW = n * keyW + (n - 1) * gap;
            int x0 = ix + (iw - rowW) / 2;
            for (int c = 0; c < n; ++c)
                letterKey(cv::Rect(x0 + c * (keyW + gap), y, keyW, keyH), row[c]);
            y += keyH + gap;
        }

        // Fila 3: SHIFT (rojo) + letras + borrar (rojo)
        {
            const int sideW = 96;
            const auto& row = ROWS[2];
            int n = (int)row.size();
            bool hasShift = !showNumbers_;
            int slots = n + 1 + (hasShift ? 1 : 0);
            int k3 = (iw - sideW - (hasShift ? sideW : 0) - (slots - 1) * gap) / n;
            int x = ix;
            if (hasShift) {
                btnShift_ = cv::Rect(x, y, sideW, keyH);
                drawKey(btnShift_,
                        capsLock_ ? greenC : (shiftActive_ ? goldC : redC),
                        capsLock_ ? greenEdge : (shiftActive_ ? goldEdge : redEdge),
                        btnShift_.contains(hovPt_));
                cv::Point c(btnShift_.x + sideW / 2, btnShift_.y + keyH / 2 - 2);
                drawArrow(frame, c, true, white, 10);
                cv::rectangle(frame, cv::Rect(c.x - 4, c.y + 8, 8, 8), white, cv::FILLED, cv::LINE_AA);
                if (capsLock_) cv::line(frame, {c.x - 12, c.y + 22}, {c.x + 12, c.y + 22}, white, 3, cv::LINE_AA);
                x += sideW + gap;
            } else {
                btnShift_ = cv::Rect(0, 0, 0, 0);
            }
            for (int c = 0; c < n; ++c)
                letterKey(cv::Rect(x + c * (k3 + gap), y, k3, keyH), row[c]);

            btnBackspace_ = cv::Rect(ix + iw - sideW, y, sideW, keyH);
            drawKey(btnBackspace_, redC, redEdge, btnBackspace_.contains(hovPt_));
            cv::Point bc(btnBackspace_.x + sideW / 2, btnBackspace_.y + keyH / 2);
            std::vector<cv::Point> pent = {{bc.x - 18, bc.y}, {bc.x - 7, bc.y - 11}, {bc.x + 17, bc.y - 11},
                                           {bc.x + 17, bc.y + 11}, {bc.x - 7, bc.y + 11}};
            cv::polylines(frame, pent, true, white, 2, cv::LINE_AA);
            cv::line(frame, {bc.x - 1, bc.y - 5}, {bc.x + 9, bc.y + 5}, white, 2, cv::LINE_AA);
            cv::line(frame, {bc.x + 9, bc.y - 5}, {bc.x - 1, bc.y + 5}, white, 2, cv::LINE_AA);
            y += keyH + gap;
        }

        // Fila inferior: 123 (rosa) | CANCELAR (cyan) | ESPACIO (durazno) | CONECTAR (verde)
        {
            int bx = ix;
            btnNumToggle_ = cv::Rect(bx, y, 96, keyH);  bx += 96 + gap;
            btnCancel_    = cv::Rect(bx, y, 124, keyH); bx += 124 + gap;
            const int okW = 150;
            int spW = iw - (bx - ix) - gap - okW;
            btnSpace_ = cv::Rect(bx, y, spW, keyH);     bx += spW + gap;
            btnOk_    = cv::Rect(bx, y, okW, keyH);

            drawKey(btnNumToggle_, pinkC, pinkEdge, btnNumToggle_.contains(hovPt_));
            label(btnNumToggle_, showNumbers_ ? "ABC" : "123", 0.7, white);
            drawKey(btnCancel_, cyanC, cyanEdge, btnCancel_.contains(hovPt_));
            label(btnCancel_, "CANCELAR", 0.55, white);
            drawKey(btnSpace_, keyC, keyEdge, btnSpace_.contains(hovPt_));
            label(btnSpace_, "ESPACIO", 0.6, brown);
            drawKey(btnOk_, greenC, greenEdge, btnOk_.contains(hovPt_));
            label(btnOk_, "CONECTAR", 0.58, white);
        }
    }

    bool isPassword_ { true };  // mostrar *** en el campo

private:
    bool showNumbers_ { false };
    bool shiftActive_ { false };
    bool capsLock_    { false };
    int  hovKey_      { -1 };
    cv::Point hovPt_  { 0, 0 };
    std::vector<cv::Rect> keyRects_;
    cv::Rect btnBackspace_{0,0,0,0}, btnNumToggle_{0,0,0,0};
    cv::Rect btnOk_{0,0,0,0}, btnCancel_{0,0,0,0};
    cv::Rect btnSpace_{0,0,0,0}, btnShift_{0,0,0,0};

    const std::vector<std::string>& getCurrentKeys() {
        static const std::vector<std::string> ALPHA = {
            "q","w","e","r","t","y","u","i","o","p",
            "a","s","d","f","g","h","j","k","l",
            "z","x","c","v","b","n","m"
        };
        static const std::vector<std::string> NUM = {
            "1","2","3","4","5","6","7","8","9","0",
            "-","/",":",";","(",")","$","&","@","\"",
            ".","_","#","!","?","=","+","<",">"
        };
        return showNumbers_ ? NUM : ALPHA;
    }

    int findKey(int x, int y) {
        for (int i=0;i<(int)keyRects_.size();++i)
            if (keyRects_[i].contains({x,y})) return i;
        return -1;
    }
};

// =============================================================================
//  WifiSetupScreen — versión asíncrona con validación de Internet
// =============================================================================
struct WifiNetwork {
    std::string ssid;
    bool saved { false };
    bool secured { true };   // asumir WPA por defecto
};

class WifiSetupScreen {
public:
    std::function<void()>  onConnected;
    std::function<void()>  onSkip;
    bool* isEnglish = nullptr;
    bool eng() const { return isEnglish && *isEnglish; }

    void refresh() {
        if (is_refreshing_.load()) return;
        
        is_refreshing_ = true;
        statusMsg_   = "";
        connecting_  = false;
        selectedIdx_ = -1;
        scroll_      = 0;
        keyboard_.hide();

        std::thread([this]() {
            std::vector<WifiNetwork> temp_networks;

            FILE* p = popen("nmcli -t -f NAME,TYPE connection show 2>/dev/null", "r");
            if (p) {
                char buf[256];
                while (fgets(buf, sizeof(buf), p)) {
                    std::string line(buf);
                    while (!line.empty() && (line.back()=='\n'||line.back()=='\r')) line.pop_back();
                    if (line.find(":802-11-wireless") != std::string::npos) {
                        std::string ssid = line.substr(0, line.find(':'));
                        if (!ssid.empty()) temp_networks.push_back({ssid, true, true});
                    }
                }
                pclose(p);
            }

            struct ScanEntry { std::string ssid; std::string security; };
            std::vector<ScanEntry> scanned;

            FILE* p2 = popen("nmcli --terse --fields SSID,SECURITY dev wifi list 2>/dev/null", "r");
            if (p2) {
                char buf[512];
                while (fgets(buf, sizeof(buf), p2)) {
                    std::string line(buf);
                    while (!line.empty() && (line.back()=='\n'||line.back()=='\r')) line.pop_back();
                    if (line.empty()) continue;

                    std::vector<std::string> parts;
                    std::string token;
                    for (size_t i = 0; i < line.size(); ++i) {
                        if (line[i] == '\\' && i+1 < line.size() && line[i+1] == ':') {
                            token += ':';
                            ++i;
                        } else if (line[i] == ':') {
                            parts.push_back(token);
                            token.clear();
                        } else {
                            token += line[i];
                        }
                    }
                    parts.push_back(token);

                    if (parts.size() < 1) continue;
                    std::string ssid = parts[0];
                    std::string sec  = parts.size() > 1 ? parts[1] : "";
                    if (ssid.empty() || ssid == "--") continue;

                    scanned.push_back({ssid, sec});
                }
                pclose(p2);
            }

            for (auto& e : scanned) {
                bool alreadyIn = false;
                for (auto& n : temp_networks) {
                    if (n.ssid == e.ssid) { alreadyIn = true; break; }
                }
                if (!alreadyIn) {
                    bool sec = !(e.security.empty() || e.security == "--");
                    temp_networks.push_back({e.ssid, false, sec});
                }
            }

            {
                std::lock_guard<std::mutex> lock(networks_mutex_);
                networks_ = temp_networks;
            }
            
            is_refreshing_ = false;
        }).detach();
    }

    void render(cv::Mat& frame) {
        if (frame.empty()) return;
        renderMain(frame);
        if (keyboard_.visible) keyboard_.render(frame);
    }

    void handleMouse(int ev, int x, int y) {
        if (keyboard_.visible) {
            bool confirmed = keyboard_.handleMouse(ev, x, y);
            if (confirmed && !keyboard_.text.empty()) {
                connectWithPassword(pendingSsid_, keyboard_.text);
            }
            return;
        }
        hovPt_ = {x,y};
        if (ev != cv::EVENT_LBUTTONDOWN) return;

        if (btnSkip_.contains({x,y})) { if(onSkip) onSkip(); return; }
        if (is_refreshing_.load() || connecting_) return;

        if (btnScrollUp_.contains({x,y})  && btnScrollUp_.area()>0)  { scroll_ = std::max(0, scroll_ - 1); return; }
        if (btnScrollDown_.contains({x,y})&& btnScrollDown_.area()>0){ scroll_ += 1; return; }
        for (auto& [r,idx] : rowRects_)
            if (r.contains({x,y})) { selectedIdx_=idx; return; }
        if (btnRefresh_.contains({x,y})) { refresh(); return; }
        if (btnConnect_.contains({x,y}) && selectedIdx_>=0) {
            initiateConnect();
        }
    }

private:
    std::vector<WifiNetwork> networks_;
    std::mutex networks_mutex_;
    std::atomic<bool> is_refreshing_{false};

    int  selectedIdx_ { -1 };
    int  scroll_      { 0 };
    bool connecting_  { false };
    std::string statusMsg_;
    std::string pendingSsid_;
    cv::Point hovPt_ { 0,0 };
    cv::Rect btnConnect_    {0,0,0,0};
    cv::Rect btnRefresh_    {0,0,0,0};
    cv::Rect btnSkip_       {0,0,0,0};
    cv::Rect btnScrollUp_   {0,0,0,0};
    cv::Rect btnScrollDown_ {0,0,0,0};
    struct RowEntry { cv::Rect r; int idx; };
    std::vector<RowEntry> rowRects_;
    OnScreenKeyboard keyboard_;

    // NUEVO: Función para comprobar si hay internet de verdad
    bool checkInternetLocal() {
        // Intenta 2 veces con curl (timeout de 3s). Si hay internet retorna true.
        for (int i = 0; i < 2; ++i) {
            int ret = std::system("curl -s -I -m 3 https://api.groq.com > /dev/null 2>&1");
            if (ret == 0) return true;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        return false;
    }

    void initiateConnect() {
        WifiNetwork net;
        {
            std::lock_guard<std::mutex> lock(networks_mutex_);
            if (selectedIdx_<0 || selectedIdx_>=(int)networks_.size()) return;
            net = networks_[selectedIdx_];
        }
        
        pendingSsid_ = net.ssid;

        if (net.saved) {
            connectSaved(net.ssid);
        } else if (!net.secured) {
            connectOpen(net.ssid);
        } else {
            keyboard_.isPassword_ = true;
            keyboard_.show((eng() ? "Password for: " : "Clave para: ") + net.ssid, "");
        }
    }

    void connectSaved(const std::string& ssid) {
        connecting_  = true;
        statusMsg_   = eng() ? "Connecting..." : "Conectando...";
        std::thread([this,ssid](){
            std::string cmd = "nmcli connection up '" + ssid + "' 2>/dev/null";
            int ret = std::system(cmd.c_str());
            if (ret == 0) {
                statusMsg_ = eng() ? "Verifying internet..." : "Verificando internet...";
                if (checkInternetLocal()) {
                    statusMsg_ = eng() ? "Connected!" : "Conectado!";
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    if (onConnected) onConnected();
                } else {
                    statusMsg_ = eng() ? "Connected, but NO INTERNET." : "Conectada, pero SIN INTERNET.";
                    std::this_thread::sleep_for(std::chrono::seconds(4));
                    statusMsg_ = "";
                }
            } else {
                statusMsg_ = eng() ? "Failed. Try reconnecting." : "Error al conectar.";
                std::this_thread::sleep_for(std::chrono::seconds(3));
                statusMsg_ = "";
            }
            connecting_ = false;
        }).detach();
    }

    void connectOpen(const std::string& ssid) {
        connecting_  = true;
        statusMsg_   = eng() ? "Connecting to open network..." : "Conectando a red abierta...";
        std::thread([this,ssid](){
            std::string cmd = "nmcli device wifi connect '" + ssid + "' 2>/dev/null";
            int ret = std::system(cmd.c_str());
            if (ret == 0) {
                statusMsg_ = eng() ? "Verifying internet..." : "Verificando internet...";
                if (checkInternetLocal()) {
                    statusMsg_ = eng() ? "Connected!" : "Conectado!";
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    if (onConnected) onConnected();
                } else {
                    statusMsg_ = eng() ? "Connected, but NO INTERNET." : "Conectada, pero SIN INTERNET.";
                    std::this_thread::sleep_for(std::chrono::seconds(4));
                    statusMsg_ = "";
                }
            } else {
                statusMsg_ = eng() ? "Failed to connect." : "Error al conectar.";
                std::this_thread::sleep_for(std::chrono::seconds(3));
                statusMsg_ = "";
            }
            connecting_ = false;
        }).detach();
    }

    void connectWithPassword(const std::string& ssid, const std::string& password) {
        connecting_  = true;
        statusMsg_   = eng() ? "Connecting..." : "Conectando...";
        
        std::thread([this, ssid, password]() {
            std::string safePwd = password;
            size_t pos = 0;
            while ((pos = safePwd.find("'", pos)) != std::string::npos) {
                safePwd.replace(pos, 1, "'\\''");
                pos += 4;
            }
            
            std::string cmd = "nmcli device wifi connect '" + ssid +
                              "' password '" + safePwd + "' > /tmp/yaren_wifi_err.txt 2>&1";
                              
            int ret = std::system(cmd.c_str());
            
            if (ret == 0) {
                statusMsg_ = eng() ? "Verifying internet..." : "Verificando internet...";
                
                if (checkInternetLocal()) {
                    statusMsg_ = eng() ? "Connected!" : "Conectado!";
                    {
                        std::lock_guard<std::mutex> lock(networks_mutex_);
                        for (auto& n : networks_)
                            if (n.ssid == ssid) { n.saved = true; break; }
                    }
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    if (onConnected) onConnected();
                } else {
                    statusMsg_ = eng() ? "Connected, but NO INTERNET." : "Conectada, pero SIN INTERNET.";
                    std::this_thread::sleep_for(std::chrono::seconds(4));
                    statusMsg_ = "";
                }
                connecting_ = false;
            } else {
                std::string errMsg = eng() ? "Wrong password." : "Clave incorrecta.";
                FILE* f = fopen("/tmp/yaren_wifi_err.txt", "r");
                if (f) {
                    char buf[256] = {};
                    if (fgets(buf, sizeof(buf), f)) {
                        std::string raw(buf);
                        if (raw.find("Secrets were required") != std::string::npos ||
                            raw.find("password") != std::string::npos ||
                            raw.find("authentication") != std::string::npos) {
                            errMsg = eng() ? "Wrong password, try again." : "Clave incorrecta, intenta de nuevo.";
                        } else if (raw.find("not found") != std::string::npos) {
                            errMsg = eng() ? "Network not found." : "Red no encontrada.";
                        }
                    }
                    fclose(f);
                }
                
                statusMsg_ = errMsg;
                connecting_ = false;
                
                std::this_thread::sleep_for(std::chrono::seconds(4));
                if (statusMsg_ == errMsg) {
                    statusMsg_ = "";
                }
            }
        }).detach();
    }

    static constexpr int kRows = 4;
    static cv::Scalar orange()   { return cv::Scalar(40, 160, 255); }
    static cv::Scalar orangeDk() { return cv::Scalar(20, 105, 205); }
    static cv::Scalar caramel()  { return cv::Scalar(95, 140, 190); }
    static cv::Scalar tan()      { return cv::Scalar(150, 190, 225); }

    void roundBtn(cv::Mat& f, const cv::Rect& r, bool up, bool enabled) {
        bool hov = r.contains(hovPt_) && enabled;
        cv::Point c(r.x + r.width / 2, r.y + r.height / 2);
        int R = std::min(r.width, r.height) / 2;
        cv::Scalar a = enabled ? orange() : cv::Scalar(205, 210, 214);
        cv::circle(f, {c.x, c.y + 4}, R, kd::shade(), cv::FILLED, cv::LINE_AA);
        cv::circle(f, c, R, hov ? a : kd::white(), cv::FILLED, cv::LINE_AA);
        cv::circle(f, c, R, a, 3, cv::LINE_AA);
        cv::Scalar sym = hov ? kd::white() : a;
        int s = R / 3 + 3;
        std::vector<cv::Point> ch;
        if (up) ch = {{c.x - s, c.y + s / 2}, {c.x, c.y - s / 2}, {c.x + s, c.y + s / 2}};
        else    ch = {{c.x - s, c.y - s / 2}, {c.x, c.y + s / 2}, {c.x + s, c.y - s / 2}};
        cv::polylines(f, ch, false, sym, 4, cv::LINE_AA);
    }

    void renderMain(cv::Mat& frame) {
        const int W = frame.cols;
        const cv::Scalar white = kd::white();
        frame.setTo(kd::cream());

        std::vector<WifiNetwork> safe_networks;
        {
            std::lock_guard<std::mutex> lock(networks_mutex_);
            safe_networks = networks_;
        }
        const bool refreshing = is_refreshing_.load();

        // Cabecera
        txu::put(frame, eng() ? "WIFI SETUP" : "CONFIGURAR WIFI", {20, 38},
                 cv::FONT_HERSHEY_DUPLEX, 0.9, orangeDk(), 1, cv::LINE_AA);

        if (!statusMsg_.empty()) {
            std::string msg = statusMsg_;
            bool isErr = msg.find("Error") != std::string::npos ||
                         msg.find("fallo") != std::string::npos ||
                         msg.find("Failed") != std::string::npos ||
                         msg.find("Wrong") != std::string::npos ||
                         msg.find("incorrecta") != std::string::npos ||
                         msg.find("SIN INTERNET") != std::string::npos ||
                         msg.find("NO INTERNET") != std::string::npos;
            cv::Scalar col = isErr ? cv::Scalar(70, 70, 225) : orangeDk();
            int bl = 0;
            cv::Size ts = txu::size(msg, cv::FONT_HERSHEY_DUPLEX, 0.5, 1, &bl);
            txu::put(frame, msg, {W - 20 - ts.width, 36}, cv::FONT_HERSHEY_DUPLEX, 0.5, col, 1, cv::LINE_AA);
        }

        // Panel
        cv::Rect P(14, 62, W - 28, 340);
        kd::card(frame, P, 26, tan(), 2);
        txu::put(frame, eng() ? "NETWORKS" : "REDES", {P.x + 26, P.y + 40},
                 cv::FONT_HERSHEY_DUPLEX, 0.85, kd::ink(), 1, cv::LINE_AA);
        cv::line(frame, {P.x + 20, P.y + 56}, {P.x + P.width - 20, P.y + 56}, kd::shade(), 2, cv::LINE_AA);

        btnRefresh_ = cv::Rect(P.x + P.width - 20 - 150, P.y + 10, 150, 38);
        kd::button(frame, btnRefresh_, eng() ? "REFRESH" : "REFRESCAR", caramel(),
                   btnRefresh_.contains(hovPt_) && !refreshing, !refreshing, 0.5);

        const int rowH = 58, rowGap = 8, btnSz = 56;
        const int listX = P.x + 20, listY = P.y + 74;
        const int listW = P.width - 40 - btnSz - 14;
        const int listH = kRows * rowH + (kRows - 1) * rowGap;

        rowRects_.clear();
        if (refreshing) {
            double t = kd::now();
            int d = (int)(t * 3.0) % 4;
            std::string lt = (eng() ? "Scanning networks" : "Buscando redes") + std::string(d, '.');
            kd::centered(frame, lt, W, P.y + 150, cv::FONT_HERSHEY_DUPLEX, 0.8, kd::ink(), 1);
            { cv::Scalar mono = orange(); kd::bouncingDots(frame, cv::Point(W / 2, P.y + 210), t, &mono); }
            btnScrollUp_ = {0, 0, 0, 0}; btnScrollDown_ = {0, 0, 0, 0};
        } else {
            const int total = (int)safe_networks.size();
            scroll_ = std::max(0, std::min(scroll_, total - kRows));

            if (total == 0)
                kd::centered(frame, eng() ? "No networks found" : "No se encontraron redes", W,
                             P.y + 170, cv::FONT_HERSHEY_DUPLEX, 0.7, kd::gray(), 1);

            for (int i = 0; i < kRows; ++i) {
                int idx = scroll_ + i;
                if (idx >= total) break;
                cv::Rect r(listX, listY + i * (rowH + rowGap), listW, rowH);
                rowRects_.push_back({r, idx});
                const WifiNetwork& n = safe_networks[idx];
                bool sel = (selectedIdx_ == idx);
                bool hov = r.contains(hovPt_);

                kd::fillRound(frame, cv::Rect(r.x, r.y + 3, r.width, r.height), 18, kd::shade());
                kd::fillRound(frame, r, 18, sel ? kd::mix(orange(), white, 0.88)
                                                : (hov ? cv::Scalar(246, 250, 252) : white));
                kd::strokeRound(frame, r, 18, sel ? orange() : tan(), sel ? 3 : 2);

                cv::Point ic(r.x + 34, r.y + r.height / 2);
                cv::circle(frame, ic, 20, sel ? orange() : kd::mix(orange(), white, 0.85), cv::FILLED, cv::LINE_AA);
                kd::wifiGlyph(frame, ic, sel ? white : orange());

                std::string label = n.ssid;
                int maxW = r.width - 62 - 165, bl = 0;
                if (txu::size(label, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &bl).width > maxW) {
                    while (label.size() > 3 &&
                           txu::size(label + "..", cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &bl).width > maxW)
                        label.pop_back();
                    label += "..";
                }
                cv::Size ts = txu::size(label, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &bl);
                txu::put(frame, label, {r.x + 62, r.y + (r.height + ts.height) / 2},
                         cv::FONT_HERSHEY_DUPLEX, 0.6, sel ? kd::ink() : cv::Scalar(90, 95, 100), 1, cv::LINE_AA);

                if (n.secured)
                    kd::lockGlyph(frame, cv::Point(r.x + r.width - 142, r.y + r.height / 2 - 2), kd::gray());

                cv::Rect pill(r.x + r.width - 118, r.y + r.height / 2 - 14, 106, 28);
                kd::fillRound(frame, pill, 14, n.saved ? cv::Scalar(90, 200, 90) : caramel());
                drawTextInRect(frame, n.saved ? (eng() ? "saved" : "guardada") : (eng() ? "new" : "nueva"),
                               pill, cv::FONT_HERSHEY_DUPLEX, 0.45, white, 1);
            }

            btnScrollUp_   = cv::Rect(listX + listW + 14, listY, btnSz, btnSz);
            btnScrollDown_ = cv::Rect(listX + listW + 14, listY + listH - btnSz, btnSz, btnSz);
            roundBtn(frame, btnScrollUp_,   true,  scroll_ > 0);
            roundBtn(frame, btnScrollDown_, false, scroll_ + kRows < total);
        }

        // Barra inferior (igual que Settings)
        btnSkip_ = cv::Rect(14, 418, 196, 52);
        kd::button(frame, btnSkip_, eng() ? "SKIP" : "OMITIR", caramel(), btnSkip_.contains(hovPt_), true, 0.6);

        btnConnect_ = cv::Rect(W - 14 - 260, 418, 260, 52);
        bool canC = (selectedIdx_ >= 0) && !connecting_ && !keyboard_.visible && !refreshing;
        std::string cLabel = connecting_ ? (eng() ? "Connecting..." : "Conectando...")
                                         : (eng() ? "CONNECT" : "CONECTAR");
        kd::button(frame, btnConnect_, cLabel, orange(), btnConnect_.contains(hovPt_) && canC, canC, 0.6);
    }
};
// =============================================================================
//  Navegación e Items
// =============================================================================
struct MenuItem {
    std::string id;
    std::string label;
    std::string sublabel;
    cv::Scalar  color   { 0, 0, 0, 0 };
    cv::Rect    rect    { 0, 0, 0, 0 };
    std::string cmd;
    std::string stopCmd;
    bool        hasSubMenu { false };
    std::string subMenuKey;
    std::string iconKey;
    std::vector<std::string> lifecycle_nodes;
    MenuItem() = default;
    MenuItem(std::string id_, std::string label_, std::string sublabel_, cv::Scalar color_,
             std::string cmd_, std::string stopCmd_, bool hasSubMenu_, std::string subMenuKey_, std::string iconKey_, std::vector<std::string> lc_nodes = {})
        : id(std::move(id_)), label(std::move(label_)), sublabel(std::move(sublabel_)), color(color_), rect(0, 0, 0, 0)
        , cmd(std::move(cmd_)), stopCmd(std::move(stopCmd_)), hasSubMenu(hasSubMenu_), subMenuKey(std::move(subMenuKey_)), iconKey(std::move(iconKey_)), lifecycle_nodes(std::move(lc_nodes)) {}
};
struct NavLevel {
    std::string           title;
    cv::Scalar            accentColor { 0, 200, 200 };
    std::vector<MenuItem> items;
    std::string           key {};
};
enum class FaceOverlay { NONE, ERROR_MSG, MIC_COUNTDOWN, MIC_PLAYING };

// =============================================================================
//  VideoSynchronizer
// =============================================================================
class VideoSynchronizer : public rclcpp::Node {
public:
    VideoSynchronizer() : Node("face_screen") {
        std::string pkgDir = ament_index_cpp::get_package_share_directory("yaren_face_display");
        {
            const std::string dir = pkgDir + "/faces/nuevas_caras";
            std::string openP, closedP, talkP;
            if (fs::exists(dir)) {
                for (const auto& e : fs::directory_iterator(dir)) {
                    std::string ext = e.path().extension().string();
                    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                    if (ext != ".png") continue;
                    std::string n = e.path().filename().string();
                    std::transform(n.begin(), n.end(), n.begin(), ::tolower);
                    if (n.find("cerr") != std::string::npos)       closedP = e.path().string();
                    else if (n.find("abier") != std::string::npos) openP   = e.path().string();
                    else if (n.find("habl") != std::string::npos)  talkP   = e.path().string();
                }
            }
            if (!openP.empty())   faceOpenImg_   = cv::imread(openP,   cv::IMREAD_UNCHANGED);
            if (!closedP.empty()) faceClosedImg_ = cv::imread(closedP, cv::IMREAD_UNCHANGED);
            if (!talkP.empty())   faceTalkImg_   = cv::imread(talkP,   cv::IMREAD_UNCHANGED);
        }
        lastInteractionTime = std::chrono::system_clock::now();
        auto checkImg = [&](const cv::Mat& m, const std::string& name) {
            if (m.empty()) RCLCPP_WARN(get_logger(), "IMAGEN NO CARGADA: %s", name.c_str());
        };
        checkImg(faceOpenImg_, "cara_abierta"); checkImg(faceClosedImg_, "cara_cerrada");
        yarenSplashImg = cv::imread(pkgDir + "/icons/yaren.png", cv::IMREAD_UNCHANGED);
        loadFrames(pkgDir + "/faces/transitions/blinking_frames", eyesFrames);
        loadFrames(pkgDir + "/faces/transitions/blinking_frames", mouthFrames);
        auto loadIcon = [&](const std::string& key, const std::string& file) {
            cv::Mat img = cv::imread(pkgDir + "/icons/" + file, cv::IMREAD_UNCHANGED);
            if (!img.empty()) {
                int mn = std::min(img.cols, img.rows);
                if (mn > 400) cv::resize(img, img, cv::Size(), 400.0 / mn, 400.0 / mn, cv::INTER_AREA);
                iconMap[key] = img;
            }
            else RCLCPP_WARN(get_logger(), "Icono no encontrado: %s", file.c_str());
        };
        loadIcon("camera", "camera.png"); loadIcon("microfono", "microfono.png"); loadIcon("motores", "motores.png");
        loadIcon("test", "test.png"); loadIcon("brazo_izq", "brazo_izquierdo.png"); loadIcon("brazo_der", "brazo_derecho.png");
        loadIcon("girar_base", "girar_base.png"); loadIcon("girar_cabeza", "girar_cabeza.png"); loadIcon("yaren", "yaren.png");
        loadIcon("arriba", "arriba.png"); loadIcon("abajo", "abajo.png"); loadIcon("izquierda", "izquierda.png");
        loadIcon("derecha", "derecha.png"); loadIcon("medio", "derecha.png"); loadIcon("mimic", "mimic.png");
        loadIcon("chat", "chat_bot.png"); loadIcon("dice", "simon_dice.png"); loadIcon("movements", "movements.png");
        loadIcon("emotions", "emociones.png"); loadIcon("filtros", "filtros_menu.png"); loadIcon("rutina1", "yaren2.png");
        loadIcon("rutina2", "yaren3.png"); loadIcon("rutina3", "yaren4.png"); loadIcon("animales", "animales.png");
        loadIcon("accesorios", "filtro.png"); loadIcon("settings", "settings.png"); loadIcon("piopio", "piopio.png");
        loadIcon("gallinaturuleca", "gallinaturuleca.png"); loadIcon("susanita", "susanita.png"); loadIcon("vacalola", "vacalola.png");
        loadIcon("video", "video.png"); loadIcon("musica", "musica.png"); loadIcon("radio", "radio.png");
        loadIcon("palabras", "juegopalabras.png"); loadIcon("paint", "pintor.png");
        loadIcon("juegobaile", "juegobaile.png"); loadIcon("memoria", "memoria.png");
        loadIcon("rutinapersonal", "rutinapersonal.png"); loadIcon("juegos", "juegos.png");
        {
            cv::Mat bg = cv::imread(pkgDir + "/icons/fondo_menu.png", cv::IMREAD_COLOR);
            if (!bg.empty()) {
                const int TW = 800, TH = 480;
                double k = std::max(TW / (double)bg.cols, TH / (double)bg.rows);
                cv::Mat rs;
                cv::resize(bg, rs, cv::Size(), k, k, cv::INTER_AREA);
                int x0 = std::max(0, (rs.cols - TW) / 2), y0 = std::max(0, (rs.rows - TH) / 2);
                menuBg_ = rs(cv::Rect(x0, y0, std::min(TW, rs.cols), std::min(TH, rs.rows))).clone();
            } else {
                RCLCPP_WARN(get_logger(), "Fondo del menu no encontrado: icons/fondo_menu.png");
            }
        } 

        ttsActive = false; isBlinking = false; hoveredItem = -1;
        hoveredBack = hoveredStop = hoveredExit = false;
        activeMode = ""; activeStopCmd = ""; running = true;
        showSettings_ = false; showRadio_ = false; showRoutines_ = false;
        lastBlinkTime = std::chrono::system_clock::now();

        // 1. Inicializar el tiempo de inactividad
        lastInteractionTime = std::chrono::system_clock::now();
        faceOverlay = FaceOverlay::NONE; overlayMessage = ""; micCountdownSecs = 0;
        
        settingsMenu.onBack = [this]() {
            std::lock_guard<std::mutex> lk(modeFlagMutex);
            showSettings_ = false;
        };
        settingsMenu.onWifi = [this]() {
            // Cerrar settings y abrir WiFi setup
            {
                std::lock_guard<std::mutex> lk(modeFlagMutex);
                showSettings_  = false;
                showWifiSetup_ = true;
            }
            wifiSetup_.refresh();
            RCLCPP_INFO(this->get_logger(),
                "[ WiFi ] Abriendo configuracion WiFi desde Settings...");
        };

        radioApp.init(pkgDir, this->get_logger());
        radioApp.isEnglish = &isEnglish;
        routinesApp.isEnglish = &isEnglish;
        wifiSetup_.isEnglish = &isEnglish;
        wifiSetup_.onSkip = [this]() {
            std::lock_guard<std::mutex> lk(modeFlagMutex);
            showWifiSetup_ = false;
        };
        wifiSetup_.onConnected = [this]() {
            chatAvailable_ = checkChatAvailable();
            std::lock_guard<std::mutex> lk(modeFlagMutex);
            showWifiSetup_ = false;
        };
        radioApp.setAudioMutex(&audioMutex_);
        settingsMenu.setAudioMutex(&audioMutex_);

        std::srand(std::time(nullptr));
        
        // 2. Definir ws_dir ANTES de usarla
        std::string ws_dir = std::filesystem::current_path().string();
        {
            const std::string introDir = ws_dir + "/src/YAREN2/yaren_radio/videos/";
            introPath_ = introDir + "intro_yaren.mp4";
            introVideo.open(introPath_);
            if (introVideo.isOpened()) {
                introFps = introVideo.get(cv::CAP_PROP_FPS);
                if (introFps < 1.0 || introFps > 120.0) introFps = 30.0;
                {
                    double nf = introVideo.get(cv::CAP_PROP_FRAME_COUNT);
                    introDur_ = (nf > 0) ? nf / introFps : 0.0;
                }
                introIdx = -1;
                introStart = std::chrono::steady_clock::now();
                const std::string wav = introDir + "intro_yaren.wav";
                if (fs::exists(wav)) {
                    std::lock_guard<std::mutex> lk(audioMutex_);
                    introMusic = Mix_LoadMUS(wav.c_str());
                    if (introMusic) {
                        Mix_VolumeMusic(settingsMenu.volumeLevel);
                        Mix_PlayMusic(introMusic, -1);
                        introAudioOk_ = true;
                    } else {
                        RCLCPP_WARN(this->get_logger(), "No se pudo cargar audio intro: %s", Mix_GetError());
                    }
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "No se pudo abrir el video de intro en: %s", introDir.c_str());
            }
        }

        // 3. Asignar rutas a las variables (sin re-declararlas)
        idleVideoPaths = {
            ws_dir + "/src/YAREN2/yaren_radio/videos/carro.mp4",
            ws_dir + "/src/YAREN2/yaren_radio/videos/gatito.mp4",
            ws_dir + "/src/YAREN2/yaren_radio/videos/jake.mp4",
            ws_dir + "/src/YAREN2/yaren_radio/videos/bart.mp4"
        };
        
        idleMusicPaths = {
            ws_dir + "/src/YAREN2/yaren_radio/audios/The One That Got Away.mp3",
            ws_dir + "/src/YAREN2/yaren_radio/audios/Ghost.mp3",
            ws_dir + "/src/YAREN2/yaren_radio/audios/Un Beso Y Una Flor.mp3",
            ws_dir + "/src/YAREN2/yaren_radio/audios/The Winner Takes It All.mp3",
            ws_dir + "/src/YAREN2/yaren_radio/audios/Love Me Again.mp3",
            ws_dir + "/src/YAREN2/yaren_radio/audios/Balada.mp3",
            ws_dir + "/src/YAREN2/yaren_radio/audios/Spiderman.mp3"
        };
        bootMusicPaths = {
            ws_dir + "/src/YAREN2/yaren_radio/audios/bringmetolife.mp3",
            ws_dir + "/src/YAREN2/yaren_radio/audios/cancionmundial2010.mp3",
            ws_dir + "/src/YAREN2/yaren_radio/audios/justthewayuare.mp3"
        };

        // Reproducir musica de arranque
       // Reproducir musica de arranque
        {
            std::lock_guard<std::mutex> lk(audioMutex_);
            std::string bootTrack = bootMusicPaths[std::rand() % bootMusicPaths.size()];
            if (!introAudioOk_ && fs::exists(bootTrack)) {
                bootMusic = Mix_LoadMUS(bootTrack.c_str());
                if (bootMusic) {
                    // El -1 hace que la canción haga loop infinito durante la carga
                    Mix_PlayMusic(bootMusic, -1); 
                    Mix_VolumeMusic(settingsMenu.volumeLevel);
                }
            } else {
                // Esto te ayudará a saber si escribiste mal el nombre del .mp3
                RCLCPP_WARN(this->get_logger(), "Música de arranque no encontrada: %s", bootTrack.c_str());
            }
        }
        menuPlaylist = {
            ws_dir + "/src/YAREN2/yaren_radio/audios/Intro1.mp3",
            ws_dir + "/src/YAREN2/yaren_radio/audios/Intro2.mp3",
            ws_dir + "/src/YAREN2/yaren_radio/audios/Intro3.mp3",
        };
        radioApp.onBack = [this]() {
            std::lock_guard<std::mutex> lock(navMutex);
            radioApp.killAudio();
            {
                std::lock_guard<std::mutex> lk(modeFlagMutex);
                showRadio_ = false;
            }
            startMenuMusic();
        };
        routinesApp.onBack = [this]() {
            {
                std::lock_guard<std::mutex> lk(modeFlagMutex);
                showRoutines_ = false;
            }
            auto msg = std_msgs::msg::String();
            msg.data = "idle";
            modePublisher->publish(msg);
        };
        routinesApp.onLaunchSubprocess = [this](std::string cmd) {
            std::thread([this, cmd]() {
                int ret = std::system(cmd.c_str());
                if (ret != 0) {
                    RCLCPP_ERROR(get_logger(), "Fallo ejecutando rutina: %s", cmd.c_str());
                    showErrorOverlay(isEnglish ? "Failed to execute routine." : "Fallo al ejecutar la rutina.", 3.0);
                }
            }).detach();
        };
        ttsSubscription = this->create_subscription<std_msgs::msg::Bool>(
            "/audio_playing", 10, std::bind(&VideoSynchronizer::audioPlayingCallback, this, std::placeholders::_1));
        faceScreenPublisher = this->create_publisher<sensor_msgs::msg::Image>("/face_screen", 10);
        modePublisher       = this->create_publisher<std_msgs::msg::String>("/yaren_mode", 10);
        rclcpp::QoS uiQos(1);
        uiQos.transient_local();   // la app recibe el último estado apenas se conecta
        uiStatePub_ = this->create_publisher<std_msgs::msg::String>(kUiStateTopic, uiQos);
        commandSub_ = this->create_subscription<std_msgs::msg::String>(
            kCommandTopic, 10,
            std::bind(&VideoSynchronizer::onRemoteCommand, this, std::placeholders::_1));

        modeSubscription_ = this->create_subscription<std_msgs::msg::String>(
            "/yaren_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                // FIX-I: copiar datos y publicar fuera del lock de navMutex
                // FIX-D: no retener navMutex mientras se hacen std::system() bloqueantes
                std::string modeData = msg->data;
                std::string stopCmdToRun;
                std::string modeToPublish;
                MenuItem targetItem;
                bool clearNav   = false;
                bool stopRadio  = false;
                bool stopRouts  = false;
                bool doRadioReset = false;
                int playSongIdx = -1;

                {
                    std::lock_guard<std::mutex> lock(navMutex);
                    if (modeData == "idle" || modeData.empty()) {
                        for (const auto& node : active_lifecycle_nodes) {
                            change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                        }
                        active_lifecycle_nodes.clear();
                        stopCmdToRun = activeStopCmd;
                        if (activeMode == "yaren_dice_con_ayuda" || activeMode == "yaren_dice_sesion") {
                                std::string homeCmd = "ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"";
                            std::thread([homeCmd]() { std::system(homeCmd.c_str()); }).detach();
                        }
                        activeMode.clear(); activeStopCmd.clear();
                        bool radioShowing, routinesShowing;
                        {
                            std::lock_guard<std::mutex> lk(modeFlagMutex);
                            radioShowing    = showRadio_;
                            routinesShowing = showRoutines_;
                        }
                        stopRadio = radioShowing;
                        stopRouts = routinesShowing;
                    } else if (modeData.rfind("play_radio_song:", 0) == 0) {
		    navStack.clear();
		    hoveredItem = -1;
		    hoveredBack = hoveredStop = hoveredExit = false;
		    
		    // --- INICIO BLOQUE PROTEGIDO ---
		    try {
			playSongIdx = std::stoi(modeData.substr(16));
		    } catch (const std::exception& e) {
			RCLCPP_WARN(this->get_logger(), "Comando de cancion invalido recibido: '%s'. Error: %s", modeData.c_str(), e.what());
			playSongIdx = -1; // -1 significa que no reproduzca nada
		    }
		    // --- FIN BLOQUE PROTEGIDO ---

		    doRadioReset = true;
		    activeMode = "radio_musica";
		    activeStopCmd = "ros2 topic pub --once /yaren_mode std_msgs/msg/String \"{data: 'idle'}\"";
		    clearNav = true;
		} else if (modeData == "radio_musica") {
                        navStack.clear();
                        hoveredItem = -1;
                        hoveredBack = hoveredStop = hoveredExit = false;
                        doRadioReset = true;
                        activeMode = "radio_musica";
                        activeStopCmd = "ros2 topic pub --once /yaren_mode std_msgs/msg/String \"{data: 'idle'}\"";
                        clearNav = true;
                    }else {
                        navStack.clear();
                        hoveredItem = -1;
                        hoveredBack = hoveredStop = hoveredExit = false;
                        if (activeMode != modeData) {
                            targetItem = findMenuItem(modeData);
                            activeMode = modeData; // ✅ FIX: Actualizar el modo ANTES de soltar el lock
                        }
                        if (activeMode == "yaren_chat") first_listen_done = false;
                        clearNav = true;
                    }
                } // navMutex released

                // FIX-D: ejecutar std::system FUERA del lock
                if (!stopCmdToRun.empty()) {
                    std::thread([stopCmdToRun]() { std::system(stopCmdToRun.c_str()); }).detach();
                }
                if (stopRadio) {
                    radioApp.killAudio();
                    {
                        std::lock_guard<std::mutex> lk(modeFlagMutex);
                        showRadio_ = false;
                    }
                    startMenuMusic();
                }
                if (stopRouts) {
                    {
                        std::lock_guard<std::mutex> lk(modeFlagMutex);
                        showRoutines_ = false;
                    }
                    startMenuMusic();
                }
                if (clearNav && (modeData.rfind("play_radio_song:", 0) == 0 || modeData == "radio_musica")) {
                    stopMenuMusic();
                    {
                        std::lock_guard<std::mutex> lk(modeFlagMutex);
                        showRadio_ = true;
                    }
                    radioApp.reset();
                    if (playSongIdx >= 0) radioApp.playSong(playSongIdx);
                }
                // FIX-I: executeMode llama publish FUERA de navMutex
                if (!targetItem.id.empty()) {
                    executeMode(targetItem, false);
                }
            });

        sttListeningSubscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/stt_listening", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (activeMode == "yaren_chat" || activeMode == "yaren_chat_local") {
                    if (msg->data) {
                        // Es el turno del niño
                        std::thread([this]() {
                            showCustomOverlay(FaceOverlay::MIC_PLAYING,
                                isEnglish ? "Your turn Speak now" : "Tu turno, habla ahora",
                                2.0);
                        }).detach();
                    } else {
                        
                    }
                }
            }
        );
        lipsyncSub_ = this->create_subscription<std_msgs::msg::String>(
            "/yaren/tts_text", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                std::string data = msg->data;
                std::queue<VisemeFrame> q;

                try {
                    if (data.find('|') != std::string::npos) {
                        // Formato nuevo: "dur:idx|dur:idx|..."
                        std::istringstream ss(data);
                        std::string token;
                        while (std::getline(ss, token, '|')) {
                            if (token.empty()) continue;
                            size_t colon = token.find(':');
                            // Evitar error si no hay ':' o si no hay nada antes del ':'
                            if (colon == std::string::npos || colon == 0) continue; 
                            
                            uint32_t dur = (uint32_t)std::stoul(token.substr(0, colon));
                            int      idx = std::stoi(token.substr(colon + 1));
                            q.push({std::max(0, std::min(8, idx)), dur});
                        }
                    } else {
                        // Formato legacy: "dur_ms:idx,idx,..."
                        static constexpr uint32_t VISEME_MS[9] = {80,55,90,100,80,90,110,120,75};
                        size_t colon = data.find(':');
                        // Evitar error si no hay ':' o si no hay nada antes del ':'
                        if (colon == std::string::npos || colon == 0) return; 
                        
                        uint32_t base = (uint32_t)std::stoul(data.substr(0, colon));
                        std::istringstream ss(data.substr(colon + 1));
                        std::string token;
                        while (std::getline(ss, token, ',')) {
                            if (token.empty()) continue;
                            int idx = std::max(0, std::min(8, std::stoi(token)));
                            uint32_t dur = std::max(40u, std::min(200u,
                                (uint32_t)(base * VISEME_MS[idx] / 80u)));
                            q.push({idx, dur});
                        }
                    }
                } 
                catch (const std::exception& e) {
                    // Si el TTS manda basura, lo atrapamos aquí en lugar de crashear el nodo entero
                    RCLCPP_WARN(this->get_logger(), "Mensaje de visema ignorado por formato invalido '%s': %s", data.c_str(), e.what());
                    return; 
                }

                std::lock_guard<std::mutex> lk(visemeMutex_);
                visemeQueue_ = std::move(q);
                visemeDeadline_ = std::chrono::steady_clock::now();
                visemeBlend_ = 0.0f;
            }
        );
        cv::namedWindow("Yaren Face", cv::WINDOW_NORMAL);
        cv::setWindowProperty("Yaren Face", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::moveWindow("Yaren Face", 0, 0);
        cv::setMouseCallback("Yaren Face", VideoSynchronizer::mouseCallbackStatic, this);

        rclcpp::QoS qos_profile(1);
        qos_profile.transient_local();
        languagePublisher = this->create_publisher<std_msgs::msg::Bool>("/yaren/is_english", qos_profile);
        rclcpp::QoS qos_profile_idle(1);
        qos_profile_idle.transient_local();
        idleStatePublisher = this->create_publisher<std_msgs::msg::Bool>("/yaren/face_idle", qos_profile_idle);
        rclcpp::QoS qos_mic(1);
        qos_mic.transient_local();
        micOwnerPublisher_ = this->create_publisher<std_msgs::msg::String>("/yaren/mic_owner", qos_mic);
        micTestPublisher_ = this->create_publisher<std_msgs::msg::Bool>("/yaren/mic_test", 10); 

        wakeEventSubscription = this->create_subscription<std_msgs::msg::Bool>(
            "/yaren/wake_event", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                resetIdleTimer(); // Despertar a Yaren si escucha su nombre
                RCLCPP_INFO(this->get_logger(), "¡Wake Word detectado! Despertando a Yaren...");
            }
        );
        auto initial_msg = std_msgs::msg::Bool();
        initial_msg.data = this->isEnglish;
        languagePublisher->publish(initial_msg);
        settingsMenu.isEnglish = &this->isEnglish;
        settingsMenu.onLanguageChanged = [this]() {
            std::lock_guard<std::mutex> lock(navMutex);
            this->buildMenus();
            if (navStack.size() > 1) {
                std::vector<std::string> keys;
                for(size_t i = 1; i < navStack.size(); ++i) keys.push_back(navStack[i].key);
                navStack.clear();
                NavLevel root;
                root.title = isEnglish ? "MAIN MENU" : "MENÚ PRINCIPAL";
                root.items = rootMenuItems;
                navStack.push_back(root);
                for(const auto& key : keys) {
                    if(subMenuMap.count(key)) {
                        NavLevel lvl = subMenuMap.at(key);
                        lvl.key = key;
                        navStack.push_back(lvl);
                    }
                }
            } else {
                navStack[0].title = isEnglish ? "MAIN MENU" : "MENÚ PRINCIPAL";
                navStack[0].items = rootMenuItems;
            }
            auto msg = std_msgs::msg::Bool();
            msg.data = this->isEnglish;
            this->languagePublisher->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Pantalla: Idioma publicado -> %s", msg.data ? "English" : "Español");
        };
        buildMenus();
        // NOTA: renderThread se arranca desde main() post-construcción
        RCLCPP_INFO(get_logger(), "face_screen listo con Radio y Rutinas Personales.");

        std::system("for pid in $(ps aux | grep -E 'ws_bridge|wake_word_node|yaren_voice_menu|gestor_idioma|yaren_chat|lifecycle_node|yaren_emotions|yaren_radio|yaren_filters|yaren_dice|yaren_mimic|memoria_node|dance_game_node|paint_node|palabras_node' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done");
        const char* home = std::getenv("HOME");
        if (home) {
            std::string python  = std::string(home) + "/robotis_ws/venv_yaren/bin/python3";
            std::string ws      = std::string(home) + "/robotis_ws";
            std::string setup   = ws + "/install/setup.bash";
            std::string venv    = ws + "/venv_yaren/bin/activate";
            std::string bridge = ws + "/src/YAREN2/yaren_face_display/scripts/ws_bridge.py";
            std::string wake    = ws + "/src/YAREN2/yaren_wakeupword/yaren_wakeupword/wake_word_node.py";
            std::string voice   = ws + "/src/YAREN2/yaren_wakeupword/yaren_wakeupword/yaren_voice_menu.py";
            std::string lang    = ws + "/src/YAREN2/yaren_idioma/yaren_idioma/gestor_idioma.py";

            std::string cmd_wake  = "bash -c 'source " + setup + " && " + python + " " + wake  + "' &";
            std::string cmd_voice = "bash -c 'source " + setup + " && " + python + " " + voice + "' &";
            std::string cmd_lang  = "bash -c 'source " + setup + " && " + python + " " + lang  + "' &";

            std::string cmd_llm = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_chat llm_lifecycle_node.py' &";
            std::string cmd_stt = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_chat stt_lifecycle_node.py' &";
            std::string cmd_tts = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_chat tts_lifecycle_node.py' &";
            std::string cmd_det = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_emotions detect_emotion.py' &";
            std::string cmd_filter = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_filters face_filter_node' &";
            std::string cmd_animal = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_filters animal_filter_mask' &";
            std::string cmd_landmarks = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_filters face_landmark_detector.py' &";
            std::string cmd_body = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_dice body_landmarks.py' &";
            std::string cmd_body_help = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_dice body_landmarks_visual.py' &";
            std::string cmd_speak = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_dice speaker_node.py' &";
            std::string cmd_fondo = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_filters fondo_virtual.py' &";
            std::string cmd_camara = "bash -c 'source " + setup + " && ros2 run camara_usb_csi csi_cam_pub.py' &";
            std::string cmd_llm_local = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_chat2 llm_local_lifecycle_node.py' &";
            std::string cmd_stt_local = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_chat2 stt_local_lifecycle_node.py' &";
            std::string cmd_tts_local = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_chat2 tts_local_lifecycle_node.py' &";
            std::string cmd_paint = "bash -c 'source " + setup + " && source " + venv + " && python3 " + ws + "/src/YAREN2/yaren_paint/yaren_paint/paint_node.py' &";
            std::string cmd_memoria = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_juegos memoria_node' &";
            std::string cmd_dance = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_juegos dance_game_node' &"; // NUEVO
            std::string cmd_palabras = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_juegos palabras_node' &";
            std::string cmd_body_tracker = "bash -c 'source " + setup + " && ros2 run yaren_arm_mimic body_tracker_node' &";
            std::string cmd_controller   = "bash -c 'source " + setup + " && ros2 run yaren_arm_mimic yaren_controller' &";
            std::string cmd_mimic_gate   = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_arm_mimic mimic_gate_node.py' &";
            std::string cmd_body_mimic = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_arm_mimic body_points_detector.py' &";
            std::string cmd_bridge = "bash -c 'source " + setup +" && python3 " + bridge +" --ros-args -p allow_poweroff:=true' > /tmp/ws_bridge.log 2>&1 &";

            std::system(cmd_wake.c_str());
            std::system(cmd_voice.c_str());
            std::system(cmd_lang.c_str());
            std::system(cmd_llm.c_str());
            std::system(cmd_stt.c_str());
            std::system(cmd_tts.c_str());
            std::system(cmd_det.c_str());
            std::system(cmd_filter.c_str());
            std::system(cmd_animal.c_str());
            std::system(cmd_landmarks.c_str());
            std::system(cmd_body.c_str());
            std::system(cmd_body_help.c_str());
            std::system(cmd_speak.c_str());
            std::system(cmd_fondo.c_str());
            std::system(cmd_camara.c_str()); 
            std::system(cmd_llm_local.c_str());
            std::system(cmd_stt_local.c_str());
            std::system(cmd_tts_local.c_str());
            std::system(cmd_memoria.c_str());
            std::system(cmd_dance.c_str());
            std::system(cmd_paint.c_str());
            std::system(cmd_palabras.c_str());
            std::system(cmd_body_tracker.c_str());
            std::system(cmd_controller.c_str());
            std::system(cmd_mimic_gate.c_str());
            std::system(cmd_body_mimic.c_str());
            std::system(cmd_bridge.c_str());


                        std::thread([this, setup]() {
                // ── a) Pausa inicial para que se vea la animación ──
                RCLCPP_INFO(this->get_logger(),
                    "[ BOOT ] Esperando 3s antes de verificar red...");
                std::this_thread::sleep_for(std::chrono::seconds(3));
 
                // ── b) Verificar internet con timeout de 10s ──
                RCLCPP_INFO(this->get_logger(),
                    "[ BOOT ] Verificando conexion a internet...");
                chatAvailable_ = checkChatAvailable();
 
                // ── c) Sin internet → mostrar pantalla WiFi ──
                if (!chatAvailable_) {
                    RCLCPP_WARN(this->get_logger(),
                        "[ BOOT ] Sin internet. Mostrando configuracion WiFi...");
 
                    {
                        std::lock_guard<std::mutex> lk(configStatusMutex);
                        configStatus = isEnglish
                            ? "No internet - configure WiFi..."
                            : "Sin internet - configura el WiFi...";
                    }
 
                    wifiSetup_.refresh();
                    {
                        std::lock_guard<std::mutex> lk(modeFlagMutex);
                        showWifiSetup_ = true;
                    }
 
                    // Esperar hasta que el usuario conecte u omita
                    while (true) {
                        bool still;
                        {
                            std::lock_guard<std::mutex> lk(modeFlagMutex);
                            still = showWifiSetup_;
                        }
                        if (!still) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    }
 
                    // Re-verificar tras el intento de conexión
                    chatAvailable_ = checkChatAvailable();
                    RCLCPP_INFO(this->get_logger(),
                        "[ BOOT ] Tras WiFi setup — internet: %s",
                        chatAvailable_ ? "SI" : "NO");
                } else {
                    RCLCPP_INFO(this->get_logger(),
                        "[ BOOT ] Internet OK. Continuando arranque...");
                }
 
                // ── d) Configurar lifecycle nodes ──
                auto configure_node = [&](const std::string& name,
                                          const std::string& display) {
                    {
                        std::lock_guard<std::mutex> lk(configStatusMutex);
                        configStatus = "Cargando " + display + "...";
                    }
                    RCLCPP_INFO(this->get_logger(),
                        "[ NODE ] Configurando: %s", name.c_str());
 
                    bool ok = false;
                    for (int intento = 0; intento < 3 && !ok; intento++) {
                        if (intento > 0) {
                            RCLCPP_WARN(this->get_logger(),
                                "[ NODE ] Reintentando %s (%d/3)...",
                                name.c_str(), intento + 1);
                            std::this_thread::sleep_for(std::chrono::seconds(3));
                        }
                        int ret = std::system(
                            ("bash -c 'source " + setup +
                             " && ros2 lifecycle set /" + name +
                             " configure' > /dev/null 2>&1").c_str());
                        ok = (ret == 0);
                    }
 
                    if (ok)
                        RCLCPP_INFO(this->get_logger(),
                            "[ NODE ] ✓ %s configurado.", name.c_str());
                    else
                        RCLCPP_ERROR(this->get_logger(),
                            "[ NODE ] ✗ No se pudo configurar: %s", name.c_str());
 
                    configProgress++;
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                };
                
                configure_node("llm_lifecycle_node",        "Inteligencia Artificial (Cloud)");
                configure_node("llm_local_lifecycle_node",  "Inteligencia Artificial (Local)");               
                configure_node("stt_lifecycle_node",        "Reconocimiento de Voz (Cloud)");
                configure_node("tts_lifecycle_node",        "Síntesis de Voz (Cloud)");
                configure_node("stt_local_lifecycle_node",        "Reconocimiento de Voz (Local)");
                configure_node("tts_local_lifecycle_node",        "Síntesis de Voz (Local)");
                configure_node("detector",                  "Detección de Emociones");
                configure_node("face_filter_node",          "Filtros de Cara");
                configure_node("body_points_detector_node", "Detección Corporal");
                configure_node("body_points_detector_node_visual", "Detección Corporal con Ayuda");
                configure_node("yaren_speaker_node",        "Altavoz");
                configure_node("virtual_background_node",   "Fondo Virtual");
                configure_node("filtro_animales",           "Filtro de Animales");
                configure_node("face_landmark_publisher",   "Landmarks Faciales");
                configure_node("csi_cam_node",              "Cámara Principal");
                configure_node("paint_node", "Yaren Paint");
                configure_node("memoria_node",  "Juego de Memoria");
                configure_node("dance_game_node", "Juego de Baile");
                configure_node("palabras_node", "Juego de palabras");
 
                // ── e) Finalizar pantalla de carga ──
                {
                    std::lock_guard<std::mutex> lk(configStatusMutex);
                    configStatus   = isEnglish ? "Yaren is ready!" : "Yaren listo!";
                    configProgress = configTotal;
                }
                RCLCPP_INFO(this->get_logger(),
                    "[ BOOT ] Sistema listo. Iniciando modo interactivo.");
 
                std::this_thread::sleep_for(std::chrono::seconds(2));
                resetIdleTimer();
 
                {
                    std::lock_guard<std::mutex> lk(audioMutex_);
                    if (bootMusic) {
                        Mix_HaltMusic();
                        Mix_FreeMusic(bootMusic);
                        bootMusic = nullptr;
                    }
                }
 
                {
                    std::lock_guard<std::mutex> lk(audioMutex_);
                    if (introMusic) {
                        Mix_HaltMusic();
                        Mix_FreeMusic(introMusic);
                        introMusic = nullptr;
                    }
                }
                configuring = false;
 
            }).detach();
        }
    }

    // FIX: startRenderThread se llama desde main() post-construcción
    void startRenderThread() {
        renderThread = std::thread(&VideoSynchronizer::renderLoop, this);
    }
    void startIdleScreen() {
        isIdleScreenActive = true;
        stopMenuMusic(); // Pausar la música habitual del menú

        // Iniciar Video Aleatorio
        if (!idleVideoPaths.empty()) {
            std::string vidPath = idleVideoPaths[std::rand() % idleVideoPaths.size()];
            idleVideo.open(vidPath);
        }

        // Iniciar Música Relajante Aleatoria
        if (!idleMusicPaths.empty()) {
            std::lock_guard<std::mutex> lk(audioMutex_);
            if (idleMusic) { Mix_FreeMusic(idleMusic); idleMusic = nullptr; }
            std::string audPath = idleMusicPaths[std::rand() % idleMusicPaths.size()];
            idleMusic = Mix_LoadMUS(audPath.c_str());
            if (idleMusic) {
                Mix_PlayMusic(idleMusic, -1); // -1 para loop infinito
                Mix_VolumeMusic(settingsMenu.volumeLevel);
            }
        }
    }

    void stopIdleScreen() {
        if (!isIdleScreenActive) return;
        isIdleScreenActive = false;
        lastInteractionTime = std::chrono::system_clock::now();

        if (idleVideo.isOpened()) idleVideo.release();

        {
            std::lock_guard<std::mutex> lk(audioMutex_);
            if (idleMusic) {
                Mix_HaltMusic();
                Mix_FreeMusic(idleMusic);
                idleMusic = nullptr;
            }
        }

        // Si volvemos y estábamos en el menú principal, reanudar su música
        if (navStack.size() == 1 && !showSettings_ && !showRadio_ && !showRoutines_) {
            startMenuMusic();
        }
    }

    void resetIdleTimer() {
        lastInteractionTime = std::chrono::system_clock::now();
        if (isIdleScreenActive) {
            stopIdleScreen(); // Despertar a Yaren
        }
    }
    ~VideoSynchronizer() {
        running = false;
        {
            // FIX-E: killAudio ya es thread-safe vía audioMutex_
            radioApp.killAudio();
        }
        // FIX-G: liberar menuMusic en destructor
        {
            std::lock_guard<std::mutex> lk(audioMutex_);
            if (menuMusic) {
                Mix_HaltMusic();
                Mix_FreeMusic(menuMusic);
                menuMusic = nullptr;
            }
        }
        {
            std::lock_guard<std::mutex> lk(audioMutex_);
            if (bootMusic) {
                Mix_HaltMusic();
                Mix_FreeMusic(bootMusic);
                bootMusic = nullptr;
            }
        }
        if (renderThread.joinable()) renderThread.join();
        // FIX-F: verificar joinable antes de join en testThread
        cv::destroyAllWindows();
        if (!activeStopCmd.empty()) std::system(activeStopCmd.c_str());
            std::system("for pid in $(ps aux | grep -E 'ws_bridge|wake_word_node|yaren_voice_menu|gestor_idioma|yaren_chat|lifecycle_node|yaren_emotions|yaren_radio|yaren_filters|yaren_dice|yaren_mimic|memoria_node|dance_game_node|paint_node|palabras_node' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done");
        }

    void drawWindow() {
        // FIX-C: copiar frame con lock, luego mostrar fuera del lock
        cv::Mat frameToDraw;
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            if (!latestFrame.empty()) frameToDraw = latestFrame.clone();
        }
        if (!frameToDraw.empty()) {
            cv::imshow("Yaren Face", frameToDraw);
            int key = cv::waitKey(1);
            if (key == 27) {
                bool sSettings, sRadio;
                {
                    std::lock_guard<std::mutex> lk(modeFlagMutex);
                    sSettings = showSettings_;
                    sRadio    = showRadio_;
                }
                if (sSettings) {
                    std::lock_guard<std::mutex> lk(modeFlagMutex);
                    showSettings_ = false;
                    return;
                }
                if (sRadio) {
                    radioApp.killAudio();
                    {
                        std::lock_guard<std::mutex> lk(modeFlagMutex);
                        showRadio_ = false;
                    }
                    startMenuMusic();
                    return;
                }
                std::lock_guard<std::mutex> nlock(navMutex);
                if (!navStack.empty()) {
                    if (navStack.size() > 1) navStack.pop_back();
                    else {
                        navStack.clear();
                        stopMenuMusic();
                    }
                    hoveredItem = -1; hoveredBack = hoveredStop = hoveredExit = false;
                } else {
                    running = false; rclcpp::shutdown();
                }
            }
        }
    }

    static void mouseCallbackStatic(int event, int x, int y, int /*flags*/, void* userdata) {
        static_cast<VideoSynchronizer*>(userdata)->handleMouse(event, x, y);
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr modeSubscription_;
    SettingsMenu settingsMenu;

    mutable std::mutex modeFlagMutex;
    bool showSettings_  { false };
    bool showRadio_     { false };
    bool showRoutines_  { false };
    bool showWifiSetup_ { false };
    bool chatAvailable_ { false };
    WifiSetupScreen wifiSetup_;
    

    RadioApp     radioApp;
    RoutinesApp  routinesApp;
    cv::Rect settingsButtonRect {0,0,0,0};

    // FIX-M: hoveredSettings y hoveredPower como atomic para acceso cross-thread seguro
    std::atomic<bool> hoveredSettings {false};
    std::atomic<bool> hoveredPower    {false};

    cv::Rect leftNavArrowRect  {0,0,0,0};
    cv::Rect rightNavArrowRect {0,0,0,0};
    bool hoveredLeftNav  {false};
    bool hoveredRightNav {false};
    cv::Rect powerButtonRect    {0,0,0,0};

    bool isEnglish = false;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr languageClient;
    static MenuItem MI(const char* id, const char* label, const char* sublabel, cv::Scalar color,
                       const char* cmd, const char* stop, bool hasSub, const char* subKey, const char* iconKey, std::vector<std::string> lc_nodes = {}) {
        return MenuItem(id, label, sublabel, color, cmd, stop, hasSub, subKey, iconKey, std::move(lc_nodes));
    }
    std::vector<std::string> menuPlaylist;
    Mix_Music* menuMusic { nullptr };
    bool isMenuMusicPlaying { false };
    // FIX-E: mutex compartido para todas las operaciones SDL_mixer
    std::mutex audioMutex_;

    void startMenuMusic() {
        std::lock_guard<std::mutex> lk(audioMutex_);
        // Detener musica de boot si aun suena
        if (bootMusic) {
            Mix_HaltMusic();
            Mix_FreeMusic(bootMusic);
            bootMusic = nullptr;
        }
        if (menuPlaylist.empty()) return;
        // FIX-G: siempre detener y liberar música anterior antes de cargar nueva
        if (menuMusic) {
            Mix_HaltMusic();
            Mix_FreeMusic(menuMusic);
            menuMusic = nullptr;
        }
        int idx = std::rand() % menuPlaylist.size();
        if (fs::exists(menuPlaylist[idx])) {
            menuMusic = Mix_LoadMUS(menuPlaylist[idx].c_str());
            if (!menuMusic) {
                RCLCPP_ERROR(get_logger(), "Mix_LoadMUS falló para música de menú: %s — %s",
                             menuPlaylist[idx].c_str(), Mix_GetError());
                return;
            }
            Mix_PlayMusic(menuMusic, -1);
            Mix_VolumeMusic(settingsMenu.volumeLevel);
            isMenuMusicPlaying = true;
        }
    }

    void stopMenuMusic() {
        std::lock_guard<std::mutex> lk(audioMutex_);
        if (isMenuMusicPlaying) {
            Mix_FadeOutMusic(500);
            isMenuMusicPlaying = false;
            // FIX-G: lanzar hilo que libera menuMusic tras el fade
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(600));
                std::lock_guard<std::mutex> lk2(audioMutex_);
                if (menuMusic && !Mix_PlayingMusic()) {
                    Mix_FreeMusic(menuMusic);
                    menuMusic = nullptr;
                }
            }).detach();
        }
    }

    void buildMenus() {
        std::string absCwd = std::filesystem::current_path().string();
        std::string mvDir  = absCwd + "/src/YAREN2/yaren_movements/yaren_movements/";
        std::string rdDir  = absCwd + "/src/YAREN2/yaren_radio/yaren_radio/";

        rootMenuItems = {
            MI("modo_prueba", isEnglish ? "TEST MODE" : "MODO PRUEBA", isEnglish ? "diagnostics and tests" : "diagnóstico y tests", {255,140,0}, "", "", true, "sub_modo_prueba", "test"),
            MI("yaren", "YAREN", isEnglish ? "main modes" : "modos principales", {0,229,255}, "", "", true, "sub_yaren", "yaren"),
        };
        subMenuMap["sub_modo_prueba"] = { isEnglish ? "TEST MODE" : "MODO PRUEBA", {255,140,0}, {
            MI("test_camara", isEnglish ? "CAMERA" : "CÁMARA", isEnglish ? "test camera" : "probar cámara", {0,200,255}, ("python " + absCwd + "/src/YAREN2/CSI-Camera/simple_camera.py &").c_str(), "", false, "", "camera"),
            MI("test_mic", isEnglish ? "MICROPHONE" : "MICRÓFONO", isEnglish ? "test microphone" : "probar micrófono", {0,255,128}, "", "", false, "", "microfono"),
            MI("test_motores", isEnglish ? "MOTORS" : "MOTORES", isEnglish ? "test motors" : "probar motores", {255,140,0}, "", "", true, "sub_motores", "motores"),
        }};
        subMenuMap["sub_motores"] = { isEnglish ? "TEST MOTORS" : "PROBAR MOTORES", {255,140,0}, {
            MI("motor_pos_orig", isEnglish ? "HOME POS." : "POS. ORIG.", isEnglish ? "initial position" : "posición inicial", {255,180,50}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "yaren"),
            MI("motor_brazo_izq", isEnglish ? "LEFT ARM" : "BRAZO IZQ", isEnglish ? "left arm" : "brazo izquierdo", {220,100,50}, "", "", true, "sub_brazo_izq", "brazo_izq"),
            MI("motor_brazo_der", isEnglish ? "RIGHT ARM" : "BRAZO DER", isEnglish ? "right arm" : "brazo derecho", {200,120,60}, "", "", true, "sub_brazo_der", "brazo_der"),
            MI("motor_base", "BASE", isEnglish ? "base rotation" : "giro de base", {180,100,80}, "", "", true, "sub_base", "girar_base"),
            MI("motor_cabeza", isEnglish ? "HEAD" : "CABEZA", isEnglish ? "head movement" : "movimiento cabeza", {160,80,100}, "", "", true, "sub_cabeza", "girar_cabeza"),
        }};
        subMenuMap["sub_brazo_izq"] = { isEnglish ? "LEFT ARM" : "BRAZO IZQUIERDO", {220,100,50}, {
            MI("brazo_izq_alto", isEnglish ? "UP" : "ARRIBA", isEnglish ? "up position" : "posición arriba", {220,110,55}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5,-3.0, 0.0, 3.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "arriba"),
            MI("brazo_izq_med", isEnglish ? "MIDDLE" : "MEDIO", isEnglish ? "middle position" : "posición media", {180,80,40}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.5, 0.0, 3.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "medio"),
            MI("brazo_izq_bajo", isEnglish ? "DOWN" : "BAJO", isEnglish ? "down position" : "posición baja", {200,90,45}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "abajo"),
        }};
        subMenuMap["sub_brazo_der"] = { isEnglish ? "RIGHT ARM" : "BRAZO DERECHO", {200,120,60}, {
            MI("brazo_der_alto", isEnglish ? "UP" : "ARRIBA", isEnglish ? "up position" : "posición arriba", {200,130,65}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "arriba"),
            MI("brazo_der_med", isEnglish ? "MIDDLE" : "MEDIO", isEnglish ? "middle position" : "posición media", {160,100,50}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "medio"),
            MI("brazo_der_bajo", isEnglish ? "DOWN" : "BAJO", isEnglish ? "down position" : "posición baja", {180,110,55}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "abajo"),
        }};
        subMenuMap["sub_base"] = { "BASE", {180,100,80}, {
            MI("base_izq", isEnglish ? "LEFT" : "IZQUIERDA", isEnglish ? "turn left" : "girar izquierda", {180,110,85}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [-1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "izquierda"),
            MI("base_der", isEnglish ? "RIGHT" : "DERECHA", isEnglish ? "turn right" : "girar derecha", {170,90,75}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "derecha"),
            MI("base_orig", isEnglish ? "HOME POS." : "POS. ORIGINAL", isEnglish ? "original position" : "posición original", {160,80,70}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "yaren"),
        }};
        subMenuMap["sub_cabeza"] = { isEnglish ? "HEAD" : "CABEZA", {160,80,100}, {
            MI("cabeza_izq", isEnglish ? "LEFT" : "IZQUIERDA", isEnglish ? "turn left" : "girar izquierda", {165,85,105}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, -0.8, 0.2, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "izquierda"),
            MI("cabeza_der", isEnglish ? "RIGHT" : "DERECHA", isEnglish ? "turn right" : "girar derecha", {150,70,90}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.8, 0.2, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "derecha"),
            MI("cabeza_orig", isEnglish ? "HOME POS." : "POS. ORIGINAL", isEnglish ? "original position" : "posición original", {140,60,80}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "yaren"),
        }};
        subMenuMap["sub_yaren"] = { "YAREN", {0,229,255}, {
            MI("yaren_mimic", "MIMIC", isEnglish ? "Yaren imitates you" : "Yaren te Imita",{0,229,255},"", "",   false, "", "mimic",{"csi_cam_node", "body_points_mimic_detector_node","body_tracker_node", "body_trajectory_controller", "mimic_gate_node"}),
            MI("yaren_ai", "IA", isEnglish ? "Artificial Intelligence" : "Inteligencia Artificial", {29,233,22}, "", "", true, "sub_yaren_ai", "chat"),            
            MI("yaren_juegos", isEnglish ? "GAMES" : "JUEGOS", isEnglish ? "Play with Yaren" : "Juega con Yaren", {255, 140, 50}, "", "", true, "sub_yaren_juegos", "juegos"),
            MI("yaren_movements", isEnglish ? "MOVEMENTS" : "MOVIMIENTOS", isEnglish ? "Yaren moves" : "Yaren se mueve", {251,64,224}, "", "", true, "sub_yaren_movements", "movements"),
            MI("yaren_emotions", isEnglish ? "EMOTIONS" : "EMOCIONES", isEnglish ? "Detects your emotion" : "Yaren detecta tu emoción", {82,82,255}, "", "", false, "", "emotions", {"csi_cam_node","detector"}),
            MI("yaren_filtros", isEnglish ? "FILTERS" : "FILTROS", isEnglish ? "Fun face filters" : "Yaren te pone filtros", {105,240,174}, "", "", true, "sub_yaren_filtros", "filtros"),
        }};
        subMenuMap["sub_yaren_ai"] = { isEnglish ? "ARTIFICIAL INTELLIGENCE" : "INTELIGENCIA ARTIFICIAL", {29,233,22}, {
            MI("yaren_chat", 
            isEnglish ? "CLOUD AI" : "IA EN LA NUBE", 
            isEnglish ? "Chat with Yaren using Groq" : "Chatea con Yaren usando Groq", 
            {29,233,22}, 
            "", 
            "", 
            false, 
            "", 
            "chat", 
            {"llm_lifecycle_node", "stt_lifecycle_node", "tts_lifecycle_node"}),
            
            MI("yaren_chat_local", 
            isEnglish ? "LOCAL AI" : "IA LOCAL", 
            isEnglish ? "Chat with Yaren using local LLM" : "Chatea con Yaren usando LLM local", 
            {255,200,50}, 
            "", 
            "", 
            false, 
            "", 
            "chat", 
            {"llm_local_lifecycle_node", "stt_local_lifecycle_node", "tts_local_lifecycle_node"}),
        }};
        subMenuMap["sub_yaren_dice"] = { isEnglish ? "YAREN SAYS" : "YAREN DICE", {64,171,255}, {
            MI("yaren_dice_sin_ayuda",
            isEnglish ? "WITHOUT HELP"  : "SIN AYUDA",
            isEnglish ? "No visual guide" : "Sin guía visual",
            {64,171,255},
            "ros2 run yaren_dice game_manager --ros-args -p use_help:=false &",
            "for pid in $(ps aux | grep -E 'game_manager' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done",
            false, "", "dice",
            {"csi_cam_node", "body_points_detector_node", "yaren_speaker_node"}),

            MI("yaren_dice_con_ayuda",
            isEnglish ? "WITH HELP"    : "CON AYUDA",
            isEnglish ? "Shows your pose" : "Muestra tu pose",
            {0,200,255},
            "ros2 run yaren_dice game_manager --ros-args -p use_help:=true &",
            "for pid in $(ps aux | grep -E 'game_manager' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done",
            false, "", "dice",
            {"csi_cam_node", "body_points_detector_node_visual", "yaren_speaker_node"}),

            MI("yaren_dice_sesion",
                isEnglish ? "SESSION" : "SESIÓN",
                isEnglish ? "Clinical record" : "Registro clínico",
                {100,200,100},
                "ros2 run yaren_dice game_manager --ros-args -p is_session:=true &",
                "for pid in $(ps aux | grep -E 'game_manager' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done",
                false, "", "dice",
                {"csi_cam_node", "body_points_detector_node", "yaren_speaker_node", "detector"}),
        }};
        subMenuMap["sub_yaren"].key = "sub_yaren";
        subMenuMap["sub_yaren_movements"] = { isEnglish ? "MOVEMENTS" : "MOVIMIENTOS", {251,64,224}, {
            MI("yaren_rutina1", isEnglish ? "ROUTINE 1" : "RUTINA 1", isEnglish ? "Routines" : "Rutinas", {251,64,224}, ("python3 " + mvDir + "yaren_movement.py &").c_str(), "for pid in $(ps aux | grep -E 'yaren_rutina1' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", false, "", "rutina1"),
            MI("yaren_rutina2", isEnglish ? "ROUTINE 2" : "RUTINA 2", isEnglish ? " Routines" : "Rutinas", {220,80,200}, ("python3 " + mvDir + "yaren_fullmovement.py &").c_str(), "for pid in $(ps aux | grep -E 'yaren_fullmovement' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", false, "", "rutina2"),
            MI("yaren_rutinanueva", isEnglish ? "PERSONAL ROUTINES" : "RUTINAS PERSONALES", isEnglish ? "manage and record" : "gestionar y grabar", {130, 80, 255}, "INTERNAL_ROUTINES", "", false, "", "rutinapersonal"),
        }};
        subMenuMap["sub_yaren_filtros"] = { isEnglish ? "FILTERS" : "FILTROS", {105,240,174}, {
            MI("yaren_animales", isEnglish ? "ANIMALS" : "ANIMALES", isEnglish ? "animal filter" : "filtro animal", {105,240,174}, "", "", false, "", "animales", {"csi_cam_node","face_landmark_publisher", "filtro_animales"}),
            MI("yaren_accesorios", isEnglish ? "ACCESSORIES" : "ACCESORIOS", isEnglish ? "accessories filter" : "filtro accesorios", {60,200,130}, "", "", false, "", "accesorios", {"csi_cam_node","face_landmark_publisher", "face_filter_node"}),
            MI("yaren_fondo", isEnglish ? "VIRT. BACKGROUNDS" : "FONDOS VIRTUAL", isEnglish ? "virtual background" : "filtro fondo virtual", {60,200,130}, "", "", false, "", "accesorios", {"csi_cam_node","virtual_background_node"}),
        }};
        subMenuMap["sub_yaren_p2"] = { "YAREN", {0,229,255}, {
            MI("yaren_radio", "YAREN RADIO", isEnglish ? "music and animation" : "música y animación", {255,80,160}, "", "", true, "sub_yaren_radio", "radio"),
        }};
        subMenuMap["sub_yaren_p2"].key = "sub_yaren_p2";

        subMenuMap["sub_yaren_juegos"] = { isEnglish ? "GAMES" : "JUEGOS", {255, 140, 50}, {
            MI("yaren_dice", isEnglish ? "SAYS" : "DICE", isEnglish ? "Play Yaren Says" : "Jugar Yaren Dice", {64,171,255}, "", "", true, "sub_yaren_dice", "dice"),
            MI("yaren_paint", isEnglish ? "PAINT" : "PINTURA", isEnglish ? "Draw and color" : "Dibuja y colorea", {255, 120, 90}, "", "", false, "", "paint", {"paint_node"}),
            MI("yaren_memoria", isEnglish ? "MEMORY GAME" : "JUEGO MEMORIA", isEnglish ? "Remember the colors" : "Acuérdate de los colores", {80, 200, 255}, "", "", false, "", "memoria", {"memoria_node"}),
            MI("yaren_palabras",isEnglish ? "HANGMAN" : "PALABRAS",isEnglish ? "Guess the word" : "Adivina la palabra",{220, 100, 255},"", "",false, "", "palabras",   {"palabras_node"}),
            MI("yaren_dance", isEnglish ? "DANCE GAME" : "JUEGO DE BAILE", isEnglish ? "Follow the rhythm" : "Sigue el ritmo", {50, 255, 150}, "", "", false, "", "juegobaile", {"csi_cam_node","dance_game_node"}),
        }};
        subMenuMap["sub_yaren_juegos"].key = "sub_yaren_juegos";
        subMenuMap["sub_yaren_radio"] = { "YAREN RADIO", {255,80,160}, {
            MI("radio_musica", isEnglish ? "MUSIC" : "MÚSICA", isEnglish ? "play music" : "reproducir música", {255,120,200}, "INTERNAL_RADIO", "", false, "", "musica"),
            
            // Lanza tu nueva App PyQt5
            MI("radio_videos", "YAREN TV", isEnglish ? "watch videos" : "ver videos", {200,60,140}, 
               ("nohup python3 " + rdDir + "tvplayer.py >/dev/null 2>&1 &").c_str(), 
               "for pid in $(ps aux | grep -E 'tvplayer.py' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", 
               false, "", "video"),
        }};
        recolorAllMenus();
    }
    bool checkChatAvailable() {
        // Reducido de 10 a 4 segundos
        const int TIMEOUT_SECS  = 4; 
        const int POLL_INTERVAL = 1;   // segundos entre intentos

        const char* key = std::getenv("GROQ_API_KEY");
        bool key_ok     = (key != nullptr && strlen(key) > 10);

        if (!key_ok) {
            RCLCPP_WARN(get_logger(),
                "[ WiFi ] GROQ_API_KEY no configurada — chat deshabilitado.");
            return false;
        }

        RCLCPP_INFO(get_logger(),
            "[ WiFi ] Buscando conexion a internet (maximo %d segundos)...",
            TIMEOUT_SECS);

        for (int elapsed = 0; elapsed < TIMEOUT_SECS; elapsed += POLL_INTERVAL) {
            // Verificar estado de red con nmcli (rapido, sin bloqueo prolongado)
            FILE* p = popen("nmcli -t -f STATE general 2>/dev/null", "r");
            char buf[64] = {};
            if (p) { fgets(buf, sizeof(buf), p); pclose(p); }
            bool nmcli_ok = std::string(buf).find("connected") != std::string::npos;

            if (nmcli_ok) {
                // nmcli dice conectado → verificar ping real
                // Se redujo el timeout de curl a 2 segundos (-m 2) para no trabar el bucle
                bool ping_ok = (std::system("curl -s -I -m 2 https://api.groq.com > /dev/null 2>&1") == 0);    

                if (ping_ok) {
                    RCLCPP_INFO(get_logger(),
                        "[ WiFi ] ✓ Internet disponible (tras %d segundo(s)).",
                        elapsed + POLL_INTERVAL);
                    return true;
                }
            }

            RCLCPP_INFO(get_logger(),
                "[ WiFi ] Sin internet... reintentando (%d/%d)",
                elapsed + POLL_INTERVAL, TIMEOUT_SECS);

            std::this_thread::sleep_for(std::chrono::seconds(POLL_INTERVAL));
        }

        RCLCPP_WARN(get_logger(),
            "[ WiFi ] ✗ No se encontro internet tras %d segundos.", TIMEOUT_SECS);
        return false;
    }


    bool runCommand(const std::string& cmd) {
        if (cmd.empty()) return false;
        int ret = std::system(cmd.c_str());
        return (ret == 0);
    }

    void showCustomOverlay(FaceOverlay type, const std::string& msg, double secs) {
        {
            std::lock_guard<std::mutex> lock(overlayMutex);
            faceOverlay    = type;
            overlayMessage = msg;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(secs));
        {
            std::lock_guard<std::mutex> lock(overlayMutex);
            if (faceOverlay == type && overlayMessage == msg) {
                faceOverlay    = FaceOverlay::NONE;
                overlayMessage = "";
            }
        }
    }

    void showErrorOverlay(const std::string& msg, double secs = 3.0) {
        showCustomOverlay(FaceOverlay::ERROR_MSG, msg, secs);
    }

    void executeMicTest() {
        if (micTestRunning.load()) return;
        micTestRunning = true;

        // Lanzar TODA la lógica en un hilo independiente (detached)
        // Esto evita congelar la UI y previene el deadlock con navMutex.
        std::thread([this]() {
            {
                auto m = std_msgs::msg::String();
                m.data = "mic_test";
                micOwnerPublisher_->publish(m);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            // 1. Avisar al wake_word_node ANTES de liberar el mic
            {
                auto testMsg = std_msgs::msg::Bool();
                testMsg.data = true;
                micTestPublisher_->publish(testMsg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            // 2. Liberar face_idle
            {
                auto idleMsg = std_msgs::msg::Bool();
                idleMsg.data = true;
                idleStatePublisher->publish(idleMsg);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            {
                std::lock_guard<std::mutex> lock(overlayMutex);
                faceOverlay      = FaceOverlay::MIC_COUNTDOWN;
                micCountdownSecs = 5;
                overlayMessage   = isEnglish ? "Speak for the next 5 seconds"
                                            : "Habla por los siguientes 5 segundos";
            }

            stopMenuMusic();

            {
                // Ahora esto es seguro porque el hilo principal ya soltó el navMutex
                std::lock_guard<std::mutex> nlock(navMutex);
                navStack.clear();
                hoveredItem = -1;
                hoveredBack = hoveredStop = hoveredExit = false;
            }

            std::string micId = settingsMenu.selectedMicId;
            std::string spkId = settingsMenu.selectedSpkId;
            std::string envMic = micId.empty() ? "" : "PULSE_SOURCE='" + micId + "' ";
            std::string recordCmd = envMic +
                "arecord -D pulse -f S16_LE -r 44100 -c 1 -d 5 "
                "/tmp/yaren_mic_test.wav > /tmp/yaren_arecord.log 2>&1";

            std::atomic<int> recRet{-1};
            std::thread recordingThread([&]() {
                recRet = std::system(recordCmd.c_str());
            });

            for (int i = 5; i >= 1; --i) {
                {
                    std::lock_guard<std::mutex> lock(overlayMutex);
                    faceOverlay      = FaceOverlay::MIC_COUNTDOWN;
                    micCountdownSecs = i;
                    overlayMessage   = (isEnglish
                        ? "Speak for the next "
                        : "Habla por los siguientes ")
                        + std::to_string(i)
                        + (isEnglish ? (i == 1 ? " second" : " seconds")
                                    : (i == 1 ? " segundo" : " segundos"));
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            recordingThread.join();

            if (recRet.load() != 0) {
                showErrorOverlay(
                    isEnglish ? "Failed to record audio.\nCheck the microphone."
                            : "Fallo al grabar audio.\nRevisa el micrófono.", 4.0);
                {
                    std::lock_guard<std::mutex> lock(overlayMutex);
                    faceOverlay = FaceOverlay::NONE;
                }
                auto testMsg = std_msgs::msg::Bool();
                testMsg.data = false;
                micTestPublisher_->publish(testMsg);
                micTestRunning = false;
                return;
            }

            {
                std::lock_guard<std::mutex> lock(overlayMutex);
                faceOverlay    = FaceOverlay::NONE;
                overlayMessage = "";
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));

            {
                std::lock_guard<std::mutex> lock(overlayMutex);
                faceOverlay    = FaceOverlay::MIC_PLAYING;
                overlayMessage = isEnglish ? "Playing audio..." : "Reproduciendo audio...";
            }

            std::string envSpk = spkId.empty() ? "" : "PULSE_SINK='" + spkId + "' ";
            std::string playCmd = envSpk +
                "aplay -D pulse /tmp/yaren_mic_test.wav > /tmp/yaren_aplay.log 2>&1";
            int playRet = std::system(playCmd.c_str());

            if (playRet != 0) {
                showErrorOverlay(
                    isEnglish ? "Failed to play audio.\nCheck the speaker."
                            : "Fallo al reproducir audio.\nRevisa el parlante.", 4.0);
            }

            {
                std::lock_guard<std::mutex> lock(overlayMutex);
                faceOverlay      = FaceOverlay::NONE;
                overlayMessage   = "";
                micCountdownSecs = 0;
            }

            {
                std::lock_guard<std::mutex> nlock(navMutex);
                navStack.clear();
                NavLevel root;
                root.title       = isEnglish ? "MAIN MENU" : "MENÚ PRINCIPAL";
                root.accentColor = {0, 200, 200};
                root.items       = rootMenuItems;
                navStack.push_back(root);
                auto it = subMenuMap.find("sub_modo_prueba");
                if (it != subMenuMap.end()) {
                    NavLevel lvl = it->second;
                    lvl.key = "sub_modo_prueba";
                    navStack.push_back(lvl);
                }
                hoveredItem = -1;
                hoveredBack = false;
                hoveredStop = false;
                hoveredExit = false;
            }

            {
                auto m = std_msgs::msg::String();
                m.data = "none";
                micOwnerPublisher_->publish(m);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            startMenuMusic();
            micTestRunning = false;
        }).detach();
    }
    void onRemoteCommand(const std_msgs::msg::String::SharedPtr msg) {
        const std::string cmd = msg->data;
        RCLCPP_INFO(get_logger(), "[ REMOTE ] comando: %s", cmd.c_str());
        if (configuring.load()) return;      // ignorar mientras arranca
        resetIdleTimer();

        bool sSettings, sRadio, sRoutines, sWifi;
        {
            std::lock_guard<std::mutex> lk(modeFlagMutex);
            sSettings = showSettings_; sRadio = showRadio_;
            sRoutines = showRoutines_; sWifi  = showWifiSetup_;
        }

        // --- pantallas especiales ---
        if (cmd == "power_off") {
            showErrorOverlay(isEnglish ? "Shutting down robot..." : "Apagando robot...", 5.0);
            std::system("sudo poweroff &");
            return;
        }
        if (cmd == "open_settings") {
            std::lock_guard<std::mutex> lk(modeFlagMutex);
            showSettings_ = true; settingsMenu.refresh();
            return;
        }
        if (cmd == "back" || (cmd == "stop" && (sRadio || sRoutines))) {
            if (sSettings) { std::lock_guard<std::mutex> lk(modeFlagMutex); showSettings_ = false; return; }
            if (sWifi)     { std::lock_guard<std::mutex> lk(modeFlagMutex); showWifiSetup_ = false; return; }
            if (sRadio)    { if (radioApp.onBack) radioApp.onBack(); return; }
            if (sRoutines) { if (routinesApp.onBack) routinesApp.onBack(); return; }
        }
        if (sSettings || sRadio || sRoutines || sWifi) return; // solo "back" aplica ahí

        if (cmd == "stop") {                 // equivale al botón DETENER
            std_msgs::msg::String m; m.data = "idle";
            modePublisher->publish(m);
            return;
        }

        // --- navegación del menú ---
        std::lock_guard<std::mutex> lock(navMutex);

        auto ensureRoot = [&]() {
            if (navStack.empty()) {
                NavLevel root;
                root.title       = isEnglish ? "MAIN MENU" : "MENÚ PRINCIPAL";
                root.accentColor = {0, 200, 200};
                root.items       = rootMenuItems;
                navStack.push_back(root);
                centerCarouselOnEmotions();
                hoveredItem = -1;
                startMenuMusic();
            }
        };
        auto pushSub = [&](const std::string& key) {
            auto it = subMenuMap.find(key);
            if (it == subMenuMap.end()) {
                RCLCPP_WARN(get_logger(), "[ REMOTE ] submenu desconocido: %s", key.c_str());
                return;
            }
            if (!navStack.empty() && navStack.back().key == key) return;
            NavLevel lvl = it->second; lvl.key = key;
            navStack.push_back(lvl);
            hoveredItem = -1;
        };

        if (cmd == "open_menu") {
            ensureRoot();
        } else if (cmd == "go_home") {
            navStack.clear();
            hoveredItem = -1;
            ensureRoot(); 
        }else if (cmd == "back") {
            if (navStack.size() > 1) navStack.pop_back();
            hoveredItem = -1;
        } else if (cmd == "exit") {
            navStack.clear(); hoveredItem = -1; stopMenuMusic();
        } else if (cmd.rfind("open_submenu_", 0) == 0) {
            ensureRoot();
            pushSub(cmd.substr(13));
        } else if (cmd.rfind("select_", 0) == 0) {
            // ejecuta un item del menú por su id (igual que tocarlo)
            std::string id = cmd.substr(7);
            MenuItem item = findMenuItem(id);
            if (item.id.empty()) {
                RCLCPP_WARN(get_logger(), "[ REMOTE ] item desconocido: %s", id.c_str());
                return;
            }
            if (item.id == "yaren_chat" && !chatAvailable_) { lastUIState_.clear(); return; }  // fuerza reenviar el estado actual            
            if (item.hasSubMenu) { ensureRoot(); pushSub(item.subMenuKey); }
            else                 { executeMode(item); }
        } else {
            RCLCPP_WARN(get_logger(), "[ REMOTE ] comando no reconocido: %s", cmd.c_str());
        }
    }
    void handleMouse(int event, int x, int y) {
        if (isIdleScreenActive) {
            if (event == cv::EVENT_LBUTTONDOWN) {
                resetIdleTimer(); // Esto despierta la pantalla
            }
            
        }
        // 2. Si la pantalla ya está despierta, cualquier interacción (movimiento o clic) 
        // reinicia el temporizador para que no se duerma mientras la usas
        resetIdleTimer();        
        bool sSettings, sRadio, sRoutines, sWifi;
        {
            std::lock_guard<std::mutex> lk(modeFlagMutex);
            sSettings  = showSettings_;
            sRadio     = showRadio_;
            sRoutines  = showRoutines_;
            sWifi      = showWifiSetup_;
        }
        if (sSettings) {
            settingsMenu.handleMouse(event, x, y);
            return;
        }
        if (sRadio) {
            radioApp.handleMouse(event, x, y);
            return;
        }
        if (sRoutines) {
            routinesApp.handleMouse(event, x, y);
            return;
        }
        if (sWifi) {
            wifiSetup_.handleMouse(event, x, y);
            return;
        }

        // FIX-M: hoveredSettings/hoveredPower son atomic<bool>, safe sin lock adicional
        if (event == cv::EVENT_MOUSEMOVE) {
            hoveredSettings = settingsButtonRect.contains({x, y});
            hoveredPower    = powerButtonRect.contains({x, y});
        }

        std::lock_guard<std::mutex> lock(navMutex);

        if (event == cv::EVENT_LBUTTONDOWN) {
            if (settingsButtonRect.contains({x, y})) {
                std::lock_guard<std::mutex> lk(modeFlagMutex);
                showSettings_ = true;
                settingsMenu.refresh();
                return;
            }
            if (powerButtonRect.contains({x, y})) {
                showErrorOverlay(isEnglish ? "Shutting down robot..." : "Apagando robot...", 5.0);
                std::system("sudo poweroff &");
                return;
            }
        }

        if (!navStack.empty()) {
            if (event == cv::EVENT_MOUSEMOVE) {
                hoveredLeftNav  = (leftNavArrowRect.area()  > 0) && leftNavArrowRect.contains({x,y});
                hoveredRightNav = (rightNavArrowRect.area() > 0) && rightNavArrowRect.contains({x,y});
            }
            if (event == cv::EVENT_LBUTTONDOWN) {
                if (rightNavArrowRect.area() > 0 && rightNavArrowRect.contains({x,y})) {
                    auto it = subMenuMap.find("sub_yaren_p2");
                    if (it != subMenuMap.end()) {
                        navStack.push_back(it->second);
                        hoveredItem = -1; hoveredRightNav = false;
                    }
                    return;
                }
                if (leftNavArrowRect.area() > 0 && leftNavArrowRect.contains({x,y})) {
                    if (navStack.size() > 1) {
                        navStack.pop_back();
                        hoveredItem = -1; hoveredLeftNav = false;
                    }
                    return;
                }
            }
        }

        if (navStack.empty()) {
            if (event == cv::EVENT_LBUTTONDOWN) {
                NavLevel root;
                root.title       = isEnglish ? "MAIN MENU" : "MENÚ PRINCIPAL";
                root.accentColor = { 0, 200, 200 };
                root.items       = rootMenuItems;
                navStack.push_back(root);
                centerCarouselOnEmotions();
                hoveredItem = -1;
                hoveredBack = hoveredStop = hoveredExit = false;
                startMenuMusic();
            }
            return;
        }
        if (navStack.size() == 1 && carouselMouse(event, x, y)) return;
        auto& level = navStack.back();
        if (event == cv::EVENT_MOUSEMOVE) {
            hoveredItem = -1;
            for (int i = 0; i < (int)level.items.size(); ++i)
                if (level.items[i].rect.contains({ x, y })) { hoveredItem = i; break; }
            hoveredBack = (navStack.size() > 1) && backButtonRect.contains({ x, y });
            hoveredExit = exitButtonRect.contains({ x, y });
            hoveredStop = !activeMode.empty() && stopButtonRect.contains({ x, y });
        }
        if (event == cv::EVENT_LBUTTONDOWN) {
            if (exitButtonRect.contains({ x, y })) {
                navStack.clear(); hoveredItem = -1; hoveredBack = hoveredStop = hoveredExit = false;
                stopMenuMusic();
                return;
            }
            if (navStack.size() > 1 && backButtonRect.contains({ x, y })) {
                navStack.pop_back(); hoveredItem = -1; hoveredBack = false;
                return;
            }
            if (!activeMode.empty() && stopButtonRect.contains({ x, y })) {
                std::string prevMode = activeMode;
                for (const auto& node : active_lifecycle_nodes) {
                    change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                }
                active_lifecycle_nodes.clear();
                std::string stopCmd = activeStopCmd;
                activeMode = ""; activeStopCmd = "";
                navStack.clear();
                hoveredItem = -1; hoveredBack = hoveredStop = hoveredExit = false;
                stopMenuMusic();
                // FIX-I: publicar y ejecutar stop fuera del lock no es posible aquí
                // porque estamos bajo navMutex. Lanzamos en thread para el system call.
                auto msg = std_msgs::msg::String(); msg.data = "idle";
                modePublisher->publish(msg);
                if (!stopCmd.empty()) {
                    std::thread([stopCmd]() { std::system(stopCmd.c_str()); }).detach();
                }
                if (prevMode.rfind("vid_", 0) == 0) {
                    NavLevel root; root.title = isEnglish ? "MAIN MENU" : "MENÚ PRINCIPAL";
                    root.accentColor = {0,200,200}; root.items = rootMenuItems;
                    navStack.push_back(root);
                    for (const std::string& key : {"sub_yaren_p2", "sub_yaren_radio", "sub_yaren_videos"}) {
                        auto it = subMenuMap.find(key);
                        if (it != subMenuMap.end()) {
                            NavLevel lvl = it->second;
                            lvl.key = key;
                            navStack.push_back(lvl);
                        }
                    }
                    startMenuMusic();
                }
                return;
            }
            // Click en modo bloqueado → abrir WiFi setup
            for (int i = 0; i < (int)level.items.size(); ++i) {
                if (level.items[i].rect.contains({x,y})) {
                    bool blocked = (level.items[i].id == "yaren_chat") && !chatAvailable_;
                    if (blocked) {
                        wifiSetup_.refresh();
                        std::lock_guard<std::mutex> lk(modeFlagMutex);
                        showWifiSetup_ = true;
                        return;
                    }
                    break;
                }
            }
            for (int i = 0; i < (int)level.items.size(); ++i) {
                if (level.items[i].rect.contains({ x, y })) {
                    if (level.items[i].hasSubMenu) {
                        auto it = subMenuMap.find(level.items[i].subMenuKey);
                        if (it != subMenuMap.end()) {
                            NavLevel lvl = it->second;
                            lvl.key = level.items[i].subMenuKey;
                            navStack.push_back(lvl);
                            hoveredItem = -1;
                        }
                    } else {
                        executeMode(level.items[i]);
                    }
                    return;
                }
            }
        }
    }

    MenuItem findMenuItem(const std::string& target_id) {
        for (const auto& item : rootMenuItems) {
            if (item.id == target_id) return item;
        }
        for (const auto& pair : subMenuMap) {
            for (const auto& item : pair.second.items) {
                if (item.id == target_id) return item;
            }
        }
        return MenuItem();
    }

    void executeMode(MenuItem item, bool publish_mode = true) {
        if (item.id == "test_mic") { executeMicTest(); return; }
        if (item.cmd == "INTERNAL_RADIO") {
            { std::lock_guard<std::mutex> lk(modeFlagMutex); showRadio_ = true; }
            radioApp.reset();
            hoveredItem = -1;
            return;
        }
        if (item.cmd == "INTERNAL_ROUTINES") {
            { std::lock_guard<std::mutex> lk(modeFlagMutex); showRoutines_ = true; }
            routinesApp.refresh();
            hoveredItem = -1;
            stopMenuMusic();
            return;
        }

        // Cierra el menú en la UI instantáneamente para que responda rápido
        navStack.clear();
        stopMenuMusic();
        hoveredItem = -1;
        hoveredBack = hoveredStop = hoveredExit = false;

        std::string cleanCmd = item.cmd;
        size_t pos = cleanCmd.find_last_not_of(" \t&");
        if (pos != std::string::npos) cleanCmd = cleanCmd.substr(0, pos + 1);

        // === OPCIÓN C: EJECUCIÓN EN SEGUNDO PLANO ===
        std::thread([this, item, cleanCmd, publish_mode]() {
            
            // 1. Mostrar cartel de "Iniciando..." (solo si el modo usa nodos)
            if (!item.lifecycle_nodes.empty()) {
                std::lock_guard<std::mutex> lock(overlayMutex);
                faceOverlay = FaceOverlay::MIC_PLAYING; // Animación de ondas como indicador de carga
                overlayMessage = isEnglish ? "Starting " + item.label + "..." : "Iniciando " + item.label + "...";
            }

            // 2. Apagar el modo anterior si existe
            if (!activeMode.empty()) {
                for (const auto& node : active_lifecycle_nodes) {
                    change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
                }
                active_lifecycle_nodes.clear();
                
                std::string prevStop = activeStopCmd;
                if (!prevStop.empty()) {
                    std::system(prevStop.c_str());
                }
            }

            // 3. Guardar el nuevo modo
            activeMode = item.id;
            activeStopCmd = item.stopCmd;
            active_lifecycle_nodes = item.lifecycle_nodes;

            // 4. Configurar y activar los nuevos nodos
            for (const auto& node : active_lifecycle_nodes) {
                ensure_lifecycle_active(node);
            }

            // 5. Quitar el cartel de "Iniciando..."
            if (!item.lifecycle_nodes.empty()) {
                std::lock_guard<std::mutex> lock(overlayMutex);
                faceOverlay = FaceOverlay::NONE;
                overlayMessage = "";
            }

            // 6. Avisarle al resto de ROS 2 que ya cambiamos de modo
            if (publish_mode) {
                auto msg = std_msgs::msg::String();
                msg.data = activeMode;
                modePublisher->publish(msg);
            }

            // 7. Ejecutar el script/comando del modo si lo hay
            if (!cleanCmd.empty()) {
                std::system(cleanCmd.c_str());   // bloquea hasta que el programa termina

                // Terminó (cerraron tvplayer, acabó la rutina, etc.): si seguimos en este modo, volver a idle
                if (activeMode == item.id) {
                    std_msgs::msg::String m;
                    m.data = "idle";
                    modePublisher->publish(m);
                }
            }

        }).detach(); // <- Esto "libera" el hilo para que corra por su cuenta sin trabar la pantalla
    }
    void renderSettingsButton(cv::Mat& frame) {
        int W = frame.cols, btnSz = 44, margin = 10;
        settingsButtonRect = {W - btnSz - margin, margin, btnSz, btnSz};
        bool hs = hoveredSettings.load();
        const cv::Scalar white(255, 255, 255);
        cv::Point c(settingsButtonRect.x + btnSz / 2, settingsButtonRect.y + btnSz / 2);
        cv::Scalar g = hs ? cv::Scalar(120, 165, 210) : cv::Scalar(95, 140, 190);
        cv::circle(frame, cv::Point(c.x, c.y + 3), btnSz / 2, g * 0.5, cv::FILLED, cv::LINE_AA);
        cv::circle(frame, c, btnSz / 2, g, cv::FILLED, cv::LINE_AA);
        cv::circle(frame, c, btnSz / 2, white, 3, cv::LINE_AA);
        for (int k = 0; k < 8; ++k) {
            double ang = k * CV_PI / 4.0;
            cv::Point p1((int)(c.x + 9 * std::cos(ang)),  (int)(c.y + 9 * std::sin(ang)));
            cv::Point p2((int)(c.x + 13 * std::cos(ang)), (int)(c.y + 13 * std::sin(ang)));
            cv::line(frame, p1, p2, white, 4, cv::LINE_AA);
        }
        cv::circle(frame, c, 9, white, cv::FILLED, cv::LINE_AA);
        cv::circle(frame, c, 4, g, cv::FILLED, cv::LINE_AA);
    }

    void renderSideNavArrow(cv::Mat& frame, const cv::Rect& r, bool pointLeft, const std::string& label, bool hovered) {
        cv::Mat ov = frame.clone();
        cv::rectangle(ov, r, cv::Scalar(10, 5, 25), cv::FILLED);
        cv::addWeighted(ov, 0.80, frame, 0.20, 0, frame);
        cv::rectangle(frame, r, hovered ? cv::Scalar(255,120,210) : cv::Scalar(100,40,120), hovered ? 2 : 1, cv::LINE_AA);
        int cx = r.x + r.width / 2, cy = r.y + r.height / 2 - 12, sz = 14;
        cv::Scalar arrowColor = hovered ? cv::Scalar(255,180,230) : cv::Scalar(200,80,160);
        std::vector<cv::Point> tri;
        if (pointLeft) tri = { {cx + sz, cy - sz}, {cx - sz, cy}, {cx + sz, cy + sz} };
        else           tri = { {cx - sz, cy - sz}, {cx + sz, cy}, {cx - sz, cy + sz} };
        cv::fillPoly(frame, tri, arrowColor);
        int bl = 0; double scale = 0.70;
        cv::Size ts = txu::size(label, cv::FONT_HERSHEY_PLAIN, scale, 1, &bl);
        txu::put(frame, label, { r.x + (r.width - ts.width) / 2, r.y + r.height - 8 },
                    cv::FONT_HERSHEY_PLAIN, scale, hovered ? cv::Scalar(255,255,255) : cv::Scalar(180,100,160), 1, cv::LINE_AA);
    }

    void renderPowerButton(cv::Mat& frame) {
        int btnSz = 44, margin = 10;
        powerButtonRect = {margin, margin, btnSz, btnSz};
        bool hp = hoveredPower.load();
        const cv::Scalar white(255, 255, 255);
        cv::Point c(margin + btnSz / 2, margin + btnSz / 2);
        cv::Scalar red = hp ? cv::Scalar(40, 165, 255) : cv::Scalar(0, 120, 255);
        cv::circle(frame, cv::Point(c.x, c.y + 3), btnSz / 2, red * 0.5, cv::FILLED, cv::LINE_AA);
        cv::circle(frame, c, btnSz / 2, red, cv::FILLED, cv::LINE_AA);
        cv::circle(frame, c, btnSz / 2, white, 3, cv::LINE_AA);
        cv::ellipse(frame, c, cv::Size(10, 10), -90, 30, 330, white, 3, cv::LINE_AA);
        cv::line(frame, cv::Point(c.x, c.y - 11), cv::Point(c.x, c.y + 1), white, 3, cv::LINE_AA);
    }

    // ======================= NUEVO DISENO DEL MENU (carrusel suave) =======================
    cv::Scalar menuGlowColor_ {200, 150, 60};
    cv::Point2f menuGlowPos_  {400.f, 240.f};
    std::array<float, 16> menuHoverAnim_ {};

    static void rrPath(std::vector<cv::Point>& pts, const cv::Rect& r, int rad) {
        rad = std::max(1, std::min(rad, std::min(r.width, r.height) / 2));
        auto arc = [&](int cx, int cy, int a0) {
            for (int a = a0; a <= a0 + 90; a += 10)
                pts.push_back({cx + (int)std::lround(rad * std::cos(a * CV_PI / 180.0)),
                               cy + (int)std::lround(rad * std::sin(a * CV_PI / 180.0))});
        };
        arc(r.x + r.width - rad - 1, r.y + rad,                 270);
        arc(r.x + r.width - rad - 1, r.y + r.height - rad - 1,  0);
        arc(r.x + rad,               r.y + r.height - rad - 1,  90);
        arc(r.x + rad,               r.y + rad,                 180);
    }

    static void fillRound(cv::Mat& f, const cv::Rect& r, int rad, const cv::Scalar& c, double alpha = 1.0) {
        cv::Rect rr = r & cv::Rect(0, 0, f.cols, f.rows);
        if (rr.area() <= 0) return;
        std::vector<cv::Point> p; rrPath(p, r, rad);
        if (alpha >= 0.99) { cv::fillPoly(f, p, c, cv::LINE_AA); return; }
        cv::Mat roi = f(rr);
        cv::Mat ov = roi.clone();
        std::vector<cv::Point> q;
        for (auto& pt : p) q.push_back({pt.x - rr.x, pt.y - rr.y});
        cv::fillPoly(ov, q, c, cv::LINE_AA);
        cv::addWeighted(ov, alpha, roi, 1.0 - alpha, 0, roi);
    }

    static void gradRound(cv::Mat& f, const cv::Rect& r, int rad,
                          const cv::Scalar& top, const cv::Scalar& bot, double alpha = 1.0) {
        cv::Rect rr = r & cv::Rect(0, 0, f.cols, f.rows);
        if (rr.area() <= 0) return;
        cv::Mat roi = f(rr);
        cv::Mat grad(rr.size(), CV_8UC3);
        for (int y = 0; y < rr.height; ++y) {
            double t = (double)(y + rr.y - r.y) / std::max(1, r.height - 1);
            t = std::max(0.0, std::min(1.0, t));
            grad.row(y).setTo(top * (1.0 - t) + bot * t);
        }
        std::vector<cv::Point> p; rrPath(p, r, rad);
        std::vector<cv::Point> q;
        for (auto& pt : p) q.push_back({pt.x - rr.x, pt.y - rr.y});
        cv::Mat mask = cv::Mat::zeros(rr.size(), CV_8UC1);
        cv::fillPoly(mask, q, cv::Scalar(255), cv::LINE_AA);
        cv::Mat blended;
        cv::addWeighted(grad, alpha, roi, 1.0 - alpha, 0, blended);
        blended.copyTo(roi, mask);
    }

    static void strokeRound(cv::Mat& f, const cv::Rect& r, int rad, const cv::Scalar& c, int thick = 1) {
        std::vector<cv::Point> p; rrPath(p, r, rad);
        cv::polylines(f, p, true, c, thick, cv::LINE_AA);
    }

    void blitIcon(cv::Mat& frame, const cv::Mat& original, cv::Point center, int size) {
        if (original.empty() || size < 4) return;
        float asp = (float)original.cols / (float)original.rows;
        int iw, ih;
        if (asp >= 1.0f) { iw = size; ih = std::max(1, (int)(size / asp)); }
        else             { ih = size; iw = std::max(1, (int)(size * asp)); }
        cv::Mat icon;
        cv::resize(original, icon, {iw, ih}, 0, 0, cv::INTER_AREA);
        int x0 = center.x - iw / 2, y0 = center.y - ih / 2;
        cv::Rect dst(x0, y0, iw, ih);
        dst &= cv::Rect(0, 0, frame.cols, frame.rows);
        if (dst.area() <= 0) return;
        cv::Rect src(dst.x - x0, dst.y - y0, dst.width, dst.height);
        cv::Mat roi = frame(dst), crop = icon(src);
        if (icon.channels() == 4) {
            for (int ry = 0; ry < dst.height; ++ry)
                for (int rx = 0; rx < dst.width; ++rx) {
                    cv::Vec4b px = crop.at<cv::Vec4b>(ry, rx);
                    float a = px[3] / 255.f;
                    if (a > 0.f) {
                        cv::Vec3b& bg = roi.at<cv::Vec3b>(ry, rx);
                        for (int c = 0; c < 3; ++c)
                            bg[c] = cv::saturate_cast<uchar>(px[c] * a + bg[c] * (1.f - a));
                    }
                }
        } else if (icon.channels() == 3) {
            crop.copyTo(roi);
        }
    }

    void drawMenuBackground(cv::Mat& frame, const cv::Scalar& gTarget, cv::Point2f gPos, double t) {
        (void)gPos;
        crBackground(frame, gTarget, frame.rows - 100, t);
    }

    void drawPill(cv::Mat& frame, const cv::Rect& r, const std::string& txt,
                  const cv::Scalar& accent, bool hov, double maxScale = 0.5) {
        crPill(frame, r, txt, accent, hov, maxScale);
    }

    void renderRoundNavArrow(cv::Mat& frame, const cv::Rect& r, bool left,
                             const std::string& label, bool hov) {
        cv::Point c(r.x + r.width / 2, r.y + r.height / 2 - 10);
        int R = hov ? 25 : 23;
        cv::circle(frame, {c.x, c.y + 3}, R, cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_AA);
        cv::circle(frame, c, R, hov ? cv::Scalar(70, 60, 50) : cv::Scalar(34, 28, 24), cv::FILLED, cv::LINE_AA);
        cv::circle(frame, c, R, hov ? cv::Scalar(255, 200, 120) : cv::Scalar(120, 95, 70), hov ? 2 : 1, cv::LINE_AA);
        int s = left ? -1 : 1;
        std::vector<cv::Point> ch = { {c.x - s * 4, c.y - 9}, {c.x + s * 5, c.y}, {c.x - s * 4, c.y + 9} };
        cv::polylines(frame, ch, false, hov ? cv::Scalar(255, 255, 255) : cv::Scalar(230, 150, 255), 3, cv::LINE_AA);
        int bl = 0;
        cv::Size ts = txu::size(label, cv::FONT_HERSHEY_PLAIN, 0.8, 1, &bl);
        txu::put(frame, label, {c.x - ts.width / 2, c.y + R + 18}, cv::FONT_HERSHEY_PLAIN, 0.8,
                    cv::Scalar(170, 160, 175), 1, cv::LINE_AA);
    }

    // ================= MENU CARRUSEL PLANO (todos los modos de una) =================
    struct CarEntry { MenuItem item; std::string group; };
    std::vector<CarEntry> carousel_;
    bool  carBuiltEnglish_ {false};
    int   carSel_ {0};
    float carPos_ {0.f};
    bool  carDown_ {false}, carDragging_ {false};
    int   carDownX_ {0}, carDownIdx_ {-1}, carStride_ {180};
    float carDownPos_ {0.f};
    int   carHover_ {-1};
    bool  carEnterHot_ {false};
    cv::Rect carEnterRect_ {0,0,0,0}, carPrevBtn_ {0,0,0,0}, carNextBtn_ {0,0,0,0};
    std::vector<std::pair<int, cv::Rect>> carHit_;
    cv::Scalar carGlow_ {200, 150, 60};
    std::map<std::string, double> iconHueCache_;

    // ---- iconos en disco circular (cacheados) ----
    std::map<std::string, cv::Mat>  discCache_;
    std::map<std::string, bool>     coverCache_;

    // true = foto/render opaco (se recorta "cover" y llena el disco)
    // false = icono con transparencia (se ajusta "contain" dentro del disco)
    bool isCoverIcon(const std::string& key) {
        auto c = coverCache_.find(key);
        if (c != coverCache_.end()) return c->second;
        bool cover = true;
        auto it = iconMap.find(key);
        if (it != iconMap.end() && !it->second.empty() && it->second.channels() == 4) {
            const cv::Mat& im = it->second;
            cover = im.at<cv::Vec4b>(0, 0)[3] > 250 &&
                    im.at<cv::Vec4b>(0, im.cols - 1)[3] > 250 &&
                    im.at<cv::Vec4b>(im.rows - 1, 0)[3] > 250 &&
                    im.at<cv::Vec4b>(im.rows - 1, im.cols - 1)[3] > 250;
        }
        return coverCache_[key] = cover;
    }

    const cv::Mat& getDisc(const std::string& key, int D) {
        D = std::max(16, ((D + 7) / 8) * 8);          // cuantizar: evita llenar la cache
        const std::string ck = key + "@" + std::to_string(D);
        auto hit = discCache_.find(ck);
        if (hit != discCache_.end()) return hit->second;

        cv::Mat out(D, D, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        auto it = iconMap.find(key);
        if (it != iconMap.end() && !it->second.empty()) {
            const cv::Mat& im = it->second;
            cv::Mat bgra;
            if (im.channels() == 4)      bgra = im;
            else if (im.channels() == 3) cv::cvtColor(im, bgra, cv::COLOR_BGR2BGRA);
            else                         cv::cvtColor(im, bgra, cv::COLOR_GRAY2BGRA);

            if (isCoverIcon(key)) {
                int side = std::min(bgra.cols, bgra.rows);
                cv::Mat sq = bgra(cv::Rect((bgra.cols - side) / 2, (bgra.rows - side) / 2, side, side));
                cv::Mat rs;
                cv::resize(sq, rs, cv::Size(D, D), 0, 0, cv::INTER_AREA);
                cv::Mat mask = cv::Mat::zeros(D, D, CV_8UC1);
                cv::circle(mask, cv::Point(D / 2, D / 2), D / 2 - 1, cv::Scalar(255), cv::FILLED, cv::LINE_AA);
                std::vector<cv::Mat> ch; cv::split(rs, ch);
                cv::multiply(ch[3], mask, ch[3], 1.0 / 255.0);
                cv::merge(ch, out);
            } else {
                double k = 0.74 * D / std::max(bgra.cols, bgra.rows);
                cv::Mat rs;
                cv::resize(bgra, rs, cv::Size(), k, k, cv::INTER_AREA);
                if (!rs.empty() && rs.cols <= D && rs.rows <= D)
                    rs.copyTo(out(cv::Rect((D - rs.cols) / 2, (D - rs.rows) / 2, rs.cols, rs.rows)));
            }
        }
        return discCache_[ck] = out;
    }

    void blitDisc(cv::Mat& frame, const std::string& key, cv::Point c, int R) {
        const cv::Mat& d = getDisc(key, 2 * R);
        cv::Rect dst(c.x - d.cols / 2, c.y - d.rows / 2, d.cols, d.rows);
        cv::Rect vis = dst & cv::Rect(0, 0, frame.cols, frame.rows);
        if (vis.area() <= 0) return;
        cv::Mat src = d(cv::Rect(vis.x - dst.x, vis.y - dst.y, vis.width, vis.height));
        cv::Mat bgr, alpha;
        cv::cvtColor(src, bgr, cv::COLOR_BGRA2BGR);
        cv::extractChannel(src, alpha, 3);
        cv::Mat m = alpha > 128;
        bgr.copyTo(frame(vis), m);                    // el aro dibujado encima tapa el borde
    }
    // false: tocar una tarjeta lateral la centra. true: la lanza de una.
    static constexpr bool kCarTapSideLaunches = false;

    static void crPath(std::vector<cv::Point>& pts, const cv::Rect& r, int rad) {
        rad = std::max(1, std::min(rad, std::min(r.width, r.height) / 2));
        auto arc = [&](int cx, int cy, int a0) {
            for (int a = a0; a <= a0 + 90; a += 10)
                pts.push_back({cx + (int)std::lround(rad * std::cos(a * CV_PI / 180.0)),
                               cy + (int)std::lround(rad * std::sin(a * CV_PI / 180.0))});
        };
        arc(r.x + r.width - rad - 1, r.y + rad,                 270);
        arc(r.x + r.width - rad - 1, r.y + r.height - rad - 1,  0);
        arc(r.x + rad,               r.y + r.height - rad - 1,  90);
        arc(r.x + rad,               r.y + rad,                 180);
    }

    static void crFill(cv::Mat& f, const cv::Rect& r, int rad, const cv::Scalar& c, double alpha = 1.0) {
        cv::Rect rr = r & cv::Rect(0, 0, f.cols, f.rows);
        if (rr.area() <= 0) return;
        std::vector<cv::Point> p; crPath(p, r, rad);
        if (alpha >= 0.99) { cv::fillPoly(f, p, c, cv::LINE_AA); return; }
        cv::Mat roi = f(rr), ov = roi.clone();
        for (auto& pt : p) { pt.x -= rr.x; pt.y -= rr.y; }
        cv::fillPoly(ov, p, c, cv::LINE_AA);
        cv::addWeighted(ov, alpha, roi, 1.0 - alpha, 0, roi);
    }

    static void crStroke(cv::Mat& f, const cv::Rect& r, int rad, const cv::Scalar& c, int thick = 1) {
        std::vector<cv::Point> p; crPath(p, r, rad);
        cv::polylines(f, p, true, c, thick, cv::LINE_AA);
    }

    static void crBlit(cv::Mat& frame, const cv::Mat& original, cv::Point center, int size) {
        if (original.empty() || size < 4) return;
        float asp = (float)original.cols / (float)original.rows;
        int iw, ih;
        if (asp >= 1.0f) { iw = size; ih = std::max(1, (int)(size / asp)); }
        else             { ih = size; iw = std::max(1, (int)(size * asp)); }
        cv::Mat icon;
        cv::resize(original, icon, {iw, ih}, 0, 0, cv::INTER_AREA);
        int x0 = center.x - iw / 2, y0 = center.y - ih / 2;
        cv::Rect dst(x0, y0, iw, ih);
        dst &= cv::Rect(0, 0, frame.cols, frame.rows);
        if (dst.area() <= 0) return;
        cv::Rect src(dst.x - x0, dst.y - y0, dst.width, dst.height);
        cv::Mat roi = frame(dst), crop = icon(src);
        if (icon.channels() == 4) {
            for (int ry = 0; ry < dst.height; ++ry)
                for (int rx = 0; rx < dst.width; ++rx) {
                    cv::Vec4b px = crop.at<cv::Vec4b>(ry, rx);
                    float a = px[3] / 255.f;
                    if (a > 0.f) {
                        cv::Vec3b& bg = roi.at<cv::Vec3b>(ry, rx);
                        for (int c = 0; c < 3; ++c)
                            bg[c] = cv::saturate_cast<uchar>(px[c] * a + bg[c] * (1.f - a));
                    }
                }
        } else if (icon.channels() == 3) {
            crop.copyTo(roi);
        }
    }

    void crPill(cv::Mat& f, const cv::Rect& r, const std::string& txt,
                const cv::Scalar& accent, bool hot, double maxScale = 0.5) {
        const cv::Scalar white(255, 255, 255);
        int rad = r.height / 2;
        cv::Scalar a = kidBright(accent);
        cv::Scalar fill = hot ? kidMix(a, white, 0.25) : a;
        crFill(f, cv::Rect(r.x, r.y + 4, r.width, r.height), rad, cv::Scalar(0, 0, 0), 0.22);
        crFill(f, r, rad, fill, 1.0);
        crStroke(f, r, rad, white, hot ? 3 : 2);
        double sc = maxScale; int bl = 0;
        while (sc > 0.3 && txu::size(txt, cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl).width > r.width - 24) sc -= 0.03;
        cv::Size ts = txu::size(txt, cv::FONT_HERSHEY_DUPLEX, sc, 1, &bl);
        txu::put(f, txt, cv::Point(r.x + (r.width - ts.width) / 2, r.y + (r.height + ts.height) / 2 - 1),
                cv::FONT_HERSHEY_DUPLEX, sc, white, 1, cv::LINE_AA);
    }

    // Tono medio (0..180 estilo OpenCV) del icono. -1 si es multicolor o gris.
    double iconHue(const std::string& key) {
        auto c = iconHueCache_.find(key);
        if (c != iconHueCache_.end()) return c->second;
        double res = -1.0;
        auto it = key.empty() ? iconMap.end() : iconMap.find(key);
        if (it != iconMap.end() && !it->second.empty()) {
            cv::Mat small, bgr, alpha;
            cv::resize(it->second, small, cv::Size(48, 48), 0, 0, cv::INTER_AREA);
            if (small.channels() == 4) {
                std::vector<cv::Mat> ch; cv::split(small, ch);
                alpha = ch[3];
                cv::merge(std::vector<cv::Mat>{ch[0], ch[1], ch[2]}, bgr);
            } else {
                if (small.channels() == 1) cv::cvtColor(small, bgr, cv::COLOR_GRAY2BGR);
                else bgr = small;
                alpha = cv::Mat(small.size(), CV_8UC1, cv::Scalar(255));
            }
            cv::Mat hsv; cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
            double sx = 0, sy = 0, wsum = 0; int opaque = 0;
            for (int y = 0; y < hsv.rows; ++y)
                for (int x = 0; x < hsv.cols; ++x) {
                    if (alpha.at<uchar>(y, x) < 128) continue;
                    ++opaque;
                    cv::Vec3b px = hsv.at<cv::Vec3b>(y, x);
                    if (px[1] < 70 || px[2] < 70) continue;
                    double w = px[1] / 255.0;
                    double ang = px[0] * 2.0 * CV_PI / 180.0;
                    sx += w * std::cos(ang); sy += w * std::sin(ang); wsum += w;
                }
            if (opaque > 0 && wsum / opaque > 0.12 && std::hypot(sx, sy) / wsum > 0.5) {
                double ang = std::atan2(sy, sx);
                if (ang < 0) ang += 2.0 * CV_PI;
                res = ang * 180.0 / (2.0 * CV_PI);
            }
        }
        iconHueCache_[key] = res;
        return res;
    }

    // Pinta el fondo de cada item con un color que contraste con su icono
    // y que no se repita con el vecino.
    void recolorForContrast(const std::vector<MenuItem*>& items) {
        struct PC { cv::Scalar c; double h; };
        // BGR apagados. h = tono estilo OpenCV (0..180) para medir contraste con el icono
        static const PC PAL[8] = {
            { cv::Scalar( 86, 140,  76),  60.0 },   // verde
            { cv::Scalar( 70, 184, 222),  25.0 },   // amarillo
            { cv::Scalar( 60, 130, 225),  12.0 },   // naranja
            { cv::Scalar( 70,  70, 200),   0.0 },   // rojo
            { cv::Scalar(190, 110,  65), 110.0 },   // azul
            { cv::Scalar(215, 180, 100),  95.0 },   // celeste
            { cv::Scalar( 65,  95, 140),  10.0 },   // cafe
            { cv::Scalar(170,  90, 130), 140.0 }    // morado
        };
        int use[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        int prev = -1, prev2 = -1;
        for (MenuItem* m : items) {
            double ih = iconHue(m->iconKey);
            int best = 0; double bestScore = -1e9;
            for (int k = 0; k < 8; ++k) {
                double sc = 0.0;
                if (ih >= 0) {
                    double d = std::fabs(ih - PAL[k].h);
                    d = std::min(d, 180.0 - d);
                    sc = std::min(d, 60.0);
                }
                sc -= 8.0 * use[k];
                if (k == prev)  sc -= 100.0;
                if (k == prev2) sc -= 30.0;
                if (sc > bestScore) { bestScore = sc; best = k; }
            }
            use[best]++; prev2 = prev; prev = best;
            m->color = PAL[best].c;
        }
    }

    void recolorAllMenus() {
        { std::vector<MenuItem*> v; for (auto& m : rootMenuItems) v.push_back(&m); recolorForContrast(v); }
        for (auto& kv : subMenuMap) {
            std::vector<MenuItem*> v; for (auto& m : kv.second.items) v.push_back(&m);
            recolorForContrast(v);
        }
    }

    void centerCarouselOnEmotions() {
        buildCarouselIfNeeded();
        for (int i = 0; i < (int)carousel_.size(); ++i) {
            if (carousel_[i].item.id == "yaren_emotions") {
                carSel_ = i;
                carPos_ = (float)i;
                break;
            }
        }
    }

    void buildCarouselIfNeeded() {
        if (!carousel_.empty() && carBuiltEnglish_ == isEnglish) return;
        carousel_.clear();
        carBuiltEnglish_ = isEnglish;
        // carFolderIds: categorias que se abren como carpeta (como MODO PRUEBA)
        const std::vector<std::string> carFolderIds =
            {"yaren_ai", "yaren_juegos", "yaren_dice", "yaren_movements", "yaren_filtros"};
        auto isFolder = [&](const std::string& id) {
            return std::find(carFolderIds.begin(), carFolderIds.end(), id) != carFolderIds.end();
        };
        const std::string catLabel = isEnglish ? "CATEGORY" : "CATEGORÍA";

        std::function<void(const std::string&)> walk = [&](const std::string& key) {
            auto it = subMenuMap.find(key);
            if (it == subMenuMap.end()) return;
            for (const auto& m : it->second.items) {
                if (isFolder(m.id) && m.hasSubMenu) {
                    carousel_.push_back({m, catLabel});
                } else if (m.hasSubMenu) {
                    walk(m.subMenuKey);
                } else {
                    carousel_.push_back({m, it->second.title});
                }
            }
        };
        walk("sub_yaren");
        walk("sub_yaren_p2");
        for (const auto& m : rootMenuItems)
            if (m.id == "modo_prueba")
                carousel_.push_back({m, isEnglish ? "TOOLS" : "HERRAMIENTAS"});
        { std::vector<MenuItem*> v; for (auto& e : carousel_) v.push_back(&e.item); recolorForContrast(v); }
        if (carSel_ >= (int)carousel_.size()) carSel_ = 0;
        carPos_ = (float)carSel_;
    }

    // ===== Helpers de color infantil =====
    static cv::Scalar kidBright(const cv::Scalar& c) {
        // Colores neutrales: ya no se satura ni se aumenta el brillo
        return c;
    }

    static cv::Scalar kidMix(const cv::Scalar& a, const cv::Scalar& b, double k) {
        return a * (1.0 - k) + b * k;
    }

    static void kidText(cv::Mat& f, const std::string& s, cv::Point org, int font, double sc,
                        const cv::Scalar& col, const cv::Scalar& outline, int th = 1) {
        txu::put(f, s, org, font, sc, outline, th + 3, cv::LINE_AA);
        txu::put(f, s, org, font, sc, col, th, cv::LINE_AA);
    }

    static void kidSparkle(cv::Mat& f, cv::Point c, int r, const cv::Scalar& col) {
        std::vector<cv::Point> p = {
            {c.x, c.y - r}, {c.x + r / 3, c.y - r / 3}, {c.x + r, c.y}, {c.x + r / 3, c.y + r / 3},
            {c.x, c.y + r}, {c.x - r / 3, c.y + r / 3}, {c.x - r, c.y}, {c.x - r / 3, c.y - r / 3}};
        cv::fillPoly(f, p, col, cv::LINE_AA);
    }

    void crBackground(cv::Mat& frame, const cv::Scalar& target, int cardBottom, double t) {
        (void)cardBottom;
        const int W = frame.cols, H = frame.rows;
        const cv::Scalar white(255, 255, 255);
        carGlow_ = carGlow_ + (target - carGlow_) * 0.08;
        if (!menuBg_.empty()) {
            if (menuBg_.size() == frame.size()) menuBg_.copyTo(frame);
            else cv::resize(menuBg_, frame, frame.size(), 0, 0, cv::INTER_LINEAR);
        } else {
            frame.setTo(cv::Scalar(231, 246, 248));
        }
        (void)W; (void)H; (void)white;
    }

    // Dibuja la cara de una tarjeta (sin perspectiva) en 'img'
    void crDrawFace(cv::Mat& img, const CarEntry& e, float focus, double t,
                    bool blocked, bool hot, cv::Rect& enterOut) {
        const int w = img.cols, h = img.rows, rad = 24;
        const cv::Scalar white(255, 255, 255);
        const cv::Scalar a     = blocked ? cv::Scalar(140, 140, 150) : kidBright(e.item.color);
        const cv::Scalar aDark = a * 0.6;
        const cv::Scalar aSoft = kidMix(a, white, 0.85);

        for (int y = 0; y < h; ++y) {
            double k = (double)y / std::max(1, h - 1);
            img.row(y).setTo(kidMix(a, white, 0.30) * (1.0 - k) + a * 0.85 * k);
        }
        {   // burbujas decorativas
            cv::Mat ov = img.clone();
            cv::Scalar light = kidMix(a, white, 0.7);
            cv::circle(ov, cv::Point((int)(w * 0.88), (int)(h * 0.16)), (int)(w * 0.28), light, cv::FILLED, cv::LINE_AA);
            cv::circle(ov, cv::Point((int)(w * 0.08), (int)(h * 0.80)), (int)(w * 0.22), light, cv::FILLED, cv::LINE_AA);
            cv::addWeighted(ov, 0.25, img, 0.75, 0, img);
        }
        auto fit = [&](const std::string& s, int font, double sc, double mn, int maxW) {
            int b = 0;
            while (sc > mn && txu::size(s, font, sc, 1, &b).width > maxW) sc -= 0.03;
            return sc;
        };

        // Etiqueta superior (grupo)
        cv::Rect ban(14, 14, w - 28, 26);
        crFill(img, ban, 13, white, 0.95);
        {
            double sc = fit(e.group, cv::FONT_HERSHEY_DUPLEX, 0.42, 0.28, ban.width - 14);
            drawTextInRect(img, e.group, ban, cv::FONT_HERSHEY_DUPLEX, sc, aDark, 1);
        }

        // Disco blanco con icono
        cv::Point c(w / 2, (int)(h * 0.41));
        int R = (int)(w * 0.29);
        double pulse = focus * 3.0 * std::sin(t * 3.0);
        cv::circle(img, c, R + 10 + (int)pulse, white, 2, cv::LINE_AA);
        cv::circle(img, cv::Point(c.x, c.y + 5), R, a * 0.55, cv::FILLED, cv::LINE_AA);
        cv::circle(img, c, R, white, cv::FILLED, cv::LINE_AA);
        cv::circle(img, c, R - 5, aSoft, cv::FILLED, cv::LINE_AA);
        if (focus > 0.3f) {
            for (int k = 0; k < 3; ++k) {
                double ang = t * 1.2 + k * 2.094;
                cv::Point p((int)(c.x + (R + 24) * std::cos(ang)), (int)(c.y + (R + 22) * std::sin(ang) * 0.9));
                kidSparkle(img, p, 5 + (k % 2) * 3, (k % 2) ? cv::Scalar(40, 220, 255) : white);
            }
        }
        if (blocked) {
            cv::Scalar lc(120, 120, 130);
            cv::rectangle(img, cv::Rect(c.x - 14, c.y - 3, 28, 22), lc, 3, cv::LINE_AA);
            cv::ellipse(img, cv::Point(c.x, c.y - 3), cv::Size(10, 12), 0, 180, 360, lc, 3, cv::LINE_AA);
        } else {
            auto iconIt = (!e.item.iconKey.empty()) ? iconMap.find(e.item.iconKey) : iconMap.end();
            if (iconIt != iconMap.end()) crBlit(img, iconIt->second, c, (int)(R * 1.55));
            else cv::circle(img, c, 8, a, cv::FILLED, cv::LINE_AA);
        }

        // Nombre y descripcion
        int bl = 0;
        int ly = c.y + R + 32;
        double ls = fit(e.item.label, cv::FONT_HERSHEY_DUPLEX, 0.62, 0.32, w - 18);
        cv::Size lsz = txu::size(e.item.label, cv::FONT_HERSHEY_DUPLEX, ls, 1, &bl);
        kidText(img, e.item.label, cv::Point((w - lsz.width) / 2, ly), cv::FONT_HERSHEY_DUPLEX, ls, white, aDark, 1);
        std::string sub = e.item.sublabel;
        std::vector<std::string> lines;
        if (txu::size(sub, cv::FONT_HERSHEY_PLAIN, 0.9, 1, &bl).width <= w - 18) {
            lines.push_back(sub);
        } else {
            size_t mid = sub.size() / 2;
            size_t sp = sub.find(' ', mid), sp2 = sub.rfind(' ', mid);
            size_t cut = (sp == std::string::npos) ? sp2
                       : (sp2 == std::string::npos ? sp : ((sp - mid) < (mid - sp2) ? sp : sp2));
            if (cut == std::string::npos) lines.push_back(sub);
            else { lines.push_back(sub.substr(0, cut)); lines.push_back(sub.substr(cut + 1)); }
        }
        for (size_t k = 0; k < lines.size() && k < 2; ++k) {
            double ss = fit(lines[k], cv::FONT_HERSHEY_PLAIN, 0.9, 0.5, w - 18);
            cv::Size ssz = txu::size(lines[k], cv::FONT_HERSHEY_PLAIN, ss, 1, &bl);
            kidText(img, lines[k], cv::Point((w - ssz.width) / 2, ly + 20 + (int)k * 15),
                    cv::FONT_HERSHEY_PLAIN, ss, white, aDark, 1);
        }

        // Boton ENTRAR (pildora blanca con play)
        cv::Rect bt(w / 2 - 60, h - 64, 120, 34);
        enterOut = bt;
        bool hotNow = hot && focus > 0.6f && !blocked;
        crFill(img, cv::Rect(bt.x, bt.y + 3, bt.width, bt.height), 17, aDark, 0.5);
        crFill(img, bt, 17, hotNow ? cv::Scalar(40, 220, 255) : white, 1.0);
        std::string et = blocked ? (isEnglish ? "NO INTERNET" : "SIN INTERNET")
                       : (e.item.hasSubMenu ? (isEnglish ? "OPEN" : "ABRIR")
                                            : (isEnglish ? "ENTER" : "ENTRAR"));
        double es = fit(et, cv::FONT_HERSHEY_DUPLEX, 0.5, 0.3, bt.width - 40);
        int off = blocked ? 0 : 18;
        drawTextInRect(img, et, cv::Rect(bt.x + off, bt.y, bt.width - off, bt.height),
                       cv::FONT_HERSHEY_DUPLEX, es, aDark, 1);
        if (!blocked) {
            std::vector<cv::Point> tri = { {bt.x + 16, bt.y + 11}, {bt.x + 16, bt.y + 23}, {bt.x + 26, bt.y + 17} };
            cv::fillPoly(img, tri, aDark, cv::LINE_AA);
        }

        // Borde blanco grueso
        crStroke(img, cv::Rect(1, 1, w - 3, h - 3), rad, white, focus > 0.5f ? 6 : 4);
        img.convertTo(img, -1, 0.80 + 0.20 * focus, 0);
    }

    // Dibuja una tarjeta con perspectiva (trapecio) en la posicion del carrusel
    // Disco de color con icono adentro (estilo circulos Pinkfong)
    void kidDisc(cv::Mat& frame, cv::Point c, int R, const cv::Scalar& a,
                 const std::string& iconKey, float glow, bool blocked) {
        const cv::Scalar white(255, 255, 255);
        // sombra suave
        cv::circle(frame, cv::Point(c.x, c.y + 6), R, cv::Scalar(205, 218, 222), cv::FILLED, cv::LINE_AA);
        // halo cuando esta enfocado
        // aro de color + aro blanco
        cv::circle(frame, c, R, a, cv::FILLED, cv::LINE_AA);

        const int Ri = (int)(R * 0.96);
        const bool hasIcon = !iconKey.empty() && iconMap.count(iconKey) && !iconMap[iconKey].empty();
        if (hasIcon) {
            if (!isCoverIcon(iconKey))                // icono con transparencia: fondo suave de color
                cv::circle(frame, c, Ri, kidMix(a, white, 0.80), cv::FILLED, cv::LINE_AA);
            blitDisc(frame, iconKey, c, Ri);
        } else {
            cv::circle(frame, c, Ri, kidMix(a, white, 0.60), cv::FILLED, cv::LINE_AA);
            cv::circle(frame, c, std::max(4, R / 6), white, cv::FILLED, cv::LINE_AA);
        }

        // brillo tipo caramelo arriba a la izquierda

        if (blocked) {                                // gris translucido + candado
            cv::Rect rr = cv::Rect(c.x - Ri, c.y - Ri, 2 * Ri, 2 * Ri) & cv::Rect(0, 0, frame.cols, frame.rows);
            if (rr.area() > 0) {
                cv::Mat roi = frame(rr), ov = roi.clone();
                cv::circle(ov, c - rr.tl(), Ri, cv::Scalar(150, 150, 158), cv::FILLED, cv::LINE_AA);
                cv::addWeighted(ov, 0.65, roi, 0.35, 0, roi);
            }
            int sz = std::max(6, R / 4);
            cv::rectangle(frame, cv::Rect(c.x - sz, c.y - sz / 2, 2 * sz, (int)(1.6 * sz)), white, 3, cv::LINE_AA);
            cv::ellipse(frame, cv::Point(c.x, c.y - sz / 2), cv::Size(sz * 2 / 3, sz), 0, 180, 360, white, 3, cv::LINE_AA);
        }
    }

    // Insignia blanca abajo a la derecha (play o carpeta)
    void kidBadge(cv::Mat& frame, cv::Point c, int R, const cv::Scalar& a, bool folder) {
        const cv::Scalar white(255, 255, 255);
        int br = std::max(12, (int)(R * 0.24));
        cv::Point b(c.x + (int)(R * 0.72), c.y + (int)(R * 0.72));
        cv::circle(frame, cv::Point(b.x, b.y + 2), br + 1, cv::Scalar(190, 205, 210), cv::FILLED, cv::LINE_AA);
        cv::circle(frame, b, br, white, cv::FILLED, cv::LINE_AA);
        cv::circle(frame, b, br - 3, kidMix(a, white, 0.88), cv::FILLED, cv::LINE_AA);
        cv::Scalar ic = a * 0.8;
        int s = std::max(3, br / 2);
        if (folder) {
            std::vector<cv::Point> ch = { {b.x - s / 2, b.y - s}, {b.x + s / 2, b.y}, {b.x - s / 2, b.y + s} };
            cv::polylines(frame, ch, false, ic, 3, cv::LINE_AA);
        } else {
            std::vector<cv::Point> tri = { {b.x - s / 2, b.y - s}, {b.x - s / 2, b.y + s}, {b.x + s, b.y} };
            cv::fillPoly(frame, tri, ic, cv::LINE_AA);
        }
    }

    void crDrawCard(cv::Mat& frame, int idx, float d, double t, int cy, int D, int stride) {
        const int W = frame.cols;
        const CarEntry& e = carousel_[idx];
        double ad = std::fabs((double)d);
        float focus = (float)std::max(0.0, 1.0 - ad);
        double cxd = W / 2.0 + d * stride;
        if (cxd < -D || cxd > W + D) return;
        bool blocked = (e.item.id == "yaren_chat") && !chatAvailable_;
        const cv::Scalar a = blocked ? cv::Scalar(150, 150, 158) : kidBright(e.item.color);
        const cv::Scalar ink(52, 48, 46);

        double bob = std::sin(t * 1.8 + idx * 1.1) * (2.0 + 4.0 * focus);
        int R = (int)(D * 0.5 * (0.78 + 0.38 * focus));
        
        cv::Point c((int)cxd, (int)(cy - 10.0 * focus + bob));

        // sombra en el suelo
        

        kidDisc(frame, c, R, a, e.item.iconKey, focus > 0.3f ? 0.6f : 0.f, blocked);   // halo fijo, sin pulso
        if (!blocked) kidBadge(frame, c, R, a, e.item.hasSubMenu);

        // etiquetas oscuras debajo
        auto fit = [&](const std::string& s, int font, double sc, double mn, int maxW) {
            int b = 0;
            while (sc > mn && txu::size(s, font, sc, 1, &b).width > maxW) sc -= 0.03;
            return sc;
        };
        int maxW = stride - 16, bl = 0;
        int ly = cy + (int)(D * 0.58) + 36;
        double ls = fit(e.item.label, cv::FONT_HERSHEY_DUPLEX, 0.52 + 0.14 * focus, 0.34, maxW);
        cv::Size lsz = txu::size(e.item.label, cv::FONT_HERSHEY_DUPLEX, ls, 1, &bl);
        txu::put(frame, e.item.label, cv::Point((int)cxd - lsz.width / 2, ly), cv::FONT_HERSHEY_DUPLEX, ls,
                    focus > 0.5f ? ink : ink * 1.5, 1, cv::LINE_AA);


        // zona tocable
        cv::Rect hit = cv::Rect(c.x - R - 10, c.y - R - 10, 2 * R + 20, 2 * R + 20) & cv::Rect(0, 0, frame.cols, frame.rows);
        if (hit.area() > 0) carHit_.push_back({idx, hit});
        if (ad < 0.4 && hit.area() > 0) carEnterRect_ = hit;
    }

    void renderCarousel(cv::Mat& frame) {
        if (navStack.empty()) return;
        buildCarouselIfNeeded();
        const int N = (int)carousel_.size();
        if (N == 0) return;
        const int W = frame.cols, H = frame.rows;
        const cv::Scalar white(255, 255, 255), ink(52, 48, 46);
        const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        leftNavArrowRect = {0, 0, 0, 0}; rightNavArrowRect = {0, 0, 0, 0};

        const int D = 200;
        const int cy = (int)(H * 0.48);
        const int stride = (int)(D * 1.20);
        carStride_ = std::max(1, stride);

        if (!carDragging_) {
            carPos_ += ((float)carSel_ - carPos_) * 0.22f;
            if (std::fabs((float)carSel_ - carPos_) < 0.003f) carPos_ = (float)carSel_;
        }
        carPos_ = std::max(0.f, std::min((float)(N - 1), carPos_));
        int nearIdx = std::max(0, std::min(N - 1, (int)std::lround(carPos_)));
        const CarEntry& cur = carousel_[nearIdx];
        const cv::Scalar curA = kidBright(cur.item.color);

        crBackground(frame, cur.item.color, cy + D, t);

        // Titulo de seccion
        kd::centered(frame, isEnglish ? "SELECT A MODE" : "SELECCIONA UN MODO", W, 50, cv::FONT_HERSHEY_DUPLEX, 0.95, cv::Scalar(52, 48, 46), 1);

        // Circulos (de lejos a cerca)
        carEnterRect_ = cv::Rect(0, 0, 0, 0);
        carHit_.clear();
        std::vector<int> order;
        for (int i = 0; i < N; ++i)
            if (std::fabs((float)i - carPos_) < 3.2f) order.push_back(i);
        std::sort(order.begin(), order.end(), [&](int a, int b) {
            return std::fabs((float)a - carPos_) > std::fabs((float)b - carPos_);
        });
        for (int i : order) crDrawCard(frame, i, (float)i - carPos_, t, cy, D, stride);

        // Flechas grandes laterales + contador
        {
            auto bigArrow = [&](cv::Point c, bool left, bool enabled) {
                const int Rr = 28;
                cv::circle(frame, cv::Point(c.x, c.y + 3), Rr, cv::Scalar(205, 218, 222), cv::FILLED, cv::LINE_AA);
                cv::circle(frame, c, Rr, white, cv::FILLED, cv::LINE_AA);
                cv::circle(frame, c, Rr, enabled ? curA : cv::Scalar(215, 215, 215), 4, cv::LINE_AA);
                int sg = left ? -1 : 1;
                std::vector<cv::Point> ch = { {c.x - sg * 5, c.y - 12}, {c.x + sg * 7, c.y}, {c.x - sg * 5, c.y + 12} };
                cv::polylines(frame, ch, false, enabled ? curA * 0.8 : cv::Scalar(200, 200, 200), 5, cv::LINE_AA);
            };
            cv::Point pc(40, cy), nc(W - 40, cy);
            bigArrow(pc, true,  nearIdx > 0);
            bigArrow(nc, false, nearIdx < N - 1);
            carPrevBtn_ = (nearIdx > 0)     ? cv::Rect(pc.x - 36, pc.y - 36, 72, 72) : cv::Rect(0, 0, 0, 0);
            carNextBtn_ = (nearIdx < N - 1) ? cv::Rect(nc.x - 36, nc.y - 36, 72, 72) : cv::Rect(0, 0, 0, 0);

            (void)ink;
        }

        // SALIR / DETENER
        bool hasStop = (activeMode == "yaren_chat" || activeMode == "yaren_chat_local" ||
                        activeMode == "yaren_dice_sin_ayuda" ||
                        activeMode == "yaren_dice_sesion");
        exitButtonRect = cv::Rect(24, H - 62, 120, 40);
        crPill(frame, exitButtonRect, isEnglish ? "EXIT" : "SALIR", cv::Scalar(255, 170, 60), hoveredExit.load());
        if (hasStop) {
            stopButtonRect = cv::Rect(W - 24 - 150, H - 62, 150, 40);
            crPill(frame, stopButtonRect, isEnglish ? "STOP" : "DETENER", cv::Scalar(70, 70, 235), hoveredStop.load());
        } else { stopButtonRect = cv::Rect(0, 0, 0, 0); }
        backButtonRect = cv::Rect(0, 0, 0, 0);
    }

    void carActivate(int idx) {
        if (idx < 0 || idx >= (int)carousel_.size()) return;
        MenuItem item = carousel_[idx].item;
        if (item.id == "yaren_chat" && !chatAvailable_) {
            wifiSetup_.refresh();
            std::lock_guard<std::mutex> lk(modeFlagMutex);
            showWifiSetup_ = true;
            return;
        }
        if (item.hasSubMenu) {
            auto it = subMenuMap.find(item.subMenuKey);
            if (it != subMenuMap.end()) {
                NavLevel lvl = it->second;
                lvl.key = item.subMenuKey;
                navStack.push_back(lvl);
                hoveredItem = -1;
            }
        } else {
            executeMode(item);
        }
    }

    int carHitTest(const cv::Point& p) const {
        for (int k = (int)carHit_.size() - 1; k >= 0; --k)
            if (carHit_[k].second.contains(p)) return carHit_[k].first;
        return -1;
    }

    // Devuelve true si el evento fue consumido por el carrusel
    bool carouselMouse(int ev, int x, int y) {
        const int N = (int)carousel_.size();
        if (N == 0) return false;
        cv::Point pt(x, y);
        if (ev == cv::EVENT_MOUSEMOVE) {
            if (carDown_) {
                int dx = x - carDownX_;
                if (std::abs(dx) > 10) carDragging_ = true;
                if (carDragging_) {
                    float p = carDownPos_ - (float)dx / (float)std::max(1, carStride_);
                    carPos_ = std::max(0.f, std::min((float)(N - 1), p));
                }
                return true;
            }
            carHover_ = carHitTest(pt);
            carEnterHot_ = carEnterRect_.area() > 0 && carEnterRect_.contains(pt);
            return false;
        }
        if (ev == cv::EVENT_LBUTTONDOWN) {
            if (carPrevBtn_.area() > 0 && carPrevBtn_.contains(pt)) { carSel_ = std::max(0, carSel_ - 1); return true; }
            if (carNextBtn_.area() > 0 && carNextBtn_.contains(pt)) { carSel_ = std::min(N - 1, carSel_ + 1); return true; }
            int h = carHitTest(pt);
            if (h >= 0) {
                carDown_ = true; carDragging_ = false;
                carDownX_ = x; carDownPos_ = carPos_; carDownIdx_ = h;
                return true;
            }
            return false;
        }
        if (ev == cv::EVENT_LBUTTONUP) {
            if (!carDown_) return false;
            carDown_ = false;
            if (carDragging_) {
                carDragging_ = false;
                carSel_ = std::max(0, std::min(N - 1, (int)std::lround(carPos_)));
                return true;
            }
            int idx = carDownIdx_;
            if (idx == carSel_ || kCarTapSideLaunches) { carSel_ = idx; carActivate(idx); }
            else carSel_ = idx;
            return true;
        }
        return false;
    }

    void renderMenu(cv::Mat& frame) {
        if (navStack.size() == 1) renderCarousel(frame);   // raiz = carrusel
        else                      renderMenuGrid(frame);   // submenus (MODO PRUEBA) como antes
    }

    void renderMenuGrid(cv::Mat& frame) {
        if (navStack.empty()) return;
        auto& level = navStack.back();
        const int W = frame.cols, H = frame.rows, N = (int)level.items.size();
        const bool isRoot = (navStack.size() == 1);
        const double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        leftNavArrowRect = {0, 0, 0, 0}; rightNavArrowRect = {0, 0, 0, 0};

        // --- Fondo con resplandor que sigue a la tarjeta enfocada ---
        int hi = hoveredItem.load();
        cv::Scalar gTarget = level.accentColor;
        cv::Point2f gPos(W * 0.5f, H * 0.5f);
        if (hi >= 0 && hi < N && level.items[hi].rect.area() > 0) {
            const cv::Rect& hr = level.items[hi].rect;
            gTarget = level.items[hi].color;
            gPos = cv::Point2f(hr.x + hr.width * 0.5f, hr.y + hr.height * 0.5f);
        }
        drawMenuBackground(frame, gTarget, gPos, t);

        // --- Titulo en pastilla ---
        std::string title;
        if (isRoot) title = isEnglish ? "SELECT A MODE" : "SELECCIONA UN MODO";
        else if (navStack.size() == 2) title = navStack[1].title;
        else if (navStack.size() >= 3) title = navStack[navStack.size() - 2].title + "  >  " + navStack.back().title;
        else title = navStack.back().title;
        cv::Scalar titleColor = isRoot ? cv::Scalar(230, 190, 90) : level.accentColor;
        {
            int bl = 0;
            cv::Size tsz = txu::size(title, cv::FONT_HERSHEY_DUPLEX, 0.6, 1, &bl);
            int pw = std::min(W - 120, tsz.width + 70);
            drawPill(frame, cv::Rect((W - pw) / 2, 10, pw, 34), title, titleColor, false, 0.6);
        }

        // --- Layout (mismo calculo que antes + pequeno arco en las laterales) ---
        int COLS = (N <= 2) ? 2 : 3, ROWS = (N + COLS - 1) / COLS;
        int CW = std::min((N <= 2) ? 340 : 220, W / COLS - 20);
        int CH = std::min((N <= 2) ? 260 : 230, (H - 130 - (ROWS - 1) * 10) / std::max(ROWS, 1));
        int G = 10, TH = ROWS * CH + (ROWS - 1) * G, SY = 54 + ((H - 130) - TH) / 2;
        for (int i = 0; i < N; ++i) {
            int row = i / COLS, col = i % COLS, rowItems = std::min(COLS, N - row * COLS);
            int rowW = rowItems * CW + (rowItems - 1) * G, rowSX = (W - rowW) / 2;
            int arc = (rowItems >= 3 && (col == 0 || col == 2)) ? 14 : 0;
            level.items[i].rect = { rowSX + col * (CW + G), SY + row * (CH + G) + arc, CW, CH };
        }

        // --- Animacion de hover + dibujo (la tarjeta enfocada va al final, encima) ---
        for (int i = 0; i < N && i < 16; ++i) {
            bool blocked = (level.items[i].id == "yaren_chat") && !chatAvailable_;
            float target = (i == hi && !blocked) ? 1.f : 0.f;
            menuHoverAnim_[i] += (target - menuHoverAnim_[i]) * 0.25f;
        }
        for (int i = 0; i < N; ++i)
            if (i != hi) drawCard(frame, level.items[i], level.items[i].rect,
                                  menuHoverAnim_[std::min(i, 15)], level.items[i].hasSubMenu, t, i);
        if (hi >= 0 && hi < N)
            drawCard(frame, level.items[hi], level.items[hi].rect,
                     menuHoverAnim_[std::min(hi, 15)], level.items[hi].hasSubMenu, t, hi);

        // --- Botones inferiores (pildoras) ---
        const int btnH = 40, btnY = H - 58, gap = 16, stopW = 300, navW = 150;
        bool hasStop = (activeMode == "yaren_chat" ||
                        activeMode == "yaren_chat_local" ||
                        activeMode == "yaren_dice_sin_ayuda" ||
                        activeMode == "yaren_dice_sesion");
        bool hasBack = (navStack.size() > 1);
        int totalW = navW;
        if (hasBack) totalW += navW + gap;
        if (hasStop) totalW += stopW + gap;
        int cx = (W - totalW) / 2;
        if (hasStop) {
            stopButtonRect = { cx, btnY, stopW, btnH };
            drawPill(frame, stopButtonRect, (isEnglish ? "STOP: " : "DETENER: ") + activeMode,
                     cv::Scalar(70, 70, 235), hoveredStop.load());
            cx += stopW + gap;
        } else { stopButtonRect = {0, 0, 0, 0}; }
        if (hasBack) {
            backButtonRect = { cx, btnY, navW, btnH };
            drawPill(frame, backButtonRect, isEnglish ? "BACK" : "VOLVER",
                     cv::Scalar(200, 180, 150), hoveredBack.load());
            cx += navW + gap;
        } else { backButtonRect = {0, 0, 0, 0}; }

        // --- Flechas laterales redondas ---
        const std::string& curKey = navStack.back().key;
        if (curKey == "sub_yaren") {
            rightNavArrowRect = { W - 58, H / 2 - 70, 48, 130 };
            renderRoundNavArrow(frame, rightNavArrowRect, false, "1/2", hoveredRightNav);
        }
        if (curKey == "sub_yaren_p2") {
            leftNavArrowRect = { 10, H / 2 - 70, 48, 130 };
            renderRoundNavArrow(frame, leftNavArrowRect, true, "2/2", hoveredLeftNav);
        }

        exitButtonRect = { cx, btnY, navW, btnH };
        drawPill(frame, exitButtonRect, isEnglish ? "EXIT" : "SALIR",
                 cv::Scalar(150, 150, 160), hoveredExit.load());
    }

    void renderFaceOverlay(cv::Mat& frame) {
        if (frame.empty() || frame.cols < 100 || frame.rows < 100) return;
        FaceOverlay ov; std::string msg; int countDown;
        {
            std::lock_guard<std::mutex> lock(overlayMutex);
            ov = faceOverlay; msg = overlayMessage; countDown = micCountdownSecs;
        }
        if (ov == FaceOverlay::NONE) return;
        int W = frame.cols, H = frame.rows;
        const cv::Scalar white = kd::white();
        cv::Rect panelRect(W / 8, H / 3, 6 * W / 8, H / 3);
        bool isErr = (ov == FaceOverlay::ERROR_MSG);
        cv::Scalar a = isErr ? kd::pal(0) : (ov == FaceOverlay::MIC_PLAYING ? kd::pal(3) : kd::pal(4));
        kd::card(frame, panelRect, 28, a, 4);

        cv::circle(frame, {W / 2, panelRect.y + 28}, 12, a, cv::FILLED, cv::LINE_AA);
        cv::circle(frame, {W / 2, panelRect.y + 28}, 4, white, cv::FILLED, cv::LINE_AA);

        std::vector<std::string> lines; std::istringstream ss(msg); std::string line;
        while (std::getline(ss, line, '\n')) lines.push_back(line);
        double scale = 0.7; int lineH = 30, totalTH = (int)lines.size() * lineH;
        int startY = panelRect.y + (panelRect.height - totalTH) / 2 + lineH / 2 + 4;
        cv::Scalar textColor = isErr ? cv::Scalar(60, 60, 200) : kd::ink();
        for (int i = 0; i < (int)lines.size(); ++i) {
            int bl = 0; cv::Size ts = txu::size(lines[i], cv::FONT_HERSHEY_DUPLEX, scale, 1, &bl);
            txu::put(frame, lines[i], { (W - ts.width) / 2, startY + i * lineH },
                        cv::FONT_HERSHEY_DUPLEX, scale, textColor, 1, cv::LINE_AA);
        }
        if (ov == FaceOverlay::MIC_COUNTDOWN) {
            int barW = panelRect.width - 60, barX = panelRect.x + 30, barY = panelRect.y + panelRect.height - 24, barH = 12;
            float pct = (float)countDown / 5.0f;
            kd::fillRound(frame, cv::Rect(barX, barY, barW, barH), 6, cv::Scalar(225, 232, 236));
            int fw = (int)(barW * pct);
            if (fw > 8) kd::fillRound(frame, cv::Rect(barX, barY, fw, barH), 6, kd::rainbow(1.0f - pct));
        }
        if (ov == FaceOverlay::MIC_PLAYING) {
            double t = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            int cx = W / 2, cyAnim = panelRect.y + panelRect.height - 22;
            for (int w = 0; w < 5; ++w) {
                double amp = 7.0 * std::abs(std::sin(t * 4.0 + w * 0.8));
                int x = cx - 40 + w * 20;
                cv::line(frame, {x, cyAnim - (int)amp}, {x, cyAnim + (int)amp}, kd::pal(w), 5, cv::LINE_AA);
            }
        }
    }

    void drawCard(cv::Mat& frame, const MenuItem& item, const cv::Rect& base,
                  float hv, bool arrow, double t, int idx) {
        bool blocked = (item.id == "yaren_chat") && !chatAvailable_;
        if (blocked) hv = 0.f;
        const cv::Scalar ink(52, 48, 46);
        int bob = (int)std::lround(std::sin(t * 1.6 + idx * 1.1) * 3.0 * (1.0 - hv));
        cv::Scalar a = blocked ? cv::Scalar(150, 150, 158) : kidBright(item.color);
        int Rb = std::max(24, std::min(base.width / 2 - 12, (base.height - 46) / 2));
        int R = Rb + (int)(hv * 8);
        cv::Point c(base.x + base.width / 2, base.y + 6 + Rb + bob - (int)(hv * 4));

        cv::ellipse(frame, cv::Point(c.x, base.y + 6 + 2 * Rb + 8), cv::Size((int)(Rb * 0.8), 6),
                    0, 0, 360, cv::Scalar(205, 218, 222), cv::FILLED, cv::LINE_AA);
        kidDisc(frame, c, R, a, item.iconKey, hv, blocked);
        if (!blocked) kidBadge(frame, c, R, a, arrow);

        auto fitScale = [&](const std::string& s, int font, double sc, double minSc, int maxW) {
            int b = 0;
            while (sc > minSc && txu::size(s, font, sc, 1, &b).width > maxW) sc -= 0.03;
            return sc;
        };
        int bl = 0;
        double ls = fitScale(item.label, cv::FONT_HERSHEY_DUPLEX, 0.65, 0.32, base.width - 12);
        cv::Size lsz = txu::size(item.label, cv::FONT_HERSHEY_DUPLEX, ls, 1, &bl);
        txu::put(frame, item.label, cv::Point(base.x + (base.width - lsz.width) / 2, base.y + base.height - 12),
                    cv::FONT_HERSHEY_DUPLEX, ls, ink, 1, cv::LINE_AA);
    }

    cv::Mat getFaceFrame() {
        const int CW = 800, CH = 480;
        auto now = std::chrono::system_clock::now();
        cv::Mat res = cv::Mat::zeros(CH, CW, CV_8UC3);

        // Pestaneo: cada ~3.5 s, ojos cerrados 0.15 s
        double sinceBlink = std::chrono::duration<double>(now - lastBlinkTime).count();
        if (!isBlinking && sinceBlink > 3.5) { isBlinking = true; blinkStartTime = now; lastBlinkTime = now; }
        if (isBlinking && std::chrono::duration<double>(now - blinkStartTime).count() > 0.3) isBlinking = false;

        const cv::Mat* src = &faceOpenImg_;
        if (ttsActive.load() && !faceTalkImg_.empty())            src = &faceTalkImg_;
        else if (isBlinking.load() && !faceClosedImg_.empty())    src = &faceClosedImg_;

        if (src->empty()) {
            txu::put(res, isEnglish ? "WARNING: face images not found" : "ADVERTENCIA: imágenes de cara no encontradas",
                     { 20, CH / 2 }, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 100, 255), 1, cv::LINE_AA);
            return res;
        }
        double k = std::min(CW / (double)src->cols, CH / (double)src->rows);
        int w = std::max(1, (int)(src->cols * k)), h = std::max(1, (int)(src->rows * k));
        cv::Mat rs;
        cv::resize(*src, rs, cv::Size(w, h), 0, 0, cv::INTER_AREA);
        cv::Mat roi = res(cv::Rect((CW - w) / 2, (CH - h) / 2, w, h));
        if (rs.channels() == 4) {
            for (int y = 0; y < h; ++y)
                for (int x = 0; x < w; ++x) {
                    const cv::Vec4b& f = rs.at<cv::Vec4b>(y, x);
                    float a = f[3] / 255.f;
                    if (a > 0.f) {
                        cv::Vec3b& b = roi.at<cv::Vec3b>(y, x);
                        for (int c = 0; c < 3; ++c) b[c] = cv::saturate_cast<uchar>(f[c] * a + b[c] * (1.f - a));
                    }
                }
        } else if (rs.channels() == 3) {
            rs.copyTo(roi);
        } else {
            cv::cvtColor(rs, roi, cv::COLOR_GRAY2BGR);
        }
        return res;
    }

    // =============================================================================
    //  renderLoadingScreen — "ROBOT AWAKENING" v2
    //  La imagen de Yaren se revela de abajo → arriba sincronizada con la barra
    //  de progreso: 0% = imagen oculta, 100% = imagen completamente visible.
    //  Reemplaza el bloque completo de renderLoadingScreen() en face_screen.cpp
    // =============================================================================
    // ======================= LOADING: robot animado =======================
    struct LXf {
        double ox{0}, oy{0}, sx{1}, sy{1}, cs{1}, sn{0};
        double cx0{0}, cy0{0};   // pivote local (para girar solo la cabeza)
        cv::Point2d P(double x, double y) const {
            x -= cx0; y -= cy0;
            double rx = x * cs - y * sn, ry = x * sn + y * cs;
            return cv::Point2d(ox + rx * sx, oy + ry * sy);
        }
        double sc() const { return 0.5 * (sx + sy); }
    };

    struct LArm { cv::Point2d S, E, H; };

    static cv::Point lp(const LXf& T, double x, double y) {
        cv::Point2d p = T.P(x, y);
        return cv::Point((int)std::lround(p.x), (int)std::lround(p.y));
    }
    static void ldRR(cv::Mat& f, const LXf& T, const cv::Rect& r, int rad, const cv::Scalar& c) {
        std::vector<cv::Point> loc, pts;
        kd::roundPath(loc, r, rad);
        pts.reserve(loc.size());
        for (const auto& q : loc) pts.push_back(lp(T, q.x, q.y));
        cv::fillPoly(f, pts, c, cv::LINE_AA);
    }
    static void ldEll(cv::Mat& f, const LXf& T, double x, double y, double rx, double ry,
                      const cv::Scalar& c, double thick = -1) {
        std::vector<cv::Point> pts;
        for (int a = 0; a < 360; a += 10) {
            double r = a * CV_PI / 180.0;
            pts.push_back(lp(T, x + rx * std::cos(r), y + ry * std::sin(r)));
        }
        if (thick < 0) cv::fillPoly(f, pts, c, cv::LINE_AA);
        else cv::polylines(f, pts, true, c, std::max(1, (int)std::lround(thick * T.sc())), cv::LINE_AA);
    }
    static void ldArc(cv::Mat& f, const LXf& T, double x, double y, double rx, double ry,
                      int a0, int a1, const cv::Scalar& c, double thick) {
        std::vector<cv::Point> pts;
        for (int a = a0; a <= a1; a += 8) {
            double r = a * CV_PI / 180.0;
            pts.push_back(lp(T, x + rx * std::cos(r), y + ry * std::sin(r)));
        }
        if (pts.size() > 1)
            cv::polylines(f, pts, false, c, std::max(1, (int)std::lround(thick * T.sc())), cv::LINE_AA);
    }
    static void ldLine(cv::Mat& f, const LXf& T, double x1, double y1, double x2, double y2,
                       const cv::Scalar& c, double th) {
        cv::line(f, lp(T, x1, y1), lp(T, x2, y2), c, std::max(1, (int)std::lround(th * T.sc())), cv::LINE_AA);
    }
    static void ldPoly(cv::Mat& f, const LXf& T, const std::vector<cv::Point2d>& loc, const cv::Scalar& c) {
        std::vector<cv::Point> pts;
        for (const auto& q : loc) pts.push_back(lp(T, q.x, q.y));
        cv::fillPoly(f, pts, c, cv::LINE_AA);
    }

    static void ldEllRot(cv::Mat& f, const LXf& T, cv::Point2d c, cv::Point2d u,
                         double a, double b, const cv::Scalar& col) {
        cv::Point2d n(-u.y, u.x);
        std::vector<cv::Point> pts;
        for (int d = 0; d < 360; d += 12) {
            double r = d * CV_PI / 180.0;
            cv::Point2d p = c + u * (a * std::cos(r)) + n * (b * std::sin(r));
            pts.push_back(lp(T, p.x, p.y));
        }
        cv::fillPoly(f, pts, col, cv::LINE_AA);
    }

    static void ldRRStroke(cv::Mat& f, const LXf& T, const cv::Rect& r, int rad,
                           const cv::Scalar& c, double th) {
        std::vector<cv::Point> loc, pts;
        kd::roundPath(loc, r, rad);
        for (const auto& q : loc) pts.push_back(lp(T, q.x, q.y));
        cv::polylines(f, pts, true, c, std::max(1, (int)std::lround(th * T.sc())), cv::LINE_AA);
    }

    static LArm ldSolveArm(int side, cv::Point2d target) {
        LArm a;
        a.S = cv::Point2d(side * 52.0, -96.0);
        const double L1 = 34.0, L2 = 32.0;
        cv::Point2d d = target - a.S;
        double len = std::sqrt(d.x * d.x + d.y * d.y);
        if (len < 1e-3) { d = cv::Point2d(0, 1); len = 1.0; }
        double use = std::max(14.0, std::min(L1 + L2 - 1.0, len));
        cv::Point2d u(d.x / len, d.y / len);
        a.H = a.S + u * use;
        double aa = (L1 * L1 - L2 * L2 + use * use) / (2.0 * use);
        double h = std::sqrt(std::max(0.0, L1 * L1 - aa * aa));
        cv::Point2d base = a.S + u * aa;
        cv::Point2d perp(-u.y, u.x);
        cv::Point2d e1 = base + perp * h, e2 = base - perp * h;
        a.E = (e1.x * side > e2.x * side) ? e1 : e2;
        return a;
    }

    // Brazo negro segmentado (sin circulos en hombro / codo / muneca) + manita
    static void ldDrawArm(cv::Mat& f, const LXf& T, const LArm& a) {
        const cv::Scalar blk(44, 40, 40), blkDk(22, 19, 19), hl(108, 102, 102), hl2(84, 78, 78);
        const double side = (a.S.x < 0) ? -1.0 : 1.0;
        auto unit = [](cv::Point2d v) {
            double l = std::sqrt(v.x * v.x + v.y * v.y);
            return (l < 1e-6) ? cv::Point2d(0, 1) : cv::Point2d(v.x / l, v.y / l);
        };
        const cv::Point2d u1 = unit(a.E - a.S), u2 = unit(a.H - a.E);

        // dos segmentos redondeados (el antebrazo un poco mas delgado)
        ldLine(f, T, a.S.x, a.S.y, a.E.x, a.E.y, blk, 22);
        ldLine(f, T, a.E.x, a.E.y, a.H.x, a.H.y, blk, 19);

        // brillo (luz arriba-izquierda) en cada segmento
        auto shine = [&](cv::Point2d p0, cv::Point2d p1, cv::Point2d u, double off) {
            cv::Point2d n(-u.y, u.x);
            if (n.x + n.y > 0) n = -n;
            cv::Point2d q0 = p0 + u * 6.0 + n * off, q1 = p1 - u * 6.0 + n * off;
            ldLine(f, T, q0.x, q0.y, q1.x, q1.y, hl, 2.6);
        };
        shine(a.S, a.E, u1, 5.0);
        shine(a.E, a.H, u2, 4.2);

        // costura del codo (linea, no punto)
        cv::Point2d n2(-u2.y, u2.x);
        ldLine(f, T, a.E.x - n2.x * 8.5, a.E.y - n2.y * 8.5,
                     a.E.x + n2.x * 8.5, a.E.y + n2.y * 8.5, blkDk, 1.6);

        // mano: palma ovalada + pulgar + dedos
        cv::Point2d pc = a.H + u2 * 9.5;
        ldEllRot(f, T, pc, u2, 12.5, 10.5, blk);
        cv::Point2d nn = n2;
        if (nn.x * (-side) < 0) nn = -nn;                       // lado interior (hacia el cuerpo)
        ldEllRot(f, T, pc + nn * 9.5 + u2 * 1.5, u2, 6.5, 4.4, blk);
        for (int k = -1; k <= 1; ++k) {
            cv::Point2d b0 = pc + u2 * 3.0 + n2 * (k * 4.2);
            cv::Point2d b1 = pc + u2 * 11.5 + n2 * (k * 3.4);
            ldLine(f, T, b0.x, b0.y, b1.x, b1.y, hl2, 1.3);
        }
        cv::Point2d sh = n2; if (sh.x + sh.y > 0) sh = -sh;
        cv::Point2d hp = pc + sh * 5.0 - u2 * 2.0;
        ldEllRot(f, T, hp, u2, 5.5, 2.6, hl2);
    }

    // Ojos estilo YAREN: esclera clara, iris gris azulado, pupila y brillo
    static void ldEyes(cv::Mat& f, const LXf& T, int emo, double lx, double ly, double blink, double es) {
        const cv::Scalar scl(247, 240, 236), sclEdge(214, 196, 190), iris(156, 132, 124),
                         irisDk(122, 100, 96), pupil(54, 44, 42), white(255, 255, 255),
                         lineC(250, 246, 244);
        const double ex = 35.0, ey = -172.0;
        const double bl = std::max(0.12, 1.0 - blink);

        auto eye = [&](double cx0, double rx, double ry) {
            const double ry2 = ry * bl;
            ldEll(f, T, cx0, ey, rx + 2.0, ry2 + 2.0, sclEdge);
            ldEll(f, T, cx0, ey, rx, ry2, scl);
            const double px = cx0 + lx * 6.0, py = ey + ly * 5.0 * bl;
            const double ir = rx * 0.68;
            ldEll(f, T, px, py, ir, ir * bl, irisDk);
            ldEll(f, T, px, py, ir * 0.88, ir * 0.88 * bl, iris);
            ldEll(f, T, px, py, ir * 0.50, ir * 0.50 * bl, pupil);
            ldEll(f, T, px + ir * 0.30, py - ir * 0.34 * bl, ir * 0.24, ir * 0.24 * bl, white);
            ldEll(f, T, px - ir * 0.26, py + ir * 0.30 * bl, ir * 0.10, ir * 0.10 * bl, white);
        };
        auto arcUp = [&](double cx0) { ldArc(f, T, cx0, ey + 8, 17, 12, 200, 340, lineC, 7); };   // ^
        auto arcDn = [&](double cx0) { ldArc(f, T, cx0, ey - 4, 17, 11, 20, 160, lineC, 7); };    // u
        auto smile = [&](double w, double h) { ldArc(f, T, 0, -154, w, h, 15, 165, lineC, 3.4); };

        switch (emo) {
            case 0:  arcDn(-ex); arcDn(ex); smile(9, 5); break;
            case 2:  arcUp(-ex); arcUp(ex); smile(13, 8); break;
            case 3:
                eye(-ex, 22 * es, 23 * es);
                eye(ex, 22 * es, 23 * es);
                ldEll(f, T, 0, -148, 4.5, 6.0, lineC, 2.8);
                break;
            case 4:  eye(-ex, 21 * es, 22 * es); arcDn(ex); smile(9, 5); break;
            default:
                eye(-ex, 21 * es, 22 * es);
                eye(ex, 21 * es, 22 * es);
                smile(9, 5);
                break;
        }
    }

    bool renderIntroVideo(cv::Mat& frame) {
        if (!introVideo.isOpened()) return false;
        const int W = frame.cols, H = frame.rows;
        const double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - introStart).count();
        // Unos 3 s antes del final, abrir el siguiente video en un hilo aparte
        if (introDur_ > 0.0 && el > introDur_ - 3.0 &&
            !introNextReady_.load() && !introNextLoading_.exchange(true)) {
            std::thread([this]() {
                introNext_.open(introPath_);
                introNextReady_ = introNext_.isOpened();
                introNextLoading_ = false;
            }).detach();
        }

        const long target = (long)(el * introFps);
        bool advanced = false;
        int guard = 0;
        while (introIdx < target && guard++ < 8) {
            if (!introVideo.grab()) {
                // Fin: cambiar al video ya abierto (sin seek, sin espera)
                if (introNextReady_.load()) {
                    introVideo.release();
                    introVideo = std::move(introNext_);
                    introNext_ = cv::VideoCapture();
                    introNextReady_ = false;
                } else {
                    introVideo.release();
                    introVideo.open(introPath_);
                }
                introStart = std::chrono::steady_clock::now();
                introIdx = -1;
                std::thread([this]() {
                    std::lock_guard<std::mutex> lk(audioMutex_);
                    if (introMusic) Mix_PlayMusic(introMusic, -1);
                }).detach();
                break;   // este ciclo se queda con el ultimo introFrame
            }
            ++introIdx; advanced = true;
        }
        if (advanced) introVideo.retrieve(introFrame);

        frame.setTo(cv::Scalar(0, 0, 0));
        if (!introFrame.empty()) {
            cv::Mat src = introFrame;
            if (src.channels() == 4) cv::cvtColor(src, src, cv::COLOR_BGRA2BGR);
            else if (src.channels() == 1) cv::cvtColor(src, src, cv::COLOR_GRAY2BGR);
            double k = std::min((double)W / src.cols, (double)H / src.rows);
            int w = std::max(1, (int)(src.cols * k)), h = std::max(1, (int)(src.rows * k));
            cv::Mat rs;
            cv::resize(src, rs, cv::Size(w, h), 0, 0, cv::INTER_AREA);
            rs.copyTo(frame(cv::Rect((W - w) / 2, (H - h) / 2, w, h)));
        }

        // franja oscura abajo para que se lea la barra
        {
            cv::Rect band(0, H - 78, W, 78);
            cv::Mat roi = frame(band);
            cv::Mat ov = roi.clone();
            ov.setTo(cv::Scalar(0, 0, 0));
            cv::addWeighted(ov, 0.5, roi, 0.5, 0, roi);
        }

        std::string status;
        int prog;
        {
            std::lock_guard<std::mutex> lk(configStatusMutex);
            status = configStatus;
            prog = configProgress.load();
        }
        const float pct = std::min(1.0f, (float)prog / configTotal);
        introBarShown_ += (pct - introBarShown_) * 0.08f;
        const cv::Scalar white(255, 255, 255), ink(60, 50, 50);

        const int barH = 14, barW = W * 72 / 100, barX = (W - barW) / 2, barY = H - 56;
        kd::fillRound(frame, cv::Rect(barX, barY, barW, barH), barH / 2, white, 0.35);
        int fillW = (int)(barW * introBarShown_);
        if (fillW > barH) kd::fillRound(frame, cv::Rect(barX, barY, fillW, barH), barH / 2, cv::Scalar(50, 215, 255));
        kd::strokeRound(frame, cv::Rect(barX, barY, barW, barH), barH / 2, white, 2);
        char pb[8];
        snprintf(pb, sizeof(pb), "%d%%", (int)(pct * 100));
        kd::text(frame, pb, cv::Point(barX + barW + 14, barY + barH), cv::FONT_HERSHEY_DUPLEX, 0.55, white, ink, 1);

        std::string st = status;
        int bl = 0;
        cv::Size ts = txu::size(st, cv::FONT_HERSHEY_DUPLEX, 0.55, 1, &bl);
        kd::text(frame, st, cv::Point((W - ts.width) / 2, H - 16), cv::FONT_HERSHEY_DUPLEX, 0.55, white, ink, 1);
        return true;
    }

    void renderLoadingScreen(cv::Mat& frame) {
        const int W = frame.cols, H = frame.rows;
        const double t = kd::now();
        static const double t00 = kd::now();
        const double ts = t - t00;

        std::string status;
        int prog;
        {
            std::lock_guard<std::mutex> lk(configStatusMutex);
            status = configStatus;
            prog   = configProgress.load();
        }
        const float pct = std::min(1.0f, (float)prog / configTotal);
        const bool done = pct >= 0.999f;
        const cv::Scalar white = kd::white();
        const cv::Scalar ink(60, 50, 50);

        // ---- escenas: se barajan en cada vuelta para que no se repita el orden ----
        const double ACT = 3.4;
        const int actN = (int)std::floor(ts / ACT);
        const double u = ts / ACT - actN;
        const double tt = u * ACT;

        static const int NSC = 16;
        static const int SL[NSC] = {1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1};   // slots por escena (robo de letra = 2)
        const int CYC = 16;
        auto rawOrder = [&](int c) {
            std::vector<int> v;
            for (int i2 = 0; i2 < NSC; ++i2) if (i2 != 9) v.push_back(i2);   // sin CUCU
            uint32_t sd = (uint32_t)c * 2654435761u + 12345u;
            for (int i2 = (int)v.size() - 1; i2 > 0; --i2) {
                sd = sd * 1664525u + 1013904223u;
                int j2 = (int)((sd >> 8) % (uint32_t)(i2 + 1));
                std::swap(v[i2], v[j2]);
            }
            return v;
        };
        const int cyc = actN / CYC;
        std::vector<int> ord = rawOrder(cyc);
        if (cyc > 0 && ord[0] == rawOrder(cyc - 1).back()) std::swap(ord[0], ord[1]);
        int act = 0, slotIn = 0;
        {
            int w = actN % CYC, acc = 0;
            for (int id : ord) {
                if (w < acc + SL[id]) { act = id; slotIn = w - acc; break; }
                acc += SL[id];
            }
        }
        if (actN == 0) { act = 0; slotIn = 0; }                          // siempre empieza saludando
        if (act == 7 && slotIn == 1 && actN == 1) { act = 1; slotIn = 0; }
        if (done) { act = 1; slotIn = 0; }
        const int hl = cyc % 5;                                          // letra que se roba en esta vuelta

        double env = std::max(0.0, std::min(1.0, std::min(u / 0.14, (1.0 - u) / 0.14)));
        env = env * env * (3.0 - 2.0 * env);

        // ---- fondo que cambia de color ----
        static const cv::Scalar BG[6] = {
            cv::Scalar(240, 200, 120), cv::Scalar(155, 205, 80), cv::Scalar(235, 160, 190),
            cv::Scalar(155, 230, 247), cv::Scalar(190, 150, 245), cv::Scalar(190, 190, 70)};
        const double COLOR_HOLD  = 12.0;   // segundos que dura cada color
        const double COLOR_BLEND = 3.0;    // segundos que tarda en mezclarse al siguiente
        const int    colN = (int)std::floor(ts / COLOR_HOLD);
        const double cu   = ts - colN * COLOR_HOLD;
        double kb = std::max(0.0, (cu - (COLOR_HOLD - COLOR_BLEND)) / COLOR_BLEND);
        kb = kb * kb * (3.0 - 2.0 * kb);
        const cv::Scalar bg = kd::mix(BG[colN % 6], BG[(colN + 1) % 6], kb);
        frame.setTo(bg);

        // escenario
        cv::ellipse(frame, cv::Point(W / 2, 384), cv::Size(340, 24), 0, 0, 360,
                    kd::mix(bg, white, 0.35), cv::FILLED, cv::LINE_AA);

        // ---- titulo: posicion de cada letra (para poder robarlas) ----
        const std::string title = "YAREN";
        const int tFont = cv::FONT_HERSHEY_DUPLEX;
        const double tsc = 1.4;
        const int tth = 3;
        const cv::Point tOrg(24, 66);
        int blT = 0;
        const int capH = txu::size("Y", tFont, tsc, tth, &blT).height;
        const double slotCy = tOrg.y - capH / 2.0;
        double slotCx[5], slotW[5];
        for (int i = 0; i < 5; ++i) {
            int pre = (i == 0) ? 0 : txu::size(title.substr(0, i), tFont, tsc, tth, &blT).width;
            int w = txu::size(title.substr(i, 1), tFont, tsc, tth, &blT).width;
            slotW[i] = w;
            slotCx[i] = tOrg.x + pre + w / 2.0;
        }

        auto sm = [](double a, double b, double x) {
            double k = std::max(0.0, std::min(1.0, (x - a) / (b - a)));
            return k * k * (3.0 - 2.0 * k);
        };
        auto lerp = [](double a, double b, double k) { return a + (b - a) * k; };
        auto mixP = [](cv::Point2d a, cv::Point2d b, double k) { return a + (b - a) * k; };
        auto cl = [](double v, double a, double b) { return std::max(a, std::min(b, v)); };

        // circulo translucido (burbujas)
        auto softCircle = [&](cv::Point c, int r, const cv::Scalar& col, double a) {
            cv::Rect rc(c.x - r - 2, c.y - r - 2, 2 * r + 5, 2 * r + 5);
            rc &= cv::Rect(0, 0, frame.cols, frame.rows);
            if (rc.area() <= 0) return;
            cv::Mat roi = frame(rc);
            cv::Mat ov = roi.clone();
            cv::circle(ov, c - rc.tl(), r, col, cv::FILLED, cv::LINE_AA);
            cv::addWeighted(ov, a, roi, 1.0 - a, 0, roi);
        };
        auto heart = [&](cv::Point2d c, double s, const cv::Scalar& col) {
            int r = std::max(1, (int)std::lround(s * 0.56));
            cv::circle(frame, cv::Point((int)(c.x - s * 0.5), (int)(c.y - s * 0.3)), r, col, cv::FILLED, cv::LINE_AA);
            cv::circle(frame, cv::Point((int)(c.x + s * 0.5), (int)(c.y - s * 0.3)), r, col, cv::FILLED, cv::LINE_AA);
            std::vector<cv::Point> tri = {
                {(int)(c.x - s * 1.0), (int)(c.y - s * 0.05)},
                {(int)(c.x + s * 1.0), (int)(c.y - s * 0.05)},
                {(int)c.x, (int)(c.y + s * 1.1)}};
            cv::fillPoly(frame, tri, col, cv::LINE_AA);
        };
        auto butterfly = [&](cv::Point2d c, double s, double rate) {
            double fl = std::fabs(std::sin(t * rate));
            int ww = std::max(1, (int)std::lround((3.0 + 12.0 * fl) * s));
            int wh = std::max(1, (int)std::lround(9.0 * s));
            cv::Point cp((int)c.x, (int)c.y);
            int off = (int)(ww * 0.7);
            cv::ellipse(frame, cv::Point(cp.x - off, cp.y - (int)(4 * s)), cv::Size(ww, wh), 0, 0, 360, kd::pal(5), cv::FILLED, cv::LINE_AA);
            cv::ellipse(frame, cv::Point(cp.x + off, cp.y - (int)(4 * s)), cv::Size(ww, wh), 0, 0, 360, kd::pal(5), cv::FILLED, cv::LINE_AA);
            cv::ellipse(frame, cv::Point(cp.x - (int)(ww * 0.5), cp.y + (int)(7 * s)), cv::Size(std::max(1, (int)(ww * 0.7)), std::max(1, (int)(6 * s))), 0, 0, 360, kd::pal(1), cv::FILLED, cv::LINE_AA);
            cv::ellipse(frame, cv::Point(cp.x + (int)(ww * 0.5), cp.y + (int)(7 * s)), cv::Size(std::max(1, (int)(ww * 0.7)), std::max(1, (int)(6 * s))), 0, 0, 360, kd::pal(1), cv::FILLED, cv::LINE_AA);
            cv::ellipse(frame, cp, cv::Size(std::max(1, (int)(2 * s)), std::max(2, (int)(8 * s))), 0, 0, 360, ink, cv::FILLED, cv::LINE_AA);
            cv::line(frame, cv::Point(cp.x, cp.y - (int)(7 * s)), cv::Point(cp.x - (int)(4 * s), cp.y - (int)(14 * s)), ink, 1, cv::LINE_AA);
            cv::line(frame, cv::Point(cp.x, cp.y - (int)(7 * s)), cv::Point(cp.x + (int)(4 * s), cp.y - (int)(14 * s)), ink, 1, cv::LINE_AA);
        };

        // ---- pose base ----
        double px = W / 2.0, g = 372.0, sc = 0.85, tilt = 0.0, sqx = 1.0, sqy = 1.0;
        double headTilt = 0.0;
        double lookX = 0.0, lookY = 0.0, es = 1.0;
        double jump = 4.0 + 4.0 * std::sin(t * 2.2);
        int emo = 7;
        const cv::Point2d baseL(-63, -33), baseR(63, -33);   // brazos colgando, un poco abiertos
        cv::Point2d tL = baseL, tR = baseR;

        bool balloon = false, juggle = false, wand = false, notes = false, bubbleOn = false, tapAct = false;
        bool hideEyes = false, cheeks = false, bubblesF = false, heliF = false, flyF = false;
        bool heartsF = false, zzzF = false, dizzyF = false, spinF = false;
        double balloonK = 0.0, balloonRise = 0.0, wandAng = 0.0, jugA0 = 0.0;
        double carryK = 0.0, letterJit = 0.0, heistTh = -1.0;
        double propK = 0.0, landK = 0.0, awayK = 0.0, bfx = 0.0, bfy = 0.0, dzA = 0.0, spinK = 0.0, spinAng = 0.0;
        const double tapT[3] = {1.0, 1.5, 2.0};
        const double fadeEdge = sm(0.0, 0.4, tt) * (1.0 - sm(3.0, 3.4, tt));
        std::string bubbleTxt;

        switch (act) {
            case 0: {   // saluda y mira a los lados
                lookX = std::sin(tt * 2.4);
                lookY = 0.3 * std::sin(tt * 1.7);
                tilt = 2.5 * std::sin(tt * 3.0) * env;
                tR = mixP(baseR, cv::Point2d(96, -148 + 14.0 * std::sin(tt * 9.0)), env);
                bubbleOn = (tt > 0.5 && tt < 2.9);
                bubbleTxt = isEnglish ? "Hi!" : "Hola!";
                break;
            }
            case 1: {   // saltos
                double ph = tt / ACT * 3.0;
                double h = std::fabs(std::sin(CV_PI * ph)) * 80.0;
                jump = h;
                double comp = std::max(0.0, 1.0 - h / 12.0);
                sqy = 1.0 - 0.14 * comp + 0.06 * (h / 80.0);
                sqx = 1.0 + 0.12 * comp - 0.04 * (h / 80.0);
                double air = std::min(1.0, h / 40.0);
                tL = mixP(baseL, cv::Point2d(-88, -168), air);
                tR = mixP(baseR, cv::Point2d(88, -168), air);
                emo = 2;
                break;
            }
            case 2: {   // se acerca a la pantalla, se sorprende (inclina el cuello) y golpea el vidrio
                sc = 0.85 + 0.45 * env;
                g = 372.0 + 62.0 * env;
                emo = (tt < 0.9) ? 3 : 7;
                es = 1.12;
                tapAct = true;
                headTilt = (10.0 + 2.0 * std::sin(tt * 4.0)) * env;
                bubbleOn = (tt > 0.2 && tt < 0.95);
                bubbleTxt = "!";
                double jab = 0.0;
                for (int k = 0; k < 3; ++k) {
                    double dt = tt - tapT[k];
                    if (dt >= 0.0 && dt < 0.3) jab = std::max(jab, std::sin(CV_PI * dt / 0.3));
                }
                tR = mixP(baseR, cv::Point2d(70, -150 - 12.0 * jab), env);
                lookX = 0.0; lookY = 0.0;
                break;
            }
            case 3: {   // saca un globo
                balloon = true;
                balloonK = std::min(1.0, tt / 0.7);
                if (u > 0.84) balloonRise = (u - 0.84) / 0.16 * 380.0;
                tR = mixP(baseR, cv::Point2d(96, -118), env);
                lookY = -0.9;
                lookX = 0.7 * std::sin(tt * 2.0);
                tilt = 3.0 * std::sin(tt * 2.2) * env;
                emo = (u > 0.84) ? 3 : 7;
                break;
            }
            case 4: {   // malabares
                juggle = true;
                jugA0 = tt * 0.55 * 2.0 * CV_PI * 1.6;
                double sw = std::sin(tt * 7.0);
                tL = mixP(baseL, cv::Point2d(-78, -92 + 8.0 * sw), env);
                tR = mixP(baseR, cv::Point2d(78, -92 - 8.0 * sw), env);
                lookX = std::cos(jugA0);
                lookY = -0.4;
                tilt = 2.0 * std::sin(tt * 2.0) * env;
                break;
            }
            case 5: {   // baila
                double sw = std::sin(tt * 5.0);
                tilt = 9.0 * sw * env;
                jump = 4.0 + 4.0 * std::sin(t * 2.2) + 12.0 * std::fabs(sw) * env;
                tL = mixP(baseL, cv::Point2d(-86, -150 + 26.0 * sw), env);
                tR = mixP(baseR, cv::Point2d(86, -150 - 26.0 * sw), env);
                emo = 2;
                lookX = std::sin(tt * 2.5);
                notes = true;
                break;
            }
            case 6: {   // varita magica
                wand = true;
                wandAng = 0.9 * std::sin(tt * 5.5);
                tR = mixP(baseR, cv::Point2d(88, -130), env);
                tilt = 2.0 * std::sin(tt * 2.0) * env;
                lookX = std::sin(wandAng) + 0.5;
                lookY = -0.8;
                break;
            }
            case 7: {   // ROBO DE LETRA (2 slots = 6.8 s)
                const double Th = slotIn * ACT + tt;
                heistTh = Th;
                sc = 0.85;
                const double reachX = 59.8, reachY = -163.7;
                const double xGrab = slotCx[hl] + reachX * sc;
                const double J = std::max(0.0, (g - slotCy) / sc + reachY);
                const double offX = W + 200.0;
                const cv::Point2d armUp(-60, -165), carry(-86, -126);
                double bobAmp = 0.0, bobF = 0.0;

                if (Th < 0.85)      { px = lerp(W / 2.0, xGrab, sm(0.0, 0.85, Th)); bobAmp = 4.0; bobF = 9.0; }
                else if (Th < 2.15) { px = xGrab; }
                else if (Th < 3.0)  { double k = (Th - 2.15) / 0.85; px = lerp(xGrab, offX, k * k); bobAmp = 12.0; bobF = 17.0; }
                else if (Th < 3.9)  { px = offX; }
                else if (Th < 4.9)  { double k = (Th - 3.9); px = lerp(offX, xGrab, 1.0 - (1.0 - k) * (1.0 - k)); bobAmp = 12.0; bobF = 17.0; }
                else if (Th < 6.0)  { px = xGrab; }
                else                { px = lerp(xGrab, W / 2.0, sm(6.0, 6.8, Th)); bobAmp = 4.0; bobF = 9.0; }
                if (bobAmp > 0.0) jump = 5.0 + bobAmp * std::fabs(std::sin(Th * bobF));

                for (int s2 = 0; s2 < 2; ++s2) {
                    const double b = (s2 == 0) ? 0.85 : 4.85;
                    const double j0 = b + 0.15;
                    if (Th >= b && Th < j0) {
                        double k = sm(b, j0, Th);
                        sqy = 1.0 - 0.14 * k; sqx = 1.0 + 0.10 * k;
                    } else if (Th >= j0 && Th < j0 + 0.65) {
                        double sn2 = std::sin(CV_PI * (Th - j0) / 0.65);
                        jump = J * sn2; sqy = 1.0 + 0.06 * sn2; sqx = 1.0 - 0.03 * sn2;
                    } else if (Th >= j0 + 0.65 && Th < j0 + 0.80) {
                        double k = 1.0 - (Th - j0 - 0.65) / 0.15;
                        sqy = 1.0 - 0.12 * k; sqx = 1.0 + 0.09 * k;
                    }
                }

                if (Th < 0.85)        tL = baseL;
                else if (Th < 1.0)    tL = mixP(baseL, armUp, sm(0.85, 1.0, Th));
                else if (Th < 1.325)  tL = armUp;
                else if (Th < 4.9)    tL = mixP(armUp, carry, sm(1.325, 1.65, Th));
                else if (Th < 5.325)  tL = mixP(carry, armUp, sm(4.9, 5.2, Th));
                else                  tL = mixP(armUp, baseL, sm(5.325, 5.75, Th));
                if (bobF > 15.0 && Th > 1.65) tL.y += 8.0 * std::sin(Th * bobF);

                if (bobF > 15.0) tR = cv::Point2d(63, -33 + 22.0 * std::sin(Th * bobF + CV_PI));
                if (Th >= 5.65 && Th < 6.5)
                    tR = mixP(baseR, cv::Point2d(116, -150 + 12.0 * std::sin(Th * 12.0)),
                              sm(5.65, 5.85, Th) * (1.0 - sm(6.2, 6.5, Th)));

                if (Th < 1.0)        { emo = 3; es = 1.1;  lookX = -0.8; lookY = -1.0; headTilt = -9.0 * sm(0.1, 0.5, Th); }
                else if (Th < 1.65)  { emo = 3; es = 1.15; lookX = -0.6; lookY = -1.0; headTilt = -9.0; }
                else if (Th < 2.15)  { emo = 4; headTilt = -9.0 * (1.0 - sm(1.65, 1.95, Th)); }
                else if (Th < 3.9)   { emo = 2; tilt = 9.0 * sm(2.15, 2.4, Th); }
                else if (Th < 4.9)   { emo = 4; tilt = -9.0 * (1.0 - sm(4.5, 4.9, Th)); }
                else if (Th < 5.65)  { emo = 3; es = 1.1; lookX = -0.6; lookY = -1.0; headTilt = -7.0; }
                else if (Th < 6.5)   { emo = (Th < 6.0) ? 2 : 4; headTilt = -7.0 * (1.0 - sm(5.65, 5.85, Th)); }

                if (Th > 1.7 && Th < 2.2) { bubbleOn = true; bubbleTxt = "Hehe!"; }

                if (Th < 1.325)       carryK = 0.0;
                else if (Th < 1.445)  carryK = (Th - 1.325) / 0.12;
                else if (Th < 5.205)  carryK = 1.0;
                else if (Th < 5.325)  carryK = 1.0 - (Th - 5.205) / 0.12;
                else                  carryK = 0.0;
                if (carryK > 0.5 && bobF > 15.0) letterJit = 3.0 * std::sin(Th * bobF);
                break;
            }
            case 10: {  // BURBUJAS
                bubblesF = true;
                tR = mixP(baseR, cv::Point2d(88 + 7.0 * std::cos(tt * 4.0), -128 + 7.0 * std::sin(tt * 4.0)), env);
                lookX = 0.7 * std::sin(tt * 1.8) * env;
                lookY = -0.7 * env;
                headTilt = 6.0 * std::sin(tt * 1.5) * env;
                emo = (tt > 1.6 && tt < 2.4) ? 2 : 7;
                break;
            }
            case 11: {  // MARIPOSA: la sigue con los ojos y se le posa en la antena
                flyF = true;
                landK = sm(2.0, 2.6, tt);
                awayK = sm(3.1, 3.4, tt);
                bfx = px + 200.0 * std::sin(tt * 1.4 + 0.6);
                bfy = 170.0 + 55.0 * std::sin(tt * 2.3);
                lookX = cl((bfx - px) / 140.0, -1.0, 1.0) * (1.0 - landK);
                lookY = cl((bfy - 150.0) / 90.0, -1.0, 1.0) * (1.0 - landK) - 1.0 * landK;
                headTilt = cl((bfx - px) / 140.0, -1.0, 1.0) * 8.0 * (1.0 - landK) + 3.0 * std::sin(tt * 3.0) * landK;
                tR = mixP(baseR, cv::Point2d(70, -168), sm(0.5, 1.0, tt) * (1.0 - landK));
                if (landK > 0.6 && awayK < 0.5) { emo = 3; es = 1.15; }
                break;
            }
            case 12: {  // TROMPO: gira, se marea y le salen estrellitas
                spinF = true;
                spinK = sm(0.35, 2.5, tt);
                spinAng = spinK * 4.0 * CV_PI;
                const double cs2 = std::cos(spinAng);
                sqx = std::max(0.07, std::fabs(cs2));
                const bool spinning = (tt > 0.35 && tt < 2.5);
                hideEyes = spinning && (cs2 < 0.0);
                if (spinning) { lookX = std::sin(spinAng); emo = 2; }
                jump += 46.0 * std::sin(CV_PI * sm(0.3, 2.6, tt));
                double arms = sm(0.3, 0.6, tt) * (1.0 - sm(2.4, 2.7, tt));
                tL = mixP(baseL, cv::Point2d(-112, -96), arms);
                tR = mixP(baseR, cv::Point2d(112, -96), arms);
                if (tt >= 2.5) {
                    dzA = 1.0 - sm(2.6, 3.4, tt);
                    dizzyF = true;
                    tilt = 9.0 * std::sin(tt * 9.0) * dzA;
                    lookX = std::cos(tt * 12.0);
                    lookY = std::sin(tt * 12.0);
                    emo = 3; es = 1.1;
                }
                break;
            }
            case 13: {  // HELICOPTERO: la antena se vuelve helice
                heliF = true;
                const double up = sm(0.3, 1.2, tt) * (1.0 - sm(2.5, 3.3, tt));
                propK = sm(0.15, 0.6, tt) * (1.0 - sm(2.9, 3.3, tt));
                jump += 110.0 * up + 5.0 * std::sin(tt * 3.0) * up;
                double arms = sm(0.2, 0.6, tt) * (1.0 - sm(2.7, 3.2, tt));
                tL = mixP(baseL, cv::Point2d(-112, -112), arms);
                tR = mixP(baseR, cv::Point2d(112, -112), arms);
                tilt = 5.0 * std::sin(tt * 2.0) * up;
                headTilt = 5.0 * std::sin(tt * 3.3) * up;
                emo = 2; lookY = -0.3;
                break;
            }
            case 14: {  // CORAZONES: se sonroja y flotan corazoncitos
                cheeks = true; heartsF = true;
                tL = mixP(baseL, cv::Point2d(-12, -84), env);
                tR = mixP(baseR, cv::Point2d(12, -84), env);
                emo = 2;
                headTilt = 8.0 * std::sin(tt * 2.2) * env;
                tilt = 3.0 * std::sin(tt * 2.2 + 1.0) * env;
                jump += 5.0 * std::fabs(std::sin(tt * 4.4)) * env;
                break;
            }
            case 15: {  // SUENO: se duerme con Zzz y se despierta de un susto
                zzzF = true;
                const double slp = sm(0.4, 1.6, tt);
                const bool asleep = tt < 2.45;
                if (tt < 0.4) emo = 7;
                else if (asleep) emo = 0;
                else { emo = 3; es = 1.3; }
                headTilt = asleep ? 12.0 * slp * (1.0 + 0.25 * std::sin(tt * 3.0)) : -6.0 * (1.0 - sm(2.45, 3.2, tt));
                tilt = 3.0 * slp * std::sin(tt * 1.5) * (1.0 - sm(2.45, 2.55, tt));
                sqy = 1.0 - 0.03 * slp * (asleep ? 1.0 : 0.0);
                if (tt >= 2.45 && tt < 2.9) jump += 30.0 * std::sin(CV_PI * (tt - 2.45) / 0.45);
                double wk = sm(2.45, 2.6, tt) * (1.0 - sm(2.9, 3.3, tt));
                tL = mixP(baseL, cv::Point2d(-92, -160), wk);
                tR = mixP(baseR, cv::Point2d(92, -160), wk);
                bubbleOn = (tt >= 2.5 && tt < 3.2);
                bubbleTxt = "!";
                break;
            }
            case 8:
            default: {  // revisa el progreso
                double barW = W * 0.72, barX = (W - barW) / 2.0;
                double tx = barX + barW * std::max(0.03f, pct);
                tx = std::max(90.0, std::min(W - 90.0, tx));
                px = W / 2.0 + (tx - W / 2.0) * env;
                sc = 0.85 - 0.25 * env;
                tR = mixP(baseR, cv::Point2d(70, -18), env);
                tL = mixP(baseL, cv::Point2d(-96, -150 + 12.0 * std::sin(tt * 8.0)), env);
                lookY = 1.0 * env;
                lookX = 0.6 * env;
                bubbleOn = env > 0.7;
                char pb[16];
                snprintf(pb, sizeof(pb), "%d%%", (int)(pct * 100));
                bubbleTxt = pb;
                break;
            }
        }
        if (done) {
            bubbleOn = true;
            bubbleTxt = isEnglish ? "Ready!" : "Listo!";
        }

        // ---- transformacion cuerpo ----
        LXf T;
        const double rad = tilt * CV_PI / 180.0;
        T.cs = std::cos(rad); T.sn = std::sin(rad);
        T.sx = sc * sqx; T.sy = sc * sqy;
        T.ox = px; T.oy = g - jump * sc;

        // ---- transformacion cabeza (pivote en el cuello) ----
        LXf TH = T;
        {
            cv::Point2d n = T.P(0.0, -115.0);
            TH.cx0 = 0.0; TH.cy0 = -115.0;
            TH.ox = n.x; TH.oy = n.y;
            double ra = (tilt + headTilt) * CV_PI / 180.0;
            TH.cs = std::cos(ra); TH.sn = std::sin(ra);
        }

        // sombra
        cv::ellipse(frame, cv::Point((int)px, (int)g + 3),
                    cv::Size((int)(78 * sc * (1.0 - std::min(0.5, jump / 260.0))), (int)(10 * sc + 2)),
                    0, 0, 360, kd::mix(bg, cv::Scalar(0, 0, 0), 0.22), cv::FILLED, cv::LINE_AA);

        // ---- paleta (amarillo YAREN) ----
        const cv::Scalar orange(48, 192, 252), orangeDk(18, 146, 228), orangeLt = kd::mix(orange, white, 0.55);
        const cv::Scalar rim(12, 120, 205);
        const cv::Scalar black(44, 40, 40), metal(58, 54, 54), yellow(50, 215, 255), screenC(22, 18, 18);

        LArm aL = ldSolveArm(-1, tL), aR = ldSolveArm(1, tR);

        // base (pedestal): aro + cono con ranuras
        ldRR(frame, T, cv::Rect(-28, -30, 56, 12), 6, black);
        ldPoly(frame, T, {cv::Point2d(-26, -20), cv::Point2d(26, -20), cv::Point2d(44, 0), cv::Point2d(-44, 0)}, metal);
        ldLine(frame, T, -15, -14, -18, -4, cv::Scalar(26, 23, 23), 3);
        ldLine(frame, T, 15, -14, 18, -4, cv::Scalar(26, 23, 23), 3);

        // cuello
        ldRR(frame, T, cv::Rect(-21, -122, 42, 18), 6, black);

        // torso
        ldRR(frame, T, cv::Rect(-46, -102, 92, 76), 36, orangeDk);
        ldRR(frame, T, cv::Rect(-46, -106, 92, 76), 36, orange);
        ldRRStroke(frame, T, cv::Rect(-46, -106, 92, 76), 36, rim, 1.8);
        ldRR(frame, T, cv::Rect(-32, -99, 28, 10), 5, orangeLt);
        {   // texto "YAREN" pequeno en el pecho (sigue la inclinacion y el squash del cuerpo)
            static cv::Mat chestMask;
            if (chestMask.empty()) {
                chestMask = cv::Mat::zeros(80, 200, CV_8UC1);
                int bl0 = 0, th0 = 3;
                cv::Size t1 = cv::getTextSize("YAREN", cv::FONT_HERSHEY_DUPLEX, 1.0, th0, &bl0);
                double sc0 = 78.0 / t1.width;                       // 78 px de canvas = 39 unidades locales
                cv::Size t2 = cv::getTextSize("YAREN", cv::FONT_HERSHEY_DUPLEX, sc0, th0, &bl0);
                cv::putText(chestMask, "YAREN", cv::Point((200 - t2.width) / 2, (80 + t2.height) / 2),
                            cv::FONT_HERSHEY_DUPLEX, sc0, cv::Scalar(255), th0, cv::LINE_AA);
            }
            // canvas (u,v) a 2x -> local (x,y) = (-50 + u/2, -100 + v/2) -> pantalla
            const double m00 = T.sx * T.cs, m01 = -T.sx * T.sn, m02 = T.ox;
            const double m10 = T.sy * T.sn, m11 = T.sy * T.cs,  m12 = T.oy;
            cv::Matx23d Mf(0.5 * m00, 0.5 * m01, -50.0 * m00 - 100.0 * m01 + m02,
                           0.5 * m10, 0.5 * m11, -50.0 * m10 - 100.0 * m11 + m12);
            const cv::Point2f cn[4] = {{0, 0}, {200, 0}, {200, 80}, {0, 80}};
            float bx0 = 1e9f, by0 = 1e9f, bx1 = -1e9f, by1 = -1e9f;
            for (int q = 0; q < 4; ++q) {
                float X = (float)(Mf(0, 0) * cn[q].x + Mf(0, 1) * cn[q].y + Mf(0, 2));
                float Y = (float)(Mf(1, 0) * cn[q].x + Mf(1, 1) * cn[q].y + Mf(1, 2));
                bx0 = std::min(bx0, X); by0 = std::min(by0, Y);
                bx1 = std::max(bx1, X); by1 = std::max(by1, Y);
            }
            cv::Rect bb((int)std::floor(bx0) - 1, (int)std::floor(by0) - 1,
                        (int)std::ceil(bx1 - bx0) + 3, (int)std::ceil(by1 - by0) + 3);
            cv::Rect vis = bb & cv::Rect(0, 0, frame.cols, frame.rows);
            if (vis.area() > 0) {
                cv::Matx23d Ms = Mf;
                Ms(0, 2) -= bb.x; Ms(1, 2) -= bb.y;
                cv::Mat warped;
                cv::warpAffine(chestMask, warped, Ms, bb.size(), cv::INTER_LINEAR);
                cv::Mat mk  = warped(cv::Rect(vis.x - bb.x, vis.y - bb.y, vis.width, vis.height));
                cv::Mat roi = frame(vis);
                const cv::Scalar textCol(40, 85, 135);
                for (int yy = 0; yy < roi.rows; ++yy)
                    for (int xx = 0; xx < roi.cols; ++xx) {
                        float al = mk.at<uchar>(yy, xx) / 255.f * 0.9f;
                        if (al <= 0.f) continue;
                        cv::Vec3b& pp3 = roi.at<cv::Vec3b>(yy, xx);
                        for (int c3 = 0; c3 < 3; ++c3)
                            pp3[c3] = cv::saturate_cast<uchar>(textCol[c3] * al + pp3[c3] * (1.f - al));
                    }
            }
        }

        // orejas (audifonos)
        const cv::Scalar earLight(250, 228, 205);
        ldRR(frame, TH, cv::Rect(-120, -200, 28, 60), 13, black);
        ldRR(frame, TH, cv::Rect(92, -200, 28, 60), 13, black);
        ldRR(frame, TH, cv::Rect(-114, -176, 9, 4), 2, earLight);
        ldRR(frame, TH, cv::Rect(105, -176, 9, 4), 2, earLight);

        // cabeza
        ldRR(frame, TH, cv::Rect(-98, -255, 196, 142), 62, orangeDk);
        ldRR(frame, TH, cv::Rect(-98, -258, 196, 142), 62, orange);
        ldRRStroke(frame, TH, cv::Rect(-98, -258, 196, 142), 62, rim, 1.8);
        ldRR(frame, TH, cv::Rect(-64, -250, 62, 11), 5, orangeLt);
        ldEll(frame, TH, 0, -244, 4.5, 4.5, black);                       // camara
        ldRR(frame, TH, cv::Rect(-73, -216, 146, 86), 34, orangeDk);      // bisel
        ldRR(frame, TH, cv::Rect(-70, -213, 140, 80), 31, screenC);       // pantalla
        ldRR(frame, TH, cv::Rect(-54, -207, 42, 5), 2, kd::mix(screenC, white, 0.16));

        // ojos (y mejillas)
        const double blink = (std::fmod(t, 3.3) < 0.13) ? 1.0 : 0.0;
        if (!hideEyes) {
            if (cheeks) {
                const cv::Scalar pink(170, 130, 255);
                ldEll(frame, TH, -57, -146, 11, 5.5, pink);
                ldEll(frame, TH, 57, -146, 11, 5.5, pink);
            }
            ldEyes(frame, TH, emo, lookX, lookY, blink, es);
        }
        // helice sobre la antena
        if (heliF && propK > 0.02) {
            const double a = tt * 38.0;
            ldEll(frame, TH, 0, -262, 66 * propK, 11 * propK, kd::mix(bg, white, 0.55), 1.5);
            for (int q = 0; q < 2; ++q) {
                double aa = a + q * CV_PI / 2.0;
                ldLine(frame, TH, -62.0 * std::cos(aa) * propK, -262 - 6.0 * std::sin(aa) * propK,
                                   62.0 * std::cos(aa) * propK, -262 + 6.0 * std::sin(aa) * propK, ink, 5);
            }
            ldEll(frame, TH, 0, -262, 6 * propK, 6 * propK, ink);
        }

        // brazos AL FINAL: siempre por delante de la cabeza y con contorno
        ldDrawArm(frame, T, aL);
        ldDrawArm(frame, T, aR);

        // ---- props de las escenas ----
        if (balloon && balloonK > 0.02) {
            double sway = 12.0 * std::sin(tt * 2.3);
            cv::Point2d bc(aR.H.x + 62.0 + sway, aR.H.y - 120.0 - balloonRise);
            double rx = 28.0 * balloonK, ry = 34.0 * balloonK;
            cv::Point2d kn(bc.x, bc.y + ry + 5.0);
            if (balloonRise <= 0.0) {
                cv::Point2d mid((aR.H.x + kn.x) / 2.0 + 10.0 * std::sin(tt * 3.0), (aR.H.y + kn.y) / 2.0);
                ldLine(frame, T, aR.H.x, aR.H.y, mid.x, mid.y, ink, 1.6);
                ldLine(frame, T, mid.x, mid.y, kn.x, kn.y, ink, 1.6);
            }
            ldEll(frame, T, bc.x, bc.y, rx, ry, kd::pal(0));
            ldEll(frame, T, bc.x - rx * 0.35, bc.y - ry * 0.35, rx * 0.22, ry * 0.30, kd::mix(kd::pal(0), white, 0.7));
            ldPoly(frame, T, {cv::Point2d(bc.x, bc.y + ry - 2), cv::Point2d(bc.x - 6 * balloonK, bc.y + ry + 7 * balloonK),
                              cv::Point2d(bc.x + 6 * balloonK, bc.y + ry + 7 * balloonK)}, kd::pal(0) * 0.8);
        }
        if (juggle) {
            for (int i = 0; i < 3; ++i) {
                double a = jugA0 + i * 2.0 * CV_PI / 3.0;
                double sv = std::sin(a);
                double bx = 82.0 * std::cos(a);
                double by = -92.0 - (sv > 0 ? 42.0 * sv : 14.0 * sv);
                ldEll(frame, T, bx, by, 10.5 * env, 10.5 * env, kd::pal(i * 2));
                ldEll(frame, T, bx - 3, by - 3, 3.5 * env, 3.5 * env, white);
            }
        }
        if (wand) {
            cv::Point2d vec = cv::Point2d(std::sin(wandAng) + 0.9, -0.9 * std::cos(wandAng)) * 55.0;
            cv::Point2d tip = aR.H + vec;
            ldLine(frame, T, aR.H.x, aR.H.y, tip.x, tip.y, ink, 4.5);
            auto star = [&](double R, const cv::Scalar& c) {
                std::vector<cv::Point2d> pts;
                for (int i = 0; i < 10; ++i) {
                    double r = (i % 2 == 0) ? R : R * 0.45;
                    double a = (-90.0 + i * 36.0) * CV_PI / 180.0 + t * 2.0;
                    pts.push_back(cv::Point2d(tip.x + r * std::cos(a), tip.y + r * std::sin(a)));
                }
                ldPoly(frame, T, pts, c);
            };
            star(17.0, orangeDk);
            star(14.0, yellow);
            cv::Point sp = lp(T, tip.x, tip.y);
            for (int k = 0; k < 10; ++k) {
                double ph = std::fmod(tt * 1.4 + k * 0.1, 1.0);
                double ang = k * 2.4 + tt;
                double r = 14.0 + ph * 60.0;
                cv::Point q((int)(sp.x + r * std::cos(ang)), (int)(sp.y + r * std::sin(ang)));
                kd::sparkle(frame, q, 3 + (int)((1.0 - ph) * 6.0), kd::mix(kd::pal(k), bg, ph * 0.7));
            }
        }
        if (notes) {
            auto note = [&](cv::Point p, const cv::Scalar& c, double s) {
                cv::ellipse(frame, p, cv::Size((int)(7 * s), (int)(5 * s)), -20, 0, 360, c, cv::FILLED, cv::LINE_AA);
                int th = std::max(1, (int)(2 * s));
                cv::line(frame, cv::Point(p.x + (int)(6 * s), p.y - 1), cv::Point(p.x + (int)(6 * s), p.y - (int)(22 * s)), c, th, cv::LINE_AA);
                cv::line(frame, cv::Point(p.x + (int)(6 * s), p.y - (int)(22 * s)), cv::Point(p.x + (int)(15 * s), p.y - (int)(14 * s)), c, th, cv::LINE_AA);
            };
            for (int k = 0; k < 5; ++k) {
                double ph = std::fmod(tt * 0.7 + k * 0.2, 1.0);
                cv::Point p((int)(px + (k - 2) * 82 + 14.0 * std::sin(tt * 3.0 + k)), (int)(g - 120 - ph * 190.0));
                note(p, kd::mix(kd::pal(k + 1), bg, ph * ph * 0.8), 1.5);
            }
        }
        if (tapAct) {
            cv::Point hp = lp(T, aR.H.x, aR.H.y);
            for (int k = 0; k < 3; ++k) {
                double dt = tt - tapT[k];
                if (dt >= 0.12 && dt < 0.7) {
                    int r = (int)((dt - 0.12) * 200.0) + 10;
                    cv::circle(frame, hp, r, kd::mix(bg, white, 1.0 - (dt - 0.12) / 0.58), 3, cv::LINE_AA);
                }
            }
        }
        // burbujas
        if (bubblesF) {
            cv::Point2d ringL(aR.H.x + 10.0, aR.H.y - 34.0);
            ldLine(frame, T, aR.H.x, aR.H.y, ringL.x, ringL.y + 10.0, ink, 3.5);
            ldEll(frame, T, ringL.x, ringL.y, 13, 13, ink, 3.0);
            cv::Point rp = lp(T, ringL.x, ringL.y);
            for (int k = 0; k < 8; ++k) {
                double ph = std::fmod(tt * 0.55 + k * 0.125, 1.0);
                double bx = rp.x + 30.0 * ph + 14.0 * std::sin(ph * 9.0 + k * 1.7);
                double by = rp.y - 200.0 * ph;
                if (ph > 0.93) {
                    kd::sparkle(frame, cv::Point((int)bx, (int)by), (int)(9 * fadeEdge) + 1, white);
                    continue;
                }
                int r = std::max(2, (int)((4.0 + 20.0 * ph + (k % 3) * 3.0) * fadeEdge));
                cv::Point c((int)bx, (int)by);
                softCircle(c, r, cv::Scalar(255, 240, 200), 0.35);
                cv::circle(frame, c, r, white, 2, cv::LINE_AA);
                cv::ellipse(frame, cv::Point(c.x - (int)(r * 0.4), c.y - (int)(r * 0.4)),
                            cv::Size(std::max(1, (int)(r * 0.25)), std::max(1, (int)(r * 0.15))), -45, 0, 360, white, cv::FILLED, cv::LINE_AA);
            }
        }
        // mariposa
        if (flyF) {
            cv::Point2d wp(bfx + 220.0 * (1.0 - sm(0.0, 0.6, tt)), bfy - 160.0 * (1.0 - sm(0.0, 0.6, tt)));
            cv::Point2d lp2 = TH.P(0.0, -270.0);
            cv::Point2d pos = wp + (lp2 - wp) * landK;
            pos.x += 90.0 * awayK; pos.y -= 330.0 * awayK;
            butterfly(pos, 1.5, 22.0 - 16.0 * landK * (1.0 - awayK));
        }
        // helicoptero: viento debajo
        if (heliF && jump > 25.0) {
            double bottom = T.oy - 32.0 * sc;
            for (int k = 0; k < 6; ++k) {
                double ph = std::fmod(tt * 3.0 + k * 0.17, 1.0);
                double y1 = bottom + ph * (g - bottom);
                double x1 = px + (k - 2.5) * 22.0;
                cv::line(frame, cv::Point((int)x1, (int)y1), cv::Point((int)x1, (int)(y1 + 14.0)),
                         kd::mix(white, bg, ph * 0.7), 2, cv::LINE_AA);
            }
        }
        // corazones
        if (heartsF) {
            cv::Point hb = lp(TH, 0.0, -270.0);
            for (int k = 0; k < 6; ++k) {
                double ph = std::fmod(tt * 0.55 + k * 0.17, 1.0);
                double hx = px + ((k % 3) - 1) * 58.0 + 18.0 * std::sin(ph * 7.0 + k);
                double hy = hb.y - ph * 170.0;
                double sz = (9.0 + (k % 3) * 3.0) * (0.6 + 0.6 * ph) * fadeEdge;
                if (sz < 1.0) continue;
                heart(cv::Point2d(hx, hy), sz, kd::mix(kd::pal(0), bg, ph * ph * 0.8));
            }
        }
        // Zzz
        if (zzzF && tt < 2.45) {
            cv::Point zb = lp(TH, 60.0, -260.0);
            double zf = sm(0.4, 0.9, tt);
            for (int k = 0; k < 3; ++k) {
                double ph = std::fmod(tt * 0.6 + k * 0.33, 1.0);
                double zs = (0.5 + 0.6 * ph) * zf;
                if (zs < 0.15) continue;
                kd::text(frame, "Z", cv::Point((int)(zb.x + ph * 50.0 + 6.0 * std::sin(ph * 9.0)), (int)(zb.y - ph * 90.0)),
                         cv::FONT_HERSHEY_DUPLEX, zs, kd::mix(white, bg, ph * ph * 0.6), ink, 2);
            }
        }
        // trompo: estrellitas girando y mareo
        if (spinF && spinK > 0.02 && spinK < 0.98) {
            for (int k = 0; k < 4; ++k) {
                double a = spinAng * 1.5 + k * 1.571;
                kd::sparkle(frame, cv::Point((int)(px + 135.0 * std::cos(a)), (int)(g - 150.0 + 42.0 * std::sin(a))),
                            5 + (k % 2) * 3, kd::pal(k + 1));
            }
        }
        if (dizzyF && dzA > 0.05) {
            cv::Point db = lp(TH, 0.0, -278.0);
            for (int k = 0; k < 3; ++k) {
                double a = tt * 6.0 + k * 2.094;
                kd::sparkle(frame, cv::Point((int)(db.x + 58.0 * std::cos(a)), (int)(db.y + 14.0 * std::sin(a))),
                            6, kd::mix(kd::pal(k * 2 + 2), bg, 1.0 - dzA));
            }
        }

        // ---- titulo "YAREN" letra por letra (una puede estar en la mano del robot) ----
        {
            const cv::Point2d hand = T.P(aL.H.x, aL.H.y);
            for (int i = 0; i < 5; ++i) {
                cv::Point2d pos(slotCx[i], slotCy);
                double k = (i == hl) ? carryK : 0.0;
                double s2 = tsc;
                if (k > 0.0) {
                    cv::Point2d hp(hand.x - 3.0, hand.y - 12.0 + letterJit);
                    pos = pos + (hp - pos) * k;
                    s2 = tsc * (1.0 + 0.3 * k);
                    cv::line(frame,
                             cv::Point((int)(slotCx[i] - slotW[i] / 2.0), tOrg.y + 5),
                             cv::Point((int)(slotCx[i] + slotW[i] / 2.0), tOrg.y + 5),
                             kd::mix(bg, white, 0.7), 4, cv::LINE_AA);
                }
                // al volver a su sitio: rebota y suelta chispitas
                if (i == hl && heistTh >= 5.325 && heistTh < 6.2) {
                    double dtp = heistTh - 5.325;
                    if (dtp < 0.52) pos.y -= 14.0 * std::fabs(std::sin(CV_PI * dtp / 0.52));
                    double fade = dtp / 0.875;
                    for (int q = 0; q < 6; ++q) {
                        double aq = q * 1.047 + 0.4;
                        double rq = 12.0 + dtp * 55.0;
                        kd::sparkle(frame, cv::Point((int)(slotCx[i] + rq * std::cos(aq)), (int)(slotCy + rq * 0.8 * std::sin(aq))),
                                    4 + (q % 2) * 3, kd::mix(kd::pal(q), bg, fade));
                    }
                }
                int blL = 0;
                cv::Size ls = txu::size(title.substr(i, 1), tFont, s2, tth, &blL);
                kd::text(frame, title.substr(i, 1),
                         cv::Point((int)(pos.x - ls.width / 2.0), (int)(pos.y + ls.height / 2.0)),
                         tFont, s2, white, ink, tth);
            }
        }

        if (bubbleOn) {
            cv::Point an = lp(T, 70, -262);
            an.x += 14;
            int bl = 0;
            cv::Size ts2 = txu::size(bubbleTxt, cv::FONT_HERSHEY_DUPLEX, 0.8, 2, &bl);
            cv::Rect r(an.x, an.y - ts2.height - 22, ts2.width + 36, ts2.height + 26);
            r.x = std::max(8, std::min(W - r.width - 8, r.x));
            r.y = std::max(8, r.y);
            kd::card(frame, r, 18, kd::mix(bg, cv::Scalar(0, 0, 0), 0.25), 2);
            std::vector<cv::Point> tail = {{r.x + 18, r.y + r.height - 2}, {r.x + 8, r.y + r.height + 14}, {r.x + 34, r.y + r.height - 2}};
            cv::fillPoly(frame, tail, white, cv::LINE_AA);
            txu::put(frame, bubbleTxt, cv::Point(r.x + 18, r.y + r.height - 12),
                        cv::FONT_HERSHEY_DUPLEX, 0.8, ink, 2, cv::LINE_AA);
        }

        // ---- barra de progreso ----
        {
            const int barH = 14, barW = W * 72 / 100, barX = (W - barW) / 2, barY = 408;
            kd::fillRound(frame, cv::Rect(barX, barY, barW, barH), barH / 2, white, 0.35);
            int fillW = (int)(barW * pct);
            if (fillW > barH) kd::fillRound(frame, cv::Rect(barX, barY, fillW, barH), barH / 2, white);
            kd::strokeRound(frame, cv::Rect(barX, barY, barW, barH), barH / 2, white, 2);
            char pctBuf[8];
            snprintf(pctBuf, sizeof(pctBuf), "%d%%", (int)(pct * 100));
            kd::text(frame, pctBuf, cv::Point(barX + barW + 14, barY + barH), cv::FONT_HERSHEY_DUPLEX, 0.55, white, ink, 1);
        }

        // ---- estado ----
        {
            std::string displayText = status;
            if (!done && std::fmod(t * 2.2, 1.0) < 0.5) displayText += "_";
            int bl = 0;
            cv::Size ts2 = txu::size(displayText, cv::FONT_HERSHEY_DUPLEX, 0.55, 1, &bl);
            kd::text(frame, displayText, cv::Point((W - ts2.width) / 2, H - 24),
                     cv::FONT_HERSHEY_DUPLEX, 0.55, white, ink, 1);
        }
    }

    void renderLoop() {
        rclcpp::Rate rate(30);
        while (running && rclcpp::ok()) {
            
            // 1. Obtener banderas de menús activos
            bool sSettings, sRadio, sRoutines, sWifi;
            {
                std::lock_guard<std::mutex> lk(modeFlagMutex);
                sSettings  = showSettings_;
                sRadio     = showRadio_;
                sRoutines  = showRoutines_;
                sWifi      = showWifiSetup_;
            }

            cv::Mat frame;

            // 2. Decidir el fondo: Si Yaren se está configurando, usar la pantalla de carga. Si no, usar la cara.
            if (configuring.load()) {
                frame = cv::Mat(cv::Size(800, 480), CV_8UC3);
                if (!renderIntroVideo(frame)) renderLoadingScreen(frame);
            } else {
                if (introVideo.isOpened()) introVideo.release();
                frame = getFaceFrame();
            }

            // Ahora dibujamos el WiFi ENCIMA del fondo, incluso si sigue configurando
            if (sWifi) {
                wifiSetup_.render(frame);
            } else if (!configuring.load()) {
                // Solo dibujamos los otros menús si no estamos en configuración ni en WiFi
                bool sSettings, sRadio, sRoutines;
                {
                    std::lock_guard<std::mutex> lk(modeFlagMutex);
                    sSettings = showSettings_; sRadio = showRadio_; sRoutines = showRoutines_;
                }
                if (sSettings) settingsMenu.render(frame);
                else if (sRadio) radioApp.render(frame);
                else if (sRoutines) routinesApp.render(frame);
                else {
                    renderFaceOverlay(frame);
                    std::lock_guard<std::mutex> lock(navMutex);
                    if (!navStack.empty()) {
                        renderMenu(frame);
                        renderSettingsButton(frame);
                        renderPowerButton(frame);
                    }
                }
            }

            // 4. DETERMINAR SI YAREN ESTÁ INACTIVO
            // Solo se activa el screensaver si Yaren está en la cara pura:
            // sin menú, sin modo activo, sin settings, sin radio, sin rutinas,
            // sin WiFi, sin overlay activo y sin música de menú sonando.
            bool currentIdle = false;
            {
                std::lock_guard<std::mutex> lock(navMutex);
                FaceOverlay ovState;
                { std::lock_guard<std::mutex> lk(overlayMutex); ovState = faceOverlay; }

                currentIdle = navStack.empty()
                           && activeMode.empty()
                           && active_lifecycle_nodes.empty()
                           && !sSettings
                           && !sRadio
                           && !sRoutines
                           && !sWifi
                           && !micTestRunning.load()
                           && !ttsActive.load()
                           && ovState == FaceOverlay::NONE
                           && !isMenuMusicPlaying;
            }

            // 5. LÓGICA DEL SCREENSAVER OPTIMIZADA
            auto now = std::chrono::system_clock::now();

            // Si Yaren NO está inactivo (está en un menú, cámara, configuración, IA, etc.)
            // mantenemos el reloj de inactividad congelado en el momento actual.
            if (!currentIdle || configuring.load() || sWifi) {
                lastInteractionTime = now;
                if (isIdleScreenActive) {
                    stopIdleScreen();
                }
            }

            // Aquí el tiempo solo empezará a crecer a partir de 0 en el instante exacto 
            // en el que currentIdle se vuelva 'true' (es decir, al regresar a la cara principal).
            double elapsedIdle = std::chrono::duration<double>(now - lastInteractionTime).count();

            if (currentIdle && elapsedIdle > idleTimeoutSecs) {
                if (!isIdleScreenActive) {
                    startIdleScreen();
                }
            }

            if (isIdleScreenActive && idleVideo.isOpened()) {
                cv::Mat videoFrame;
                idleVideo >> videoFrame; 
                
                if (videoFrame.empty()) {
                    idleVideo.set(cv::CAP_PROP_POS_FRAMES, 0);
                    idleVideo >> videoFrame;
                }
                if (!videoFrame.empty()) {
                    cv::resize(videoFrame, frame, frame.size(), 0, 0, cv::INTER_LINEAR);
                    txu::put(frame, "Zzz...", {30, frame.rows - 30}, 
                                cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                }
            }
            // Estado para la app remota
            {
                std::string uiState;
                if (configuring.load()) {
                    uiState = "loading";
                } else if (sWifi) {
                    uiState = "wifi";
                } else if (sSettings) {
                    uiState = "settings";
                } else if (sRadio) {
                    uiState = "mode:radio_musica"; 
                } else if (sRoutines) {
                    uiState = "routines";
                } else {
                    std::lock_guard<std::mutex> lock(navMutex);
                    if (!navStack.empty()) {
                        if (navStack.size() == 1)            uiState = "menu_root";
                        else if (!navStack.back().key.empty()) uiState = navStack.back().key;
                        else                                  uiState = navStack.back().title;
                    } else if (!activeMode.empty()) {
                        uiState = "mode:" + activeMode;
                    } else {
                        uiState = "face";
                    }
                }
                if (uiState != lastUIState_) {
                    lastUIState_ = uiState;
                    std_msgs::msg::String m; m.data = uiState;
                    uiStatePub_->publish(m);
                }
            }
            // 6. PUBLICAR ESTADOS
            {
                std::lock_guard<std::mutex> lock(idleStateMutex);
                if (currentIdle != lastIdleState) {
                    lastIdleState = currentIdle;
                    auto idleMsg = std_msgs::msg::Bool();
                    idleMsg.data = currentIdle;
                    idleStatePublisher->publish(idleMsg);
                    RCLCPP_INFO(this->get_logger(), "Estado de escucha de Yaren cambiado a: %s", currentIdle ? "ACTIVO" : "PAUSADO");
                }
            }

            if (!frame.empty()) {
                { std::lock_guard<std::mutex> lk(frameMutex); latestFrame = frame.clone(); }
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                faceScreenPublisher->publish(*msg);
            }
            rate.sleep();
        }
    }
    void loadFrames(const std::string& dir, std::vector<cv::Mat>& frames) {
        if (!fs::exists(dir)) return;
        std::vector<std::string> files;
        for (const auto& e : fs::directory_iterator(dir))
            if (e.path().extension() == ".png") files.push_back(e.path().string());
        std::sort(files.begin(), files.end());
        for (const auto& f : files) { cv::Mat m = cv::imread(f); if (!m.empty()) frames.push_back(m); }
    }

    void audioPlayingCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        ttsActive = msg->data;
        std::lock_guard<std::mutex> lk(audioMutex_);
        if (isMenuMusicPlaying) {
            if (ttsActive) Mix_PauseMusic();
            else Mix_ResumeMusic();
        }
    }

    int phonemeToSprite(const std::string& ph) {
        static const std::map<std::string,int> M = {
            {"_",0},{"pau",0},{"",0},
            {"a",7},{"e",5},{"i",4},{"o",3},{"u",2},
            {"j",4},{"w",2},{"p",1},{"b",1},{"m",0},
            {"f",5},{"v",1},{"t",1},{"d",1},
            {"s",4},{"z",4},{"n",1},{"l",1},{"r",1},
            {"k",1},{"g",1},{"x",6},{"tʃ",4},{"ɲ",1},
        };
        auto it = M.find(ph); return it!=M.end() ? it->second : 1;
    }
    void overlayImageAlpha(cv::Mat& bg, const cv::Mat& fg, float ga) {
        if (fg.empty() || bg.empty() || ga < 0.01f) return;
        cv::Mat src = fg;
        if (fg.size() != bg.size())
            cv::resize(fg, src, bg.size(), 0, 0, cv::INTER_LINEAR);
        if (src.channels() != 4) return;
        for (int y = 0; y < bg.rows; ++y)
            for (int x = 0; x < bg.cols; ++x) {
                const cv::Vec4b& f = src.at<cv::Vec4b>(y,x);
                float a = (f[3]/255.f) * ga;
                if (a > 0.01f) {
                    cv::Vec3b& b = bg.at<cv::Vec3b>(y,x);
                    for (int c=0;c<3;++c)
                        b[c]=cv::saturate_cast<uchar>(f[c]*a+b[c]*(1.f-a));
                }
            }
    }

    void updateViseme() {
        std::lock_guard<std::mutex> lk(visemeMutex_);
        if (!ttsActive.load()) {

            currentVisemeIdx_=0; nextVisemeIdx_=0; visemeBlend_=0.f; return;

        }
        auto now = std::chrono::steady_clock::now();
        if (now >= visemeDeadline_ && !visemeQueue_.empty()) {
            auto f = visemeQueue_.front(); visemeQueue_.pop();
            currentVisemeIdx_ = nextVisemeIdx_.load();
            nextVisemeIdx_    = f.idx;
            visemeBlend_      = 0.0f;
            visemeDeadline_   = now + std::chrono::milliseconds(f.duration_ms);
        }
        float elapsed = std::chrono::duration<float>(
            now - (visemeDeadline_ - std::chrono::milliseconds(80))).count();
            visemeBlend_ = std::min(1.0f, std::max(0.0f, elapsed / 0.07f));
    } 
    void overlayImage(cv::Mat& bg, const cv::Mat& fg) {
        if (fg.empty() || bg.empty()) return;
        cv::Mat fgResized;
        const cv::Mat* src = &fg;
        if (fg.size() != bg.size()) {
            cv::resize(fg, fgResized, bg.size(), 0, 0, cv::INTER_AREA);
            src = &fgResized;
        }
        if (src->channels() == 4) {
            for (int y = 0; y < bg.rows; ++y)
                for (int x = 0; x < bg.cols; ++x) {
                    const cv::Vec4b& f = src->at<cv::Vec4b>(y, x);
                    float a = f[3]/255.f;
                    if (a > 0) {
                        cv::Vec3b& b = bg.at<cv::Vec3b>(y, x);
                        for (int c = 0; c < 3; ++c) b[c] = (uchar)(f[c]*a + b[c]*(1.f-a));
                    }
                }
        } else { src->copyTo(bg); }
    }

    cv::Mat eyesOpenImg, eyesClosedImg, mouthOpenImg, mouthClosedImg;
    cv::Mat yarenSplashImg;
    cv::Mat menuBg_;
    cv::Mat faceOpenImg_, faceClosedImg_, faceTalkImg_;
    std::vector<cv::Mat> eyesFrames, mouthFrames;
    std::map<std::string, cv::Mat> iconMap;
    std::atomic<bool> ttsActive  { false };
    std::atomic<bool> isBlinking { false };
    std::atomic<bool> running    { false };
    std::atomic<bool> configuring{true};
    std::atomic<int>  configProgress{0};
    static constexpr int configTotal{20};
    std::string       configStatus{"Iniciando sistema..."};
    std::mutex        configStatusMutex;
    std::atomic<int>  hoveredItem { -1 };
    std::atomic<bool> hoveredBack { false };
    std::atomic<bool> hoveredStop { false };
    std::atomic<bool> hoveredExit { false };
    std::string activeMode {};
    std::string activeStopCmd {};
    std::vector<std::string> active_lifecycle_nodes;
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> lifecycle_clients_;
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lifecycle_state_clients_;
    
    uint8_t get_lifecycle_state(const std::string& node_name) {
    std::string service = "/" + node_name + "/get_state";
    if (lifecycle_state_clients_.find(node_name) == lifecycle_state_clients_.end()) {
        lifecycle_state_clients_[node_name] =
            this->create_client<lifecycle_msgs::srv::GetState>(service);
    }
    auto& client = lifecycle_state_clients_[node_name];
    if (!client->wait_for_service(std::chrono::milliseconds(2000)))
        return 0;
    auto future = client->async_send_request(
        std::make_shared<lifecycle_msgs::srv::GetState::Request>());
    if (future.wait_for(std::chrono::milliseconds(2000)) == std::future_status::ready)
        return future.get()->current_state.id;
    return 0;
}

    void ensure_lifecycle_active(const std::string& node_name) {
        uint8_t state = get_lifecycle_state(node_name);
        if (state == 1 || state == 0) {
            change_lifecycle_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
            std::this_thread::sleep_for(std::chrono::seconds(3)); // No congelará la cara porque estará en un hilo
        }
        change_lifecycle_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }

    void change_lifecycle_state(const std::string& node_name, uint8_t transition_id) {
        std::string service = "/" + node_name + "/change_state";
        if (lifecycle_clients_.find(node_name) == lifecycle_clients_.end()) {
            lifecycle_clients_[node_name] = this->create_client<lifecycle_msgs::srv::ChangeState>(service);
        }
        auto& client = lifecycle_clients_[node_name];
        if (!client->wait_for_service(std::chrono::milliseconds(1000))) {
            RCLCPP_WARN(this->get_logger(), "Servicio Lifecycle no disponible para: %s", node_name.c_str());
            return;
        }
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;
        client->async_send_request(request);
    }

    cv::Rect stopButtonRect {0,0,0,0};
    cv::Rect backButtonRect {0,0,0,0};
    cv::Rect exitButtonRect {0,0,0,0};
    std::chrono::system_clock::time_point lastBlinkTime, blinkStartTime;
    std::vector<MenuItem>           rootMenuItems;
    std::map<std::string, NavLevel> subMenuMap;
    std::vector<NavLevel>           navStack;
    FaceOverlay       faceOverlay    { FaceOverlay::NONE };
    std::string       overlayMessage {};
    std::atomic<int>  micCountdownSecs { 0 };
    std::atomic<bool> micTestRunning   { false };
    std::mutex        overlayMutex;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  ttsSubscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  sttListeningSubscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lipsyncSub_;
    std::atomic<bool> first_listen_done { false };
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr faceScreenPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   modePublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr micOwnerPublisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     languagePublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     idleStatePublisher;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr     micTestPublisher_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  wakeEventSubscription;
    static constexpr const char* kCommandTopic = "/yaren/command";
    static constexpr const char* kUiStateTopic = "/yaren/ui_state";
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandSub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    uiStatePub_;
    std::string lastUIState_;

    std::mutex idleStateMutex;
    bool lastIdleState { false };

    std::thread renderThread;
    std::mutex  frameMutex, navMutex;
    cv::Mat     latestFrame;

    // --- VARIABLES DE PANTALLA DE INACTIVIDAD (AQUÍ DEBEN IR) ---
    std::chrono::system_clock::time_point lastInteractionTime;
    bool isIdleScreenActive { false };
    double idleTimeoutSecs { 30.0 }; // Segundos de inactividad
    cv::VideoCapture idleVideo;
    Mix_Music* idleMusic { nullptr };
    std::vector<std::string> idleVideoPaths;
    std::vector<std::string> idleMusicPaths;
    std::vector<std::string> bootMusicPaths;
    Mix_Music* bootMusic { nullptr };
    // Intro en video
    cv::VideoCapture introVideo;
    cv::Mat introFrame;
    Mix_Music* introMusic { nullptr };
    bool   introAudioOk_ { false };
    double introFps { 30.0 };
    long   introIdx { -1 };
    float  introBarShown_ { 0.f };
    std::chrono::steady_clock::time_point introStart;
    std::string       introPath_;
    cv::VideoCapture  introNext_;
    std::atomic<bool> introNextReady_{false};
    std::atomic<bool> introNextLoading_{false};
    double            introDur_{0.0};
    // Lip sync
    cv::Mat mouthSprites_[9];          // 0-8 PNG de boca
    std::atomic<int>  currentVisemeIdx_  { 0 };
    std::atomic<int>  nextVisemeIdx_     { 0 };
    float             visemeBlend_       { 0.0f };
    std::chrono::steady_clock::time_point visemeDeadline_;
    std::mutex        visemeMutex_;
    struct VisemeFrame { int idx; uint32_t duration_ms; };
    std::queue<VisemeFrame> visemeQueue_;
};

// =============================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoSynchronizer>();
    node->startRenderThread();
    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->drawWindow();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}