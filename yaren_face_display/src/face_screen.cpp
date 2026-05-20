#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
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

// 🎵 SDL2 y SDL_mixer para audio profesional
#include <SDL2/SDL.h>
#include <SDL2/SDL_mixer.h>

namespace fs = std::filesystem;

// ─────────────────────────────────────────────────────────────────────────────
//  Estructura de dispositivo ALSA
// ─────────────────────────────────────────────────────────────────────────────
struct AlsaDevice {
    std::string hwId;   // e.g. "plughw:2,0"
    std::string label;  // e.g. "USB Audio [USB Audio], device 0"
};

// ─────────────────────────────────────────────────────────────────────────────
//  Parsea la salida de arecord -l / aplay -l
// ─────────────────────────────────────────────────────────────────────────────
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
            // Blacklist para fuentes: excluir monitores, virtuales y NVIDIA
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
// Obtiene el sink/source por defecto actual del sistema
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
// ─────────────────────────────────────────────────────────────────────────────
//  Helper: dibuja texto centrado horizontalmente
// ─────────────────────────────────────────────────────────────────────────────
static void drawCenteredText(cv::Mat& frame, const std::string& txt,
                             int totalW, int y,
                             int font, double scale,
                             const cv::Scalar& color, int thick = 1) {
    int bl = 0;
    cv::Size s = cv::getTextSize(txt, font, scale, thick, &bl);
    cv::putText(frame, txt, {(totalW - s.width) / 2, y},
                font, scale, color, thick, cv::LINE_AA);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Helper: dibuja texto centrado en un rect
// ─────────────────────────────────────────────────────────────────────────────
static void drawTextInRect(cv::Mat& frame, const std::string& txt,
                           const cv::Rect& r,
                           int font, double scale,
                           const cv::Scalar& color, int thick = 1) {
    int bl = 0;
    cv::Size s = cv::getTextSize(txt, font, scale, thick, &bl);
    cv::putText(frame, txt,
                {r.x + (r.width  - s.width)  / 2,
                 r.y + (r.height + s.height) / 2 - 2},
                font, scale, color, thick, cv::LINE_AA);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Helper: flecha triangular ▲ / ▼
// ─────────────────────────────────────────────────────────────────────────────
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
//  SettingsMenu  —  panel completo de configuración
// =============================================================================
class SettingsMenu {
public:
    int volumeLevel = 64;           
    int previousVolume = 64;        
    bool isMuted = false;
    bool isDraggingVolume = false;  
    cv::Rect muteBtnRect {0, 0, 0, 0};
    cv::Rect sliderTrackRect {0, 0, 0, 0};
    
    std::function<void(const std::string&)> onMicSelected;
    std::function<void(const std::string&)> onSpkSelected;

    std::string selectedMicId;
    std::string selectedSpkId;

    SettingsMenu() {
        std::time_t t  = std::time(nullptr);
        std::tm* tm = std::localtime(&t);
        editDay  = tm->tm_mday;
        editMon  = tm->tm_mon + 1;
        editYear = tm->tm_year + 1900;
        editHour = tm->tm_hour;
        editMin  = tm->tm_min;
        refresh();
    }

    cv::Rect powerButtonRect {0,0,0,0};
    bool     hoveredPower    {false};

    void refresh() {
        mics = parsePulseDevices(false);   // sources (micrófonos)
        spks = parsePulseDevices(true);    // sinks   (parlantes)

        if (mics.empty()) mics.push_back({"", "(sin micrófonos detectados)"});
        if (spks.empty()) spks.push_back({"", "(sin parlantes detectados)"});

        // Pre-seleccionar el dispositivo actualmente activo en el sistema
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

        // Leer fecha/hora actual
        std::time_t t = std::time(nullptr);
        std::tm* tm   = std::localtime(&t);
        editDay  = tm->tm_mday;
        editMon  = tm->tm_mon + 1;
        editYear = tm->tm_year + 1900;
        editHour = tm->tm_hour;
        editMin  = tm->tm_min;

        initialMicId = selectedMicId;
        initialSpkId = selectedSpkId;
        initDay  = editDay;  initMon  = editMon;
        editYear_init = editYear;
        initHour = editHour; initMin  = editMin;
    }

    void render(cv::Mat& frame) {
        if (frame.empty()) return;
        W = frame.cols;
        H = frame.rows;

        cv::Mat ov = frame.clone();
        cv::rectangle(ov, {0, 0, W, H}, cv::Scalar(4, 10, 22), cv::FILLED);
        cv::addWeighted(ov, 0.95, frame, 0.05, 0, frame);

        drawCenteredText(frame, "CONFIGURACION", W, 32,
                         cv::FONT_HERSHEY_DUPLEX, 0.80, cv::Scalar(0, 229, 255), 2);
        cv::line(frame, {W/2-320, 48}, {W/2+320, 48}, cv::Scalar(0, 80, 120), 1, cv::LINE_AA);

        int micPanelH = 105; 
        int spkPanelH = 105;
        int datPanelH = 85;
        int volPanelH = 65;

        int micTop = 58;
        int spkTop = micTop + micPanelH;
        int datTop = spkTop + spkPanelH;
        int volTop = datTop + datPanelH;

        renderAudioPanel(frame, micTop, micPanelH, true);
        renderAudioPanel(frame, spkTop, spkPanelH, false);
        renderDatePanel (frame, datTop, datPanelH);
        
        renderVolumePanel(frame, 20, volTop + 5, W - 40, volPanelH - 10);
        
        renderBottomBar(frame, H - 62);
    }

    void renderVolumePanel(cv::Mat& frame, int x, int y, int w, int h) {
        cv::Rect panelRect{x, y, w, h};
        cv::rectangle(frame, panelRect, cv::Scalar(10, 18, 32), cv::FILLED);
        cv::rectangle(frame, panelRect, cv::Scalar(50, 65, 85), 1, cv::LINE_AA);
        
        cv::putText(frame, "VOLUMEN DE MUSICA", {x + 15, y + 18}, cv::FONT_HERSHEY_PLAIN, 0.80, cv::Scalar(150, 160, 180), 1, cv::LINE_AA);

        int btnSz = 26; 
        muteBtnRect = {x + 15, y + 24, btnSz, btnSz};
        bool hovMute = muteBtnRect.contains(hoveredRect.tl());
        cv::Scalar muteColor = isMuted ? cv::Scalar(40, 40, 220) : (hovMute ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 229, 255));

        cv::rectangle(frame, muteBtnRect, hovMute ? cv::Scalar(40,50,70) : cv::Scalar(20,28,42), cv::FILLED);
        cv::rectangle(frame, muteBtnRect, muteColor, 1, cv::LINE_AA);

        int cx = muteBtnRect.x + btnSz/2 - 2, cy = muteBtnRect.y + btnSz/2;
        std::vector<cv::Point> speaker = { {cx - 5, cy - 3}, {cx - 2, cy - 3}, {cx + 3, cy - 6}, {cx + 3, cy + 6}, {cx - 2, cy + 3}, {cx - 5, cy + 3} };
        cv::fillPoly(frame, speaker, muteColor, cv::LINE_AA);
        
        if (isMuted) {
            cv::line(frame, {muteBtnRect.x + 4, muteBtnRect.y + 4}, {muteBtnRect.x + btnSz - 4, muteBtnRect.y + btnSz - 4}, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
        } else {
            cv::ellipse(frame, {cx + 1, cy}, {5, 5}, 0, -45, 45, muteColor, 1, cv::LINE_AA);
            cv::ellipse(frame, {cx + 1, cy}, {8, 8}, 0, -45, 45, muteColor, 1, cv::LINE_AA);
        }

        int trackX = muteBtnRect.x + muteBtnRect.width + 15;
        int trackW = w - (trackX - x) - 20;
        int trackH = 6;
        sliderTrackRect = {trackX, muteBtnRect.y + (btnSz - trackH)/2, trackW, trackH};
        cv::rectangle(frame, sliderTrackRect, cv::Scalar(25, 35, 50), cv::FILLED);

        int fillW = (isMuted ? 0 : volumeLevel) * trackW / 128;
        cv::Rect filledTrack = {trackX, sliderTrackRect.y, fillW, trackH};
        cv::rectangle(frame, filledTrack, cv::Scalar(0, 200, 255), cv::FILLED);

        int knobX = trackX + fillW, knobY = sliderTrackRect.y + trackH/2, knobR = 8;
        cv::Rect sliderHitbox = {trackX, sliderTrackRect.y - 10, trackW, trackH + 20};
        bool hovSlider = sliderHitbox.contains(hoveredRect.tl()) || isDraggingVolume;

        cv::circle(frame, {knobX, knobY}, knobR + 2, cv::Scalar(10, 15, 25), cv::FILLED, cv::LINE_AA);
        cv::circle(frame, {knobX, knobY}, knobR, hovSlider ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 229, 255), cv::FILLED, cv::LINE_AA);
    }

    void handleMouse(int event, int x, int y) {
        cv::Point pt{x, y};

        if (event == cv::EVENT_MOUSEMOVE && !isDraggingVolume) {
            hoveredRect = findHovered(x, y);
            return;
        }

        if (event == cv::EVENT_MOUSEMOVE && isDraggingVolume) {
            int relX = std::max(0, std::min(sliderTrackRect.width, x - sliderTrackRect.x));
            volumeLevel = (relX * 128) / sliderTrackRect.width;
            isMuted = (volumeLevel == 0);
            Mix_VolumeMusic(volumeLevel);
            return;
        }

        if (event == cv::EVENT_LBUTTONUP && isDraggingVolume) {
            isDraggingVolume = false;
            return;
        }

        if (event != cv::EVENT_LBUTTONDOWN) return;

        if (muteBtnRect.contains(pt)) {
            isMuted = !isMuted;
            if (isMuted) { 
                previousVolume = volumeLevel; 
                volumeLevel = 0; 
            } else { 
                volumeLevel = (previousVolume == 0) ? 64 : previousVolume; 
            }
            Mix_VolumeMusic(volumeLevel);
            return;
        }

        cv::Rect sliderHitbox = {sliderTrackRect.x, sliderTrackRect.y - 15, sliderTrackRect.width, sliderTrackRect.height + 30};
        if (sliderHitbox.contains(pt)) {
            isDraggingVolume = true;
            int relX = std::max(0, std::min(sliderTrackRect.width, x - sliderTrackRect.x));
            volumeLevel = (relX * 128) / sliderTrackRect.width;
            isMuted = (volumeLevel == 0);
            Mix_VolumeMusic(volumeLevel);
            return;
        }

        if (btnMicUp.contains({x,y})) { 
            if (micScroll > 0) micScroll--; 
            return; 
        }
        if (btnMicDown.contains({x,y})) { 
            if (micScroll + VISIBLE_ROWS < (int)mics.size()) micScroll++; 
            return; 
        }
        
        for (int i = 0; i < (int)micRows.size(); ++i) {
            if (micRows[i].contains({x,y})) {
                selMic = micScroll + i;
                selectedMicId = mics[selMic].hwId;
                if (onMicSelected) onMicSelected(selectedMicId);
                return;
            }
        }

        if (btnSpkUp.contains({x,y})) { 
            if (spkScroll > 0) spkScroll--; 
            return; 
        }
        if (btnSpkDown.contains({x,y})) { 
            if (spkScroll + VISIBLE_ROWS < (int)spks.size()) spkScroll++; 
            return; 
        }

        for (int i = 0; i < (int)spkRows.size(); ++i) {
            if (spkRows[i].contains({x,y})) {
                selSpk = spkScroll + i;
                selectedSpkId = spks[selSpk].hwId;
                if (onSpkSelected) onSpkSelected(selectedSpkId);
                return;
            }
        }

        if (btnDayUp.contains({x,y}))   { editDay++; clampDate(); return; }
        if (btnDayDown.contains({x,y})) { editDay--; clampDate(); return; }
        if (btnMonUp.contains({x,y}))   { editMon = editMon%12+1; return; }
        if (btnMonDown.contains({x,y})) { editMon = (editMon-2+12)%12+1; return; }
       
        if (btnRefresh.contains({x,y})) { refresh(); return; }

        if (btnSaveAll.contains({x,y})) {
            bool audioChanged = (selectedMicId != initialMicId || selectedSpkId != initialSpkId);
            bool dateChanged  = (editDay != initDay   || editMon != initMon || 
                                 editYear != editYear_init || editHour != initHour || 
                                 editMin != initMin);

            if (audioChanged && dateChanged) {
                saveAudioConfig();
                applyDateTime();
                setFeedback("Se guardo Audio y Fecha");
            } 
            else if (audioChanged) {
                saveAudioConfig();
                setFeedback("Solo Audio actualizado");
            } 
            else if (dateChanged) {
                applyDateTime();
                setFeedback("Solo Fecha/Hora actualizada");
            } 
            else {
                setFeedback("Nada ha cambiado");
            }

            initialMicId = selectedMicId;
            initialSpkId = selectedSpkId;
            initDay = editDay; initMon = editMon; editYear_init = editYear;
            initHour = editHour; initMin = editMin;
            return;
        }

        if (btnBack.contains({x,y})) { if (onBack) onBack(); return; }
    }
    std::function<void()> onBack;
    std::string feedbackMsg;
    std::chrono::steady_clock::time_point feedbackTime;

    void setFeedback(const std::string& msg) {
        feedbackMsg  = msg;
        feedbackTime = std::chrono::steady_clock::now();
    }
private:
    int W = 800, H = 480;
    int currentFaceIndex { 0 }; 
    std::vector<AlsaDevice> mics, spks;
    int selMic = 0, selSpk = 0;
    int micScroll = 0, spkScroll = 0;
    static constexpr int VISIBLE_ROWS = 3;

    int editDay = 1, editMon = 1, editYear = 2024;
    int editHour = 0, editMin = 0;

    std::string initialMicId, initialSpkId;
    int initDay, initMon, editYear_init, initHour, initMin;

    cv::Rect btnMicUp, btnMicDown;
    cv::Rect btnSpkUp, btnSpkDown;
    std::vector<cv::Rect> micRows, spkRows;

    cv::Rect btnDayUp,  btnDayDown;
    cv::Rect btnMonUp,  btnMonDown;
    cv::Rect btnYearUp, btnYearDown;
    cv::Rect btnHourUp, btnHourDown;
    cv::Rect btnMinUp,  btnMinDown;

    cv::Rect btnRefresh, btnSaveAll, btnBack;
    cv::Rect hoveredRect {0,0,0,0};

    void renderAudioPanel(cv::Mat& frame, int top, int panelH, bool isMic) {
        const int MARGIN = 20;
        cv::Scalar accentMic(0, 200, 255);
        cv::Scalar accentSpk(255, 180, 0);
        cv::Scalar accent = isMic ? accentMic : accentSpk;

        std::string title = isMic ? "MICROFONO" : "PARLANTE";
        cv::putText(frame, title, {MARGIN, top + 20}, cv::FONT_HERSHEY_DUPLEX, 0.55, accent, 1, cv::LINE_AA);
        cv::line(frame, {MARGIN, top + 26}, {MARGIN + 160, top + 26},
                 cv::Scalar(accent[0]*0.4, accent[1]*0.4, accent[2]*0.4), 1, cv::LINE_AA);

        int listAreaEnd = MARGIN + (W - MARGIN*2) * 60 / 100;
        int iconCX = listAreaEnd + (W - MARGIN - listAreaEnd) / 2;
        int iconCY = top + panelH / 2;
        cv::circle(frame, {iconCX, iconCY}, 38, cv::Scalar(accent[0]*0.15, accent[1]*0.15, accent[2]*0.15), cv::FILLED);
        cv::circle(frame, {iconCX, iconCY}, 38, accent, 1, cv::LINE_AA);
        if (isMic) {
            cv::rectangle(frame, {iconCX-11, iconCY-17, 22, 28}, accent, 1, cv::LINE_AA);
            cv::ellipse(frame, {iconCX, iconCY+11}, {17,11}, 0, 0, 180, accent, 1, cv::LINE_AA);
            cv::line(frame, {iconCX, iconCY+22}, {iconCX, iconCY+30}, accent, 1, cv::LINE_AA);
            cv::line(frame, {iconCX-10, iconCY+30}, {iconCX+10, iconCY+30}, accent, 1, cv::LINE_AA);
        } else {
            std::vector<cv::Point> speaker = {
                {iconCX-20,iconCY-8},{iconCX-11,iconCY-8},
                {iconCX+3, iconCY-20},{iconCX+3,iconCY+20},
                {iconCX-11,iconCY+8},{iconCX-20,iconCY+8}
            };
            cv::polylines(frame, speaker, true, accent, 1, cv::LINE_AA);
            cv::ellipse(frame, {iconCX+5,iconCY}, {11,11}, 0, -60, 60, accent, 1, cv::LINE_AA);
            cv::ellipse(frame, {iconCX+5,iconCY}, {19,19}, 0, -60, 60, accent, 1, cv::LINE_AA);
        }

        int listX  = MARGIN;
        int listW  = (W - MARGIN*2) * 60 / 100;
        int rowH   = std::min(28, (panelH - 34) / VISIBLE_ROWS);
        int listY  = top + 32;

        auto& devs    = isMic ? mics    : spks;
        auto& scroll  = isMic ? micScroll : spkScroll;
        auto& selIdx  = isMic ? selMic  : selSpk;
        auto& rowsBuf = isMic ? micRows : spkRows;
        auto& upBtn   = isMic ? btnMicUp  : btnSpkUp;
        auto& dnBtn   = isMic ? btnMicDown : btnSpkDown;

        rowsBuf.clear();

        for (int i = 0; i < VISIBLE_ROWS; ++i) {
            int devIdx = scroll + i;
            if (devIdx >= (int)devs.size()) break;

            cv::Rect row{listX, listY + i * rowH, listW, rowH - 2};
            rowsBuf.push_back(row);

            bool sel = (devIdx == selIdx);
            bool hov = row.contains(hoveredRect.tl()) || row.contains({hoveredRect.x+1, hoveredRect.y+1});

            cv::Scalar bg   = sel  ? cv::Scalar(accent[0]*0.18, accent[1]*0.18, accent[2]*0.18)
                            : hov  ? cv::Scalar(20, 28, 42)
                            :        cv::Scalar(10, 16, 28);
            cv::Scalar bord = sel  ? accent
                            : hov  ? cv::Scalar(60,70,90)
                            :        cv::Scalar(25,35,50);

            cv::rectangle(frame, row, bg, cv::FILLED);
            cv::rectangle(frame, row, bord, 1, cv::LINE_AA);

            if (sel) cv::circle(frame, {row.x + 10, row.y + row.height/2}, 4, accent, cv::FILLED, cv::LINE_AA);

            std::string label = devs[devIdx].label;
            int maxLabelW = listW - 28;  
            int bl2 = 0;
            if (cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, 0.88, 1, &bl2).width > maxLabelW) {
                while (label.size() > 3) {
                    label.pop_back();
                    std::string test = label + "..";
                    if (cv::getTextSize(test, cv::FONT_HERSHEY_PLAIN, 0.88, 1, &bl2).width <= maxLabelW) {
                        label = test;
                        break;
                    }
                }
            }

            cv::Scalar tc = sel ? cv::Scalar(255,255,255) : hov ? cv::Scalar(180,190,200) : cv::Scalar(100,120,140);
            cv::putText(frame, label, {row.x + 20, row.y + row.height - 7},
                        cv::FONT_HERSHEY_PLAIN, 0.88, tc, 1, cv::LINE_AA);
        }

        int arrowX  = listX + listW + 8;
        int midY    = listY + (VISIBLE_ROWS * rowH) / 2;
        upBtn = {arrowX, midY - 30, 26, 26};
        dnBtn = {arrowX, midY + 4,  26, 26};

        auto drawScrollBtn = [&](const cv::Rect& btn, bool up, bool canScroll) {
            cv::Scalar bc = canScroll
                ? (btn.contains(hoveredRect.tl()) ? accent : cv::Scalar(30,40,55))
                : cv::Scalar(18,22,32);
            cv::rectangle(frame, btn, bc, cv::FILLED);
            cv::rectangle(frame, btn, cv::Scalar(50,65,85), 1, cv::LINE_AA);
            cv::Scalar ac = canScroll ? (btn.contains(hoveredRect.tl()) ? cv::Scalar(255,255,255) : accent)
                                      : cv::Scalar(40,50,65);
            drawArrow(frame, {btn.x + btn.width/2, btn.y + btn.height/2}, up, ac, 5);
        };
        drawScrollBtn(upBtn, true,  scroll > 0);
        drawScrollBtn(dnBtn, false, scroll + VISIBLE_ROWS < (int)devs.size());

        cv::line(frame, {MARGIN, top + panelH - 2}, {W - MARGIN, top + panelH - 2},
                 cv::Scalar(20, 35, 55), 1, cv::LINE_AA);
    }

    void renderDatePanel(cv::Mat& frame, int top, int panelH) {
        const int MARGIN = 20;
        cv::Scalar accent(180, 255, 130);

        cv::putText(frame, "FECHA", {MARGIN, top + 20},
                    cv::FONT_HERSHEY_DUPLEX, 0.55, accent, 1, cv::LINE_AA);
        cv::line(frame, {MARGIN, top+26}, {MARGIN+120, top+26},
                cv::Scalar(70,110,50), 1, cv::LINE_AA);

        std::time_t t_now = std::time(nullptr);
        char sysBuf[64];
        std::strftime(sysBuf, sizeof(sysBuf), "Sistema: %d/%m/%Y", std::localtime(&t_now));
        cv::putText(frame, sysBuf, {W - 260, top + 20},
                    cv::FONT_HERSHEY_PLAIN, 0.85, cv::Scalar(60, 90, 60), 1, cv::LINE_AA);

        int* vals[2]   = { &editDay, &editMon };
        const char* labels[2] = { "DIA", "MES" };
        cv::Rect* upBtns[2] = { &btnDayUp,  &btnMonUp  };
        cv::Rect* dnBtns[2] = { &btnDayDown, &btnMonDown };

        int spinCount = 2;
        int spinW     = 140; 
        int spinH     = 46;  
        int gap       = (W - MARGIN*2 - spinW*spinCount) / (spinCount + 1);
        int spinTop   = top + 32;

        for (int s = 0; s < spinCount; ++s) {
            int sx = MARGIN + gap + s * (spinW + gap);
            cv::Rect spinRect{sx, spinTop, spinW, spinH};
            cv::rectangle(frame, spinRect, cv::Scalar(10, 18, 32), cv::FILLED);
            cv::rectangle(frame, spinRect, cv::Scalar(50, 80, 50), 1, cv::LINE_AA);

            int btnSz  = 22;
            int arrowX = sx + 2; 
            
            cv::Rect up{arrowX, spinTop + 1, btnSz, btnSz};
            cv::Rect dn{arrowX, spinTop + btnSz + 1, btnSz, btnSz};

            *upBtns[s] = up;
            *dnBtns[s] = dn;

            bool hovU = up.contains(hoveredRect.tl());
            cv::rectangle(frame, up, hovU ? cv::Scalar(45,90,45) : cv::Scalar(18,30,18), cv::FILLED);
            cv::rectangle(frame, up, cv::Scalar(60,120,60), 1, cv::LINE_AA);
            drawArrow(frame, {up.x+btnSz/2, up.y+btnSz/2}, true,
                    hovU ? cv::Scalar(255,255,255) : accent, 4);

            bool hovD = dn.contains(hoveredRect.tl());
            cv::rectangle(frame, dn, hovD ? cv::Scalar(45,90,45) : cv::Scalar(18,30,18), cv::FILLED);
            cv::rectangle(frame, dn, cv::Scalar(60,120,60), 1, cv::LINE_AA);
            drawArrow(frame, {dn.x+btnSz/2, dn.y+btnSz/2}, false,
                    hovD ? cv::Scalar(255,255,255) : accent, 4);

            int contentX = arrowX + btnSz + 4;
            int contentW = spinW - (btnSz + 6); 

            int lblBl = 0;
            cv::Size ls = cv::getTextSize(labels[s], cv::FONT_HERSHEY_PLAIN, 0.70, 1, &lblBl);
            cv::putText(frame, labels[s],
                        {contentX + (contentW - ls.width)/2, spinTop + 16}, 
                        cv::FONT_HERSHEY_PLAIN, 0.70, cv::Scalar(100,150,100), 1, cv::LINE_AA);

            char vbuf[16];
            if (s == 1) {
                const char* mnames[12] = {
                    "ENERO","FEBRERO","MARZO","ABRIL","MAYO","JUNIO",
                    "JULIO","AGOSTO","SEPT","OCTUBRE","NOV","DIC"
                };
                snprintf(vbuf, sizeof(vbuf), "%s", mnames[(*vals[s]-1)%12]);
            } else {
                snprintf(vbuf, sizeof(vbuf), "%02d", *vals[s]);
            }

            int valBl = 0;
            double valScale = (s == 1) ? 0.55 : 0.70; 
            cv::Size vs = cv::getTextSize(vbuf, cv::FONT_HERSHEY_DUPLEX, valScale, 1, &valBl);
            cv::putText(frame, vbuf,
                        {contentX + (contentW - vs.width)/2, spinTop + 38}, 
                        cv::FONT_HERSHEY_DUPLEX, valScale,
                        cv::Scalar(220,255,200), 1, cv::LINE_AA);
        }

        cv::line(frame, {MARGIN, top+panelH-2}, {W-MARGIN, top+panelH-2},
                cv::Scalar(20,35,55), 1, cv::LINE_AA);
    }

    void renderBottomBar(cv::Mat& frame, int barY) {
        const int btnH = 42, gap = 14;
        int btnW1 = 220, btnW2 = 160, btnW3 = 130;
        int totalW = btnW1 + btnW2 + btnW3 + gap*2;
        int startX = (W - totalW) / 2;

        auto drawBtn = [&](cv::Rect& r, int x, int w, const std::string& txt, const cv::Scalar& accent) {
            r = {x, barY, w, btnH};
            bool hov = r.contains(hoveredRect.tl()) || r.contains({hoveredRect.x+1, hoveredRect.y+1});
            cv::Scalar bg = hov ? cv::Scalar(accent[0]*0.25, accent[1]*0.25, accent[2]*0.25) : cv::Scalar(12, 18, 30);
            cv::rectangle(frame, r, bg, cv::FILLED);
            cv::rectangle(frame, r, hov ? accent : cv::Scalar(accent[0]*0.4, accent[1]*0.4, accent[2]*0.4), hov ? 2 : 1, cv::LINE_AA);
            drawTextInRect(frame, txt, r, cv::FONT_HERSHEY_DUPLEX, 0.46, hov ? cv::Scalar(255,255,255) : cv::Scalar(180,190,200));
        };

        int cx = startX;
        drawBtn(btnSaveAll, cx, btnW1, "GUARDAR TODO", cv::Scalar(0,230,120));
        cx += btnW1 + gap;
        drawBtn(btnRefresh, cx, btnW2, "REFRESCAR",    cv::Scalar(255,180,0));
        cx += btnW2 + gap;
        drawBtn(btnBack,    cx, btnW3, "VOLVER",       cv::Scalar(150,150,150));

        if (!feedbackMsg.empty()) {
            double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - feedbackTime).count();
            if (elapsed < 3.5) {
                double alpha = (elapsed < 3.0) ? 1.0 : (3.5 - elapsed) / 0.5;
                cv::Scalar fc(0, (int)(230*alpha), (int)(140*alpha));
                drawCenteredText(frame, feedbackMsg, W, barY - 10, cv::FONT_HERSHEY_PLAIN, 0.95, fc);
            } else {
                feedbackMsg = "";
            }
        }
    }

    cv::Rect findHovered(int x, int y) { return {x, y, 1, 1}; }

    void clampDate() {
        editMon = std::max(1, std::min(12, editMon));
        int daysInMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
        if (editYear % 400 == 0 || (editYear % 4 == 0 && editYear % 100 != 0)) daysInMonth[1] = 29;
        int maxDay = daysInMonth[editMon - 1];
        editDay = std::max(1, std::min(maxDay, editDay));
    }

    void applyDateTime() {
        clampDate();
        char buf[128];
        snprintf(buf, sizeof(buf),
                "sudo timedatectl set-time '%04d-%02d-%02d' 2>/dev/null",
                2026, editMon, editDay);
        int ret = std::system(buf);
        if (ret == 0) setFeedback("Fecha aplicada: " + std::to_string(editDay) + "/" + std::to_string(editMon) + "/2026");
        else setFeedback("Error: verifica sudo NOPASSWD para timedatectl");
    }
    void saveAudioConfig() {
        bool ok = true;

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
            FILE* f = fopen(path, "w");
            if (f) { fprintf(f, "%s\n", val.c_str()); fclose(f); }
        };
        writeFile("/tmp/yaren_mic_device.txt", selectedMicId);
        writeFile("/tmp/yaren_spk_device.txt", selectedSpkId);

        if (ok) setFeedback("Audio aplicado correctamente");
        else    setFeedback("Advertencia: algun dispositivo fallo");
    }
};

// =============================================================================
//  RadioApp — Reproductor de música con SDL2_mixer 🎵
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
        logger_ = log;
        currentPage = 0;
        RCLCPP_INFO(logger_, "🔄 Ciclo de caras: original → money → open → ready → tongue → ...");
        eyesOpen    = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/7.png",  cv::IMREAD_UNCHANGED);
        eyesClosed  = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/3.png",  cv::IMREAD_UNCHANGED);
        mouthClosed = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/13.png", cv::IMREAD_UNCHANGED);
        mouthOpen   = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/8.png",  cv::IMREAD_UNCHANGED);

        if (eyesOpen.empty() || mouthClosed.empty()) {
            RCLCPP_WARN(logger_, "Algunas imágenes base de la cara no se cargaron");
        }

        std::vector<std::string> altFacePaths = {
            pkgDir + "/faces/money.png",
            pkgDir + "/faces/open_mouth.png", 
            pkgDir + "/faces/ready.png",
            pkgDir + "/faces/tongue_out.png",
        };

        for (const auto& path : altFacePaths) {
            cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
            if (!img.empty()) {
                altFaces.push_back(img);
            }
        }
        
        songs = {
            { "aramsamsam",    "ARA RAM SAM SAM",  "Cancion infantil",   "Luli Pampín - ARAM SAM SAM 2021.mp3",    {220, 100, 255}, 3.0 },
            { "gorila",        "BAILE DE GORILA",  "Baila con Yaren",    "CantaJuego - El Baile del Gorila.mp3", {180,  60, 255}, 2.5 },
            { "barney",        "BARNEY",           "Te quiero yo",       "Intro de Barney y sus amigos.mp3",        {200, 100, 100}, 2.0 },
            { "chipichapa",    "CHOPI CHOPI",      "Ritmo y movimiento", "Chipi chipi chapa chapa dubi dubi daba daba Christell - Dubidubidu Subtitulada en español.mp3",    {50,  200, 255}, 3.5 },
            { "libresoy",      "LIBRE SOY",        "Canta con Yaren",    "Martina Stoessel_ Libre Soy - Frozen_ Una Aventura Congelada.mp3",      {255, 180,  50}, 1.8 },
            { "sasa",          "SA SA",            "Cancion divertida",  "Luli Pampín - SASA LA SERPIENTE (Official Video).mp3",          {80,  255, 180}, 2.8 },
            { "sitienesganas", "SI TIENES GANAS",  "Animate!",           "Luli Pampín - SI TÚ TIENES MUCHAS GANAS DE APLAUDIR - Official Video.mp3", {60,  180, 255}, 3.2 },
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

    void killAudio() {
        if (audioInitialized && Mix_PlayingMusic()) {
            Mix_FadeOutMusic(300); 
            SDL_Delay(350);         
            Mix_HaltMusic();
        }
        if (currentMusic) {
            Mix_FreeMusic(currentMusic);
            currentMusic = nullptr;
        }
        currentSongIndex = -1;
        std::system("for pid in $(ps aux | grep -E 'yaren_dance_radio.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done");
        std::system("ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\" > /dev/null 2>&1 &");
    }

    void reset() {
        killAudio();
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
        state = RadioState::PLAYING;
        playStart = std::chrono::steady_clock::now();
        
        std::string audioPath = pkgDir + "/audios/" + songs[idx].audioFile;
        if (!fs::exists(audioPath)) {
            state = RadioState::SELECTING;
            currentSong = -1;
            currentSongIndex = -1;
            return;
        }
        
        if (audioInitialized) {
            currentMusic = Mix_LoadMUS(audioPath.c_str());
            if (currentMusic) {
                if (Mix_FadeInMusic(currentMusic, -1, 500) == 0) {
                    std::system("python3 src/YAREN2/yaren_movements/yaren_movements/yaren_dance_radio.py &");
                    return;
                }
            }
        }
        
        std::string cmd = "mpg123 -q \"" + audioPath + "\" &";
        std::thread([cmd]() { std::system(cmd.c_str()); }).detach();
        std::system("python3 src/YAREN2/yaren_movements/yaren_movements/yaren_dance_radio.py &");
    }

    void handleMouse(int ev, int x, int y) {
        mousePos = {x, y, 1, 1};
        const int ITEMS_PER_PAGE = 4; 

        if (ev == cv::EVENT_MOUSEMOVE) {
            if (state == RadioState::SELECTING) {
                hovSong = -1;
                for (int i = 0; i < (int)cardRects.size(); ++i) {
                    if (cardRects[i].contains({x,y})) { 
                        hovSong = currentPage * ITEMS_PER_PAGE + i; 
                        break; 
                    }
                }
                hovBack = backBtn.area() > 0 && backBtn.contains({x,y});
                hoveredPrevPage = prevPageBtn.area() > 0 && prevPageBtn.contains({x,y});
                hoveredNextPage = nextPageBtn.area() > 0 && nextPageBtn.contains({x,y});
                hovStop = false; 
            } else if (state == RadioState::PLAYING) {
                hovStop = stopBtn.area() > 0 && stopBtn.contains({x,y});
                hovBack = false; 
                hovSong = -1;
                hoveredPrevPage = false;
                hoveredNextPage = false;
            }
            return;
        }

        if (ev != cv::EVENT_LBUTTONDOWN) return;

        if (state == RadioState::SELECTING) {
            if (hoveredPrevPage && prevPageBtn.area() > 0) {
                currentPage--; hovSong = -1; return;
            }
            if (hoveredNextPage && nextPageBtn.area() > 0) {
                currentPage++; hovSong = -1; return;
            }
            if (hovBack) {
                killAudio(); currentPage = 0;  
                if (onBack) onBack(); return;
            }
            for (int i = 0; i < (int)cardRects.size(); ++i) {
                if (cardRects[i].contains({x,y})) {
                    int actualIndex = currentPage * ITEMS_PER_PAGE + i;  
                    if (actualIndex < (int)songs.size()) playSong(actualIndex); 
                    return; 
                }
            }
        } else if (state == RadioState::PLAYING) {
            if (hovStop) {
                killAudio();
                state = RadioState::SELECTING;  
                currentSong = -1; currentSongIndex = -1; currentPage = 0;  
                return;
            }
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
    double faceSwitchInterval { 2.0 };
    
    bool audioInitialized { false };
    Mix_Music* currentMusic { nullptr };
    int currentSongIndex { -1 };
    
    cv::Mat eyesOpen, eyesClosed, mouthOpen, mouthClosed;
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

    void renderSelector(cv::Mat& frame) {
        int W = frame.cols, H = frame.rows;
        double t = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

        for (int y = 0; y < H; ++y) {
            float r = (float)y / H;
            frame.row(y).setTo(cv::Scalar(6 + r*8, 5 + r*5, 18 + r*12));
        }

        drawFloatingNotes(frame, t);
        drawCenteredText(frame, "YAREN  RADIO", W, 34, cv::FONT_HERSHEY_DUPLEX, 0.90, cv::Scalar(200, 60, 255), 2);
        cv::line(frame, {W/2-240, 50}, {W/2-20, 50}, cv::Scalar(120, 30, 160), 1, cv::LINE_AA);
        cv::line(frame, {W/2+20, 50}, {W/2+240, 50}, cv::Scalar(120, 30, 160), 1, cv::LINE_AA);
        drawCenteredText(frame, "Elige una cancion", W, 68, cv::FONT_HERSHEY_PLAIN, 0.90, cv::Scalar(100, 50, 130), 1);

        const int COLS = 2;
        const int N = (int)songs.size();
        const int CW = 260, CH = 140, G = 25;  
        const int SY = 85;
        const int ITEMS_PER_PAGE = COLS *2;  
        const int totalPages = (N + ITEMS_PER_PAGE - 1) / ITEMS_PER_PAGE;
        
        if (currentPage < 0) currentPage = 0;
        if (currentPage >= totalPages) currentPage = totalPages - 1;

        cardRects.clear();
        int startIdx = currentPage * ITEMS_PER_PAGE;
        int endIdx = std::min(startIdx + ITEMS_PER_PAGE, N);
        
        int cardIndex = 0;
        for (int i = startIdx; i < endIdx; ++i) {
            int row = cardIndex / COLS, col = cardIndex % COLS;
            int rowItems = std::min(COLS, (endIdx - startIdx) - row * COLS);
            int rowW = rowItems * CW + (rowItems - 1) * G;
            int rowSX = (W - rowW) / 2;
            cv::Rect r{rowSX + col*(CW+G), SY + row*(CH+G), CW, CH};
            cardRects.push_back(r);
            drawSongCard(frame, r, songs[i], hovSong == i, t);
            cardIndex++;
        }
        if (totalPages > 1) {
            int arrowSize = 45;
            int rightArrowX = W - arrowSize - 15;  
            int leftArrowX = 15;                   
            int centerY = H / 2;
            
            if (currentPage > 0) {
                prevPageBtn = {leftArrowX, centerY - (arrowSize / 2), arrowSize, arrowSize};
                drawNavArrow(frame, prevPageBtn, true, hoveredPrevPage);
            } else { prevPageBtn = {0,0,0,0}; }
            
            if (currentPage < totalPages - 1) {
                nextPageBtn = {rightArrowX, centerY - (arrowSize / 2), arrowSize, arrowSize};
                drawNavArrow(frame, nextPageBtn, false, hoveredNextPage);
            } else { nextPageBtn = {0,0,0,0}; }
        }
        backBtn = {W/2 - 65, H - 46, 130, 34};
        drawFlatButton(frame, backBtn, "VOLVER", cv::Scalar(120,120,120), hovBack);
    }

    void drawSongCard(cv::Mat& frame, const cv::Rect& r, const SongInfo& s, bool hov, double t) {
        cv::Scalar a = s.color;
        float pulse = hov ? (float)(0.85 + 0.15 * std::sin(t * 6.0)) : 1.0f;
        cv::Scalar bg = hov ? cv::Scalar(a[0]*0.12*pulse, a[1]*0.12*pulse, a[2]*0.12*pulse) : cv::Scalar(10, 12, 25);

        cv::rectangle(frame, r, bg, cv::FILLED);
        cv::rectangle(frame, r, hov ? a : cv::Scalar(a[0]*0.38, a[1]*0.38, a[2]*0.38), hov ? 2 : 1, cv::LINE_AA);
        cv::line(frame, {r.x+4, r.y+4}, {r.x+14, r.y+4}, a, 1, cv::LINE_AA);
        cv::line(frame, {r.x+4, r.y+4}, {r.x+4,  r.y+14}, a, 1, cv::LINE_AA);

        int icx = r.x + r.width/2;
        int icy = r.y + r.height/2 - 14;
        if (hov) {
            double pr = 18 + 4 * std::abs(std::sin(t * 5.0));
            cv::circle(frame, {icx, icy}, (int)pr, cv::Scalar(a[0]*0.20, a[1]*0.20, a[2]*0.20), cv::FILLED);
        }
        drawMusicNote(frame, {icx-6, icy+5}, a, hov ? 1.4f : 1.0f);
        drawMusicNote(frame, {icx+8, icy+2}, cv::Scalar(a[0]*0.7, a[1]*0.7, a[2]*0.7), hov ? 1.1f : 0.8f);

        int bl = 0;
        double tsc = (s.title.size() > 12) ? 0.40 : 0.46;
        cv::Size ls = cv::getTextSize(s.title, cv::FONT_HERSHEY_DUPLEX, tsc, 1, &bl);
        cv::putText(frame, s.title, {r.x + (r.width - ls.width)/2, r.y + r.height - 26},
                    cv::FONT_HERSHEY_DUPLEX, tsc, hov ? cv::Scalar(255,255,255) : cv::Scalar(a[0]*0.75, a[1]*0.75, a[2]*0.75), 1, cv::LINE_AA);

        cv::Size ss = cv::getTextSize(s.subtitle, cv::FONT_HERSHEY_PLAIN, 0.78, 1, &bl);
        cv::putText(frame, s.subtitle, {r.x + (r.width - ss.width)/2, r.y + r.height - 8},
                    cv::FONT_HERSHEY_PLAIN, 0.78, cv::Scalar(70, 85, 110), 1, cv::LINE_AA);
    }

    void renderNowPlaying(cv::Mat& frame) {
        if (currentSong < 0 || currentSong >= (int)songs.size()) return;
        const SongInfo& s = songs[currentSong];
        int W = frame.cols, H = frame.rows;
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - playStart).count();
        double beat = s.beatHz;

        for (int y = 0; y < H; ++y) {
            float r = (float)y / H;
            frame.row(y).setTo(cv::Scalar((uchar)(s.color[0]*0.06*(1-r)+4), (uchar)(s.color[1]*0.06*(1-r)+4), (uchar)(s.color[2]*0.06*(1-r)+10)));
        }

        drawPulseRings(frame, s, t, beat);
        drawAnimatedFace(frame, s, t, beat);
        drawSongInfoBar(frame, s, t, beat);
        drawEqualizer(frame, s, t, beat);

        stopBtn = {W/2 - 75, H - 46, 150, 34};
        drawFlatButton(frame, stopBtn, "DETENER", s.color, hovStop);

        double alpha = 0.55 + 0.45 * std::sin(t * 2.0);
        cv::Scalar nc(s.color[0]*alpha, s.color[1]*alpha, s.color[2]*alpha);
        cv::putText(frame, "MUSIC: " + s.title, {12, H - 56}, cv::FONT_HERSHEY_PLAIN, 0.85, nc, 1, cv::LINE_AA);  
    }

    void drawSongInfoBar(cv::Mat& frame, const SongInfo& s, double t, double beat) {
        int W = frame.cols;
        cv::Mat ov = frame.clone();
        cv::rectangle(ov, {0, 0, W, 62}, cv::Scalar(0, 0, 0), cv::FILLED);
        cv::addWeighted(ov, 0.65, frame, 0.35, 0, frame);

        double p = 0.5 + 0.5 * std::sin(t * beat * CV_PI);
        cv::circle(frame, {22, 22}, 7, cv::Scalar(s.color[0]*p, s.color[1]*p, s.color[2]*p), cv::FILLED, cv::LINE_AA);
        cv::putText(frame, "REPRODUCIENDO", {36, 27}, cv::FONT_HERSHEY_PLAIN, 0.82, cv::Scalar(140, 140, 155), 1, cv::LINE_AA);

        drawCenteredText(frame, s.title, W, 52, cv::FONT_HERSHEY_DUPLEX, 0.78, s.color, 2);
        drawCenteredText(frame, s.subtitle, W, 72, cv::FONT_HERSHEY_PLAIN, 0.82, cv::Scalar(90, 100, 120), 1);
        cv::line(frame, {W/2-250, 80}, {W/2+250, 80}, cv::Scalar(s.color[0]*0.3, s.color[1]*0.3, s.color[2]*0.3), 1, cv::LINE_AA);
    }

    void drawAnimatedFace(cv::Mat& frame, const SongInfo& s, double t, double beat) {
        int W = frame.cols, H = frame.rows;
        auto now = std::chrono::steady_clock::now();
        
        double elapsed = std::chrono::duration<double>(now - lastFaceSwitch).count();
        if (elapsed > faceSwitchInterval) {
            currentFaceIndex = (currentFaceIndex + 1) % (altFaces.size() + 1);  
            lastFaceSwitch = now;
        }
        
        if (currentFaceIndex == 0) {
            double sinceBlink = std::chrono::duration<double>(now - lastBlinkTime).count();
            
            if (!isBlinking && sinceBlink > (2.0 + 0.8 * std::sin(t * 0.3))) {
                isBlinking = true; 
                blinkStart = now; 
                lastBlinkTime = now;
            }
            if (isBlinking && std::chrono::duration<double>(now - blinkStart).count() > 0.13) {
                isBlinking = false;
            }

            const cv::Mat& eyesSrc  = isBlinking ? eyesClosed : eyesOpen;
            const cv::Mat& mouthSrc = mouthClosed; 

            cv::Size faceSize(800, 480);
            if (!eyesOpen.empty()) faceSize = eyesOpen.size();
            cv::Mat faceCanvas = cv::Mat::zeros(faceSize, CV_8UC3);
            overlayImg(faceCanvas, eyesSrc);
            overlayImg(faceCanvas, mouthSrc);

            double bounceY = std::sin(t * beat * CV_PI) * 10.0;
            double bounceX = std::cos(t * beat * CV_PI * 0.5) * 5.0;
            double angle   = std::sin(t * beat * CV_PI * 0.25) * 3.0;

            int fW = W * 65 / 100;
            int fH = (int)((float)fW * faceSize.height / faceSize.width);

            cv::Mat faceResized;
            cv::resize(faceCanvas, faceResized, {fW, fH}, 0, 0, cv::INTER_LINEAR);
            cv::Point2f center((float)fW/2, (float)fH/2);
            cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
            cv::Mat faceRot;
            cv::warpAffine(faceResized, faceRot, rot, {fW, fH});

            int fx = (W - fW)/2 + (int)bounceX;
            int fy = (H - fH)/2 - 10 + (int)bounceY;

            int rectW = fW + 40;
            int rectH = fH + 40;
            int rectX = fx - 20;
            int rectY = fy - 20;
            
            cv::rectangle(frame, {rectX, rectY, rectW, rectH}, cv::Scalar(0, 0, 0), cv::FILLED);
            cv::rectangle(frame, {rectX, rectY, rectW, rectH}, cv::Scalar(40, 40, 40), 2, cv::LINE_AA);

            cv::Rect dst(fx, fy, fW, fH);
            cv::Rect bounds(0, 0, W, H);
            dst &= bounds;
            if (dst.area() > 0) {
                cv::Rect src(dst.x - fx, dst.y - fy, dst.width, dst.height);
                if (src.x >= 0 && src.y >= 0 && src.x + src.width <= faceRot.cols && src.y + src.height <= faceRot.rows)
                    faceRot(src).copyTo(frame(dst));
            }

            double auraA = 0.08 * std::max(0.0, std::sin(t * beat * CV_PI));
            if (auraA > 0.01) {
                cv::Mat aov = frame.clone();
                cv::rectangle(aov, {rectX - 10, rectY - 10, rectW + 20, rectH + 20}, s.color, 3, cv::LINE_AA);
                cv::addWeighted(aov, auraA, frame, 1-auraA, 0, frame);
            }
            return;
        }
        
        int altIdx = currentFaceIndex - 1;  
        if (altIdx >= 0 && altIdx < (int)altFaces.size()) {
            float alpha = 1.0f;  
            
            double bounceY = std::sin(t * beat * CV_PI) * 8.0;
            double bounceX = std::cos(t * beat * CV_PI * 0.5) * 4.0;
            
            int fW = W * 60 / 100;
            int fH = (int)((float)fW * altFaces[altIdx].rows / altFaces[altIdx].cols);
            
            cv::Mat faceResized;
            cv::resize(altFaces[altIdx], faceResized, {fW, fH}, 0, 0, cv::INTER_LINEAR);
            
            int fx = (W - fW)/2 + (int)bounceX;
            int fy = (H - fH)/2 - 10 + (int)bounceY;
            
            int rectW = fW + 40;
            int rectH = fH + 40;
            int rectX = fx - 20;
            int rectY = fy - 20;
            
            cv::rectangle(frame, {rectX, rectY, rectW, rectH}, cv::Scalar(0, 0, 0), cv::FILLED);
            cv::rectangle(frame, {rectX, rectY, rectW, rectH}, cv::Scalar(40, 40, 40), 2, cv::LINE_AA);
            
            if (faceResized.channels() == 4) {
                for (int y = 0; y < fH && y + fy < H; ++y) {
                    for (int x = 0; x < fW && x + fx < W; ++x) {
                        cv::Vec4b& px = faceResized.at<cv::Vec4b>(y, x);
                        float a = px[3] / 255.f * alpha;
                        if (a > 0.f) {
                            cv::Vec3b& bg = frame.at<cv::Vec3b>(y + fy, x + fx);
                            for (int c = 0; c < 3; ++c) {
                                bg[c] = cv::saturate_cast<uchar>(px[c]*a + bg[c]*(1.f-a));
                            }
                        }
                    }
                }
            } else {
                faceResized.copyTo(frame(cv::Rect(fx, fy, fW, fH)));
            }
            
            double auraA = 0.12 * std::max(0.0, std::sin(t * beat * CV_PI));
            if (auraA > 0.01) {
                cv::Mat aov = frame.clone();
                cv::rectangle(aov, {rectX - 10, rectY - 10, rectW + 20, rectH + 20}, s.color, 3, cv::LINE_AA);
                cv::addWeighted(aov, auraA * alpha, frame, 1-(auraA * alpha), 0, frame);
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
        int eqY  = H - 88;
        int eqH  = 32;
        int eqX  = 30;
        int eqW  = W - 60;

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
            int bx = eqX + i * (barW + 1);
            int by = eqY - bH;

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
//  RoutinesApp — Interfaz gráfica para gestionar rutinas personales
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
    
    void launchRoutine(const std::string& scriptPath, const std::string& routineName) {
        state = RadioState::PLAYING; // Reutilizamos este estado para mostrar "Reproduciendo"
        
        std::thread([this, scriptPath, routineName]() {
            // 1. Comando de reproducción
            std::string cmd = "python3 src/YAREN2/yaren_movements/yaren_movements/" + routineName + " &";
            // 4. Regresar al menú y refrescar
            state = RadioState::SELECTING;
            refresh(); // Esto recarga la lista de rutinas
        }).detach();
    }
    void refresh() {
        routines.clear();
        std::string dirPath = "src/YAREN2/yaren_movements/yaren_movements";
        std::vector<std::string> exclude = {
            "__init__.py", "yaren_dance_radio.py", "yaren_fullmovement.py",
            "yaren_movement.py", "yaren_rutinanueva.py", "yaren_rutinanueva.py", "yaren_rutina1.py"
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
        if (state == RadioState::PLAYING) {
            cv::rectangle(frame, {0,0,frame.cols, frame.rows}, cv::Scalar(0,0,0), -1);
            drawCenteredText(frame, "REPRODUCIENDO RUTINA...", frame.cols, frame.rows/2, cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2);
            return; // No dibujar el menú mientras reproduce
        }
        int W = frame.cols, H = frame.rows;
        cv::rectangle(frame, {0,0,W,H}, cv::Scalar(20, 15, 30), cv::FILLED);
        drawCenteredText(frame, "RUTINAS PERSONALES", W, 40, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255, 150, 255), 2);
        cv::line(frame, {W/2-200, 55}, {W/2+200, 55}, cv::Scalar(150, 50, 150), 1, cv::LINE_AA);

        if (routines.empty()) {
            drawCenteredText(frame, "No existe ninguna rutina creada, creala ya.", W, H/2 - 30, cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(200,200,200), 1);
            
            continueBtn = {W/2 - 120, H/2 + 30, 240, 50};
            drawFlatButton(frame, continueBtn, "CONTINUAR", cv::Scalar(130, 80, 255), hovContinue);
            
            backBtn = {W/2 - 120, H/2 + 100, 240, 50};
            drawFlatButton(frame, backBtn, "SALIR", cv::Scalar(120,120,120), hovBack);
            newBtn = {0,0,0,0};
            cardRects.clear();
        } else {
            continueBtn = {0,0,0,0};
            const int COLS = 2;
            const int ITEMS_PER_PAGE = 4;
            const int CW = 280, CH = 120, G = 20, SY = 80;
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
                cv::Rect r{rowSX + col*(CW+G), SY + row*(CH+G), CW, CH};
                cardRects.push_back(r);
                
                bool hov = (hovCard == i);
                cv::Scalar a = cv::Scalar(190, 100, 220);
                cv::rectangle(frame, r, hov ? cv::Scalar(40,20,50) : cv::Scalar(20,10,30), cv::FILLED);
                cv::rectangle(frame, r, hov ? a : cv::Scalar(a[0]*.4, a[1]*.4, a[2]*.4), hov?2:1, cv::LINE_AA);
                
                std::string niceName = routines[i];
                if (niceName.size() > 3) niceName = niceName.substr(0, niceName.size() - 3);
                
                drawTextInRect(frame, niceName, r, cv::FONT_HERSHEY_DUPLEX, 0.6, hov?cv::Scalar(255,255,255):cv::Scalar(200,200,200), 1);
                idx++;
            }

            if (totalPages > 1) {
                int arrowSize = 45;
                prevPageBtn = {15, H/2 - arrowSize/2, arrowSize, arrowSize};
                nextPageBtn = {W - 15 - arrowSize, H/2 - arrowSize/2, arrowSize, arrowSize};
                if (currentPage > 0) drawNavArrow(frame, prevPageBtn, true, hovPrev);
                else prevPageBtn = {0,0,0,0};
                if (currentPage < totalPages - 1) drawNavArrow(frame, nextPageBtn, false, hovNext);
                else nextPageBtn = {0,0,0,0};
            } else {
                prevPageBtn = {0,0,0,0}; nextPageBtn = {0,0,0,0};
            }

            newBtn = {W/2 - 170, H - 60, 160, 42};
            drawFlatButton(frame, newBtn, "NUEVA RUTINA", cv::Scalar(130, 80, 255), hovNew);
            backBtn = {W/2 + 10, H - 60, 160, 42};
            drawFlatButton(frame, backBtn, "VOLVER", cv::Scalar(120,120,120), hovBack);
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
                    std::system("for pid in $(ps aux | grep -E 'yaren_rutinanueva' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done");
                    onLaunchSubprocess("python3 src/YAREN2/yaren_movements/yaren_movements/yaren_rutinanueva.py &");
                }
            }
            if (hovPrev && currentPage > 0) currentPage--;
            if (hovNext) currentPage++;
            if (hovCard >= 0) {
                int realIdx = currentPage * 4 + hovCard;
                if (realIdx < (int)routines.size() && onLaunchSubprocess) {
                    std::string r = routines[realIdx];
                    launchRoutine("...", routines[realIdx]); // Llama a la nueva función
                    std::system(("for pid in $(ps aux | grep -E '" + r + "' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done").c_str());
                    onLaunchSubprocess("python3 src/YAREN2/yaren_movements/yaren_movements/" + r + " &");
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

    MenuItem() = default;

    MenuItem(std::string id_, std::string label_, std::string sublabel_, cv::Scalar color_,
             std::string cmd_, std::string stopCmd_, bool hasSubMenu_, std::string subMenuKey_, std::string iconKey_)
        : id(std::move(id_)), label(std::move(label_)), sublabel(std::move(sublabel_)), color(color_), rect(0, 0, 0, 0)
        , cmd(std::move(cmd_)), stopCmd(std::move(stopCmd_)), hasSubMenu(hasSubMenu_), subMenuKey(std::move(subMenuKey_)), iconKey(std::move(iconKey_)) {}
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

        eyesOpenImg    = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/7.png",  cv::IMREAD_UNCHANGED);
        eyesClosedImg  = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/3.png",  cv::IMREAD_UNCHANGED);
        mouthClosedImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/13.png", cv::IMREAD_UNCHANGED);
        mouthOpenImg   = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/8.png",  cv::IMREAD_UNCHANGED);

        
        auto checkImg = [&](const cv::Mat& m, const std::string& name) {
            if (m.empty()) RCLCPP_WARN(get_logger(), "IMAGEN NO CARGADA: %s", name.c_str());
        };
        checkImg(eyesOpenImg, "eyes_open"); checkImg(eyesClosedImg, "eyes_closed");
        checkImg(mouthClosedImg, "mouth_closed"); checkImg(mouthOpenImg, "mouth_open");

        loadFrames(pkgDir + "/faces/transitions/blinking_frames", eyesFrames);
        loadFrames(pkgDir + "/faces/transitions/blinking_frames", mouthFrames);

        auto loadIcon = [&](const std::string& key, const std::string& file) {
            cv::Mat img = cv::imread(pkgDir + "/icons/" + file, cv::IMREAD_UNCHANGED);
            if (!img.empty()) iconMap[key] = img;
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
        loadIcon("accesorios", "filtro.png"); loadIcon("settings", "settings.png");loadIcon("piopio", "piopio.png");
        loadIcon("gallinaturuleca", "gallinaturuleca.png"); loadIcon("susanita", "susanita.png");loadIcon("vacalola", "vacalola.png");
        loadIcon("video", "video.png"); loadIcon("musica", "musica.png");loadIcon("radio", "radio.png");
        loadIcon("rutina_nueva", "rutina_nueva.png");

        ttsActive = false; isBlinking = false; hoveredItem = -1;
        hoveredBack = hoveredStop = hoveredExit = false;
        activeMode = ""; activeStopCmd = ""; running = true;
        showSettings = false; showRadio = false; showRoutines = false;
        lastBlinkTime = std::chrono::system_clock::now();

        faceOverlay = FaceOverlay::NONE; overlayMessage = ""; micCountdownSecs = 0;

        settingsMenu.onBack = [this]() { showSettings = false; };

        radioApp.init(pkgDir, this->get_logger());
        std::srand(std::time(nullptr));
        menuPlaylist = {
            "/home/roberto/robotis_ws/src/YAREN2/yaren_radio/audios/Intro1.mp3",
            "/home/roberto/robotis_ws/src/YAREN2/yaren_radio/audios/Intro2.mp3",
            "/home/roberto/robotis_ws/src/YAREN2/yaren_radio/audios/Intro3.mp3",
        };
        radioApp.onBack = [this]() { 
            showRadio = false; 
            startMenuMusic(); 
        };

        // Callbacks de la app de Rutinas Personales
        routinesApp.onBack = [this]() {
            showRoutines = false;
            startMenuMusic();
        };
        routinesApp.onLaunchSubprocess = [this](std::string cmd) {
            std::thread([this, cmd]() {
                int ret = std::system(cmd.c_str());
                if (ret != 0) {
                    RCLCPP_ERROR(get_logger(), "Fallo ejecutando rutina: %s", cmd.c_str());
                    showErrorOverlay("Fallo al ejecutar la rutina.", 3.0);
                }
            }).detach();
        };

        ttsSubscription = this->create_subscription<std_msgs::msg::Bool>(
            "/audio_playing", 10, std::bind(&VideoSynchronizer::audioPlayingCallback, this, std::placeholders::_1));

        faceScreenPublisher = this->create_publisher<sensor_msgs::msg::Image>("/face_screen", 10);
        modePublisher       = this->create_publisher<std_msgs::msg::String>("/yaren_mode", 10);
        modeSubscription_ = this->create_subscription<std_msgs::msg::String>(
            "/yaren_mode", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(navMutex);
                if (msg->data == "idle" || msg->data.empty()) {
                    if (!activeStopCmd.empty()) std::system(activeStopCmd.c_str());
                    activeMode.clear(); activeStopCmd.clear();
                } else { 
                    activeMode = msg->data; 
                    if (activeMode == "yaren_chat") {
                        first_listen_done = false;
                    }
                }
            });

        sttListeningSubscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/stt_listening", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && activeMode == "yaren_chat" && !first_listen_done) {
                    first_listen_done = true; 
                    
                    std::thread([this]() {
                        showCustomOverlay(FaceOverlay::MIC_PLAYING, "Yaren te escucha...", 2.0);
                    }).detach();
                }
            });

        cv::namedWindow("Yaren Face", cv::WINDOW_NORMAL);
        cv::setWindowProperty("Yaren Face", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::moveWindow("Yaren Face", 0, 0);
        cv::setMouseCallback("Yaren Face", VideoSynchronizer::mouseCallbackStatic, this);

        buildMenus();

        renderThread = std::thread(&VideoSynchronizer::renderLoop, this);
        RCLCPP_INFO(get_logger(), "face_screen listo con Radio y Rutinas Personales.");
    }

    ~VideoSynchronizer() {
        running = false;
        radioApp.killAudio(); 
        if (renderThread.joinable()) renderThread.join();
        if (testThread.joinable()) testThread.join();
        cv::destroyAllWindows();
        if (!activeStopCmd.empty()) std::system(activeStopCmd.c_str());
    }

    void drawWindow() {
        std::lock_guard<std::mutex> lock(frameMutex);
        if (!latestFrame.empty()) {
            cv::imshow("Yaren Face", latestFrame);
            int key = cv::waitKey(1);
            if (key == 27) {
                if (showSettings) { showSettings = false; return; }
                if (showRadio) { radioApp.killAudio(); showRadio = false; return; }
                if (showRoutines) { showRoutines = false; return; }

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
    bool         showSettings { false };
    
    RadioApp     radioApp;
    bool         showRadio { false };

    RoutinesApp  routinesApp;
    bool         showRoutines { false };

    cv::Rect settingsButtonRect {0,0,0,0};
    bool     hoveredSettings    {false};
    cv::Rect leftNavArrowRect  {0,0,0,0};
    cv::Rect rightNavArrowRect {0,0,0,0};
    bool hoveredLeftNav  {false};
    bool hoveredRightNav {false};
    
    cv::Rect powerButtonRect    {0,0,0,0}; 
    bool     hoveredPower       {false};   

    static MenuItem MI(const char* id, const char* label, const char* sublabel, cv::Scalar color,
                       const char* cmd, const char* stop, bool hasSub, const char* subKey, const char* iconKey) {
        return MenuItem(id, label, sublabel, color, cmd, stop, hasSub, subKey, iconKey);
    }

    std::vector<std::string> menuPlaylist;
    Mix_Music* menuMusic { nullptr };
    bool isMenuMusicPlaying { false };

    void startMenuMusic() {
        if (menuPlaylist.empty()) return;
        if (menuMusic) { Mix_FreeMusic(menuMusic); menuMusic = nullptr; }
        int idx = std::rand() % menuPlaylist.size(); 
        if (fs::exists(menuPlaylist[idx])) {
            menuMusic = Mix_LoadMUS(menuPlaylist[idx].c_str());
            if (menuMusic) {
                Mix_PlayMusic(menuMusic, -1);
                Mix_VolumeMusic(settingsMenu.volumeLevel);
                isMenuMusicPlaying = true;
            }
        }
    }

    void stopMenuMusic() {
        if (isMenuMusicPlaying) {
            Mix_FadeOutMusic(500); 
            isMenuMusicPlaying = false;
        }
    }

    void buildMenus() {
        rootMenuItems = {
            MI("modo_prueba", "MODO PRUEBA", "diagnostico y tests", {255,140,0}, "", "", true, "sub_modo_prueba", "test"),
            MI("yaren", "YAREN", "modos principales", {0,229,255}, "", "", true, "sub_yaren", "yaren"),
        };

        subMenuMap["sub_modo_prueba"] = { "MODO PRUEBA", {255,140,0}, {
            MI("test_camara", "CAMARA", "probar camara", {0,200,255}, "python src/YAREN2/CSI-Camera/simple_camera.py &", "", false, "", "camera"),
            MI("test_mic", "MICROFONO", "probar microfono", {0,255,128}, "", "", false, "", "microfono"),
            MI("test_motores", "MOTORES", "probar motores", {255,140,0}, "", "", true, "sub_motores", "motores"),
        }};

        subMenuMap["sub_motores"] = { "PROBAR MOTORES", {255,140,0}, {
            MI("motor_pos_orig", "POS. ORIG.", "posicion inicial", {255,180,50}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "yaren"),
            MI("motor_brazo_izq", "BRAZO IZQ", "brazo izquierdo", {220,100,50}, "", "", true, "sub_brazo_izq", "brazo_izq"),
            MI("motor_brazo_der", "BRAZO DER", "brazo derecho", {200,120,60}, "", "", true, "sub_brazo_der", "brazo_der"),
            MI("motor_base", "BASE", "giro de base", {180,100,80}, "", "", true, "sub_base", "girar_base"),
            MI("motor_cabeza", "CABEZA", "movimiento cabeza", {160,80,100}, "", "", true, "sub_cabeza", "girar_cabeza"),
        }};

        subMenuMap["sub_brazo_izq"] = { "BRAZO IZQUIERDO", {220,100,50}, {
            MI("brazo_izq_alto", "ARRIBA", "posicion arriba", {220,110,55}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5,-3.0, 0.0, 3.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "arriba"),
            MI("brazo_izq_med", "MEDIO", "posicion media", {180,80,40}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.5, 0.0, 3.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "medio"),
            MI("brazo_izq_bajo", "BAJO", "posicion baja", {200,90,45}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "abajo"),
        }};

        subMenuMap["sub_brazo_der"] = { "BRAZO DERECHO", {200,120,60}, {
            MI("brazo_der_alto", "ARRIBA", "posicion arriba", {200,130,65}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "arriba"),
            MI("brazo_der_med", "MEDIO", "posicion media", {160,100,50}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "medio"),
            MI("brazo_der_bajo", "BAJO", "posicion baja", {180,110,55}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "abajo"),
        }};

        subMenuMap["sub_base"] = { "BASE", {180,100,80}, {
            MI("base_izq", "IZQUIERDA", "girar izquierda", {180,110,85}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [-1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "izquierda"),
            MI("base_der", "DERECHA", "girar derecha", {170,90,75}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "derecha"),
            MI("base_orig", "POS. ORIGINAL", "posicion original", {160,80,70}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "yaren"),
        }};

        subMenuMap["sub_cabeza"] = { "CABEZA", {160,80,100}, {
            MI("cabeza_izq", "IZQUIERDA", "girar izquierda", {165,85,105}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, -0.8, 0.2, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "izquierda"),
            MI("cabeza_der", "DERECHA", "girar derecha", {150,70,90}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.8, 0.2, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "derecha"),
            MI("cabeza_orig", "POS. ORIGINAL", "posicion original", {140,60,80}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "yaren"),
        }};

        subMenuMap["sub_yaren"] = { "YAREN", {0,229,255}, {
            MI("yaren_mimic", "MIMIC", "Yaren te Imita", {0,229,255}, "ros2 launch yaren_arm_mimic yaren_mimic.launch.py &", "for pid in $(ps aux | grep -E 'yaren_mimic' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "mimic"),
            MI("yaren_chat", "CHAT", "Conversar con Yaren", {29,233,22}, "ros2 launch yaren_chat yaren_chat.launch.py &", "for pid in $(ps aux | grep -E 'yaren_chat' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "chat"),
            MI("yaren_dice", "DICE", "Jugar Yaren Dice", {64,171,255}, "ros2 launch yaren_dice yaren_dice.launch.py &", "for pid in $(ps aux | grep -E 'yaren_dice' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "dice"),
            MI("yaren_movements", "MOVEMENTS", "Yaren se mueve", {251,64,224}, "", "", true, "sub_yaren_movements", "movements"),
            MI("yaren_emotions", "EMOTIONS", "Yaren detecta tu emocion", {82,82,255}, "ros2 launch yaren_emotions yaren_emotions.launch.py &", "for pid in $(ps aux | grep -E 'yaren_emotions' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "emotions"),
            MI("yaren_filtros", "FILTROS", "Yaren te pone filtros", {105,240,174}, "", "", true, "sub_yaren_filtros", "filtros"),
        }};
        subMenuMap["sub_yaren"].key = "sub_yaren";

        subMenuMap["sub_yaren_movements"] = { "MOVEMENTS", {251,64,224}, {
            MI("yaren_rutina1", "RUTINA 1", "Rutinas", {251,64,224}, "python3 src/YAREN2/yaren_movements/yaren_movements/yaren_rutina1.py &", "for pid in $(ps aux | grep -E 'yaren_rutina1' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "rutina1"),
            MI("yaren_rutina2", "RUTINA 2", "Rutina infinita", {220,80,200}, "python3 src/YAREN2/yaren_movements/yaren_movements/yaren_fullmovement.py &", "for pid in $(ps aux | grep -E 'yaren_fullmovement' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "rutina2"),
            MI("yaren_rutinanueva", "RUTINAS PERSONALES", "gestionar y grabar", {130, 80, 255}, "INTERNAL_ROUTINES", "", false, "", "rutina_nueva"),
        }};

        subMenuMap["sub_yaren_filtros"] = { "FILTROS", {105,240,174}, {
            MI("yaren_animales", "ANIMALES", "filtro animal", {105,240,174}, "ros2 launch yaren_filters yaren_animales.launch.py &", "for pid in $(ps aux | grep -E 'yaren_animales|AnimalFaceNode|animal_filter|face_landmark|csi_cam' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "animales"),
            MI("yaren_accesorios", "ACCESORIOS", "filtro accesorios", {60,200,130}, "ros2 launch yaren_filters yaren_accesorios.launch.py &", "for pid in $(ps aux | grep -E 'yaren_accesorios|face_filter|face_landmark|csi_cam' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "accesorios"),
        }};

        subMenuMap["sub_yaren_p2"] = { "YAREN", {0,229,255}, {
            MI("yaren_radio", "YAREN RADIO", "musica y animacion", {255,80,160}, "", "", true, "sub_yaren_radio", "radio"),
        }};
        subMenuMap["sub_yaren_p2"].key = "sub_yaren_p2";

        subMenuMap["sub_yaren_radio"] = { "YAREN RADIO", {255,80,160}, {
            MI("radio_musica", "MUSICA", "reproducir musica", {255,120,200}, "INTERNAL_RADIO", "", false, "", "musica"),
            MI("radio_videos", "VIDEOS", "reproducir videos", {200,60,140}, "", "", true, "sub_yaren_videos", "video"),
        }};
        subMenuMap["sub_yaren_radio"].key = "sub_yaren_radio";  

        subMenuMap["sub_yaren_videos"] = { "VIDEOS", {200,60,140}, {
            MI("vid_pollito", "POLLITO PIO", "Canciones de la Granja", {0, 200, 255}, "python3 src/YAREN2/yaren_radio/yaren_radio/pollitopio.py &", "for pid in $(ps aux | grep -E 'pollitopio.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "piopio"),
            MI("vid_gallina", "GALLINA TURULECA", "Canciones de Yaren", {255, 150, 50}, "python3 src/YAREN2/yaren_radio/yaren_radio/gallinaturuleca.py &", "for pid in $(ps aux | grep -E 'gallinaturuleca.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "gallinaturuleca"),
            MI("vid_vaca", "LA VACA LOLA", "Canciones Infantiles", {100, 255, 100}, "python3 src/YAREN2/yaren_radio/yaren_radio/vacalola.py &", "for pid in $(ps aux | grep -E 'vacalola.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "vacalola"),
            MI("vid_susanita", "SUSANITA", "La Granja de Zenon", {255, 100, 200}, "python3 src/YAREN2/yaren_radio/yaren_radio/susanita.py &", "for pid in $(ps aux | grep -E 'susanita.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "susanita"),
        }};
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
        stopMenuMusic();
        navStack.clear();
        hoveredItem = -1;
        hoveredBack = hoveredStop = hoveredExit = false;

        if (testThread.joinable()) testThread.join();
        testThread = std::thread([this]() {
            
            std::string micId = settingsMenu.selectedMicId;
            std::string spkId = settingsMenu.selectedSpkId; 

            std::string envMic = micId.empty() ? "" : "PULSE_SOURCE='" + micId + "' ";
            std::string recordCmd = envMic + "arecord -D pulse -f S16_LE -r 44100 -c 1 -d 5 /tmp/yaren_mic_test.wav > /tmp/yaren_arecord.log 2>&1";

            RCLCPP_INFO(get_logger(), "[MIC TEST] Iniciando grabacion con comando: %s", recordCmd.c_str());

            int recRet = -1;
            std::thread recordingThread([&]() { recRet = std::system(recordCmd.c_str()); });

            for (int i = 5; i >= 1; --i) {
                {
                    std::lock_guard<std::mutex> lock(overlayMutex);
                    faceOverlay      = FaceOverlay::MIC_COUNTDOWN;
                    micCountdownSecs = i;
                    overlayMessage   = "Habla por los siguientes " + std::to_string(i) + " segundo" + (i == 1 ? "" : "s");
                }
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            recordingThread.join();

            if (recRet != 0) {
                RCLCPP_ERROR(get_logger(), "[MIC TEST] Fallo grabacion (exit %d)", recRet);
                showErrorOverlay("Fallo al grabar audio.\nRevisa el microfono.", 4.0);
                {
                    std::lock_guard<std::mutex> lock(overlayMutex);
                    faceOverlay = FaceOverlay::NONE;
                }
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
                overlayMessage = "Reproduciendo audio...";
            }

            std::string envSpk = spkId.empty() ? "" : "PULSE_SINK='" + spkId + "' ";
            std::string playCmd = envSpk + "aplay -D pulse /tmp/yaren_mic_test.wav > /tmp/yaren_aplay.log 2>&1";
            
            int playRet = std::system(playCmd.c_str());

            if (playRet != 0) {
                RCLCPP_ERROR(get_logger(), "[MIC TEST] Fallo reproduccion (exit %d)", playRet);
                showErrorOverlay("Fallo al reproducir audio.\nRevisa el parlante.", 4.0);
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
                root.title       = "MENU PRINCIPAL";
                root.accentColor = { 0, 200, 200 };
                root.items       = rootMenuItems;
                navStack.push_back(root);

                auto it = subMenuMap.find("sub_modo_prueba");
                if (it != subMenuMap.end()) {
                    navStack.push_back(it->second);
                }

                hoveredItem = -1; 
                hoveredBack = false; 
                hoveredStop = false; 
                hoveredExit = false;
                startMenuMusic();
            }

            micTestRunning = false; 
        });
    }

    void handleMouse(int event, int x, int y) {
        if (showSettings) {
            settingsMenu.handleMouse(event, x, y);
            return;
        }
        if (showRadio) {
            radioApp.handleMouse(event, x, y);
            return;
        }
        if (showRoutines) {
            routinesApp.handleMouse(event, x, y);
            return;
        }

        std::lock_guard<std::mutex> lock(navMutex);

        if (!navStack.empty()) {
            if (event == cv::EVENT_MOUSEMOVE) {
                hoveredSettings = settingsButtonRect.contains({x, y});
                hoveredPower    = powerButtonRect.contains({x, y});
                hoveredLeftNav  = (leftNavArrowRect.area()  > 0) && leftNavArrowRect.contains({x,y});
                hoveredRightNav = (rightNavArrowRect.area() > 0) && rightNavArrowRect.contains({x,y});
            }

            if (event == cv::EVENT_LBUTTONDOWN) {
                if (settingsButtonRect.contains({x, y})) {
                    showSettings = true;
                    settingsMenu.refresh();
                    return;
                }
                if (powerButtonRect.contains({x, y})) {
                    showErrorOverlay("Apagando robot...", 5.0);
                    std::system("sudo poweroff &");
                    return;
                }
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
                root.title       = "MENU PRINCIPAL";
                root.accentColor = { 0, 200, 200 };
                root.items       = rootMenuItems;
                navStack.push_back(root);
                hoveredItem = -1;
                hoveredBack = hoveredStop = hoveredExit = false;
                startMenuMusic(); 
            }
            return;
        }

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
            if (!activeStopCmd.empty()) std::system(activeStopCmd.c_str());
            activeMode = ""; activeStopCmd = "";
            navStack.clear();
            hoveredItem = -1; hoveredBack = hoveredStop = hoveredExit = false;
            auto msg = std_msgs::msg::String(); msg.data = "idle";
            modePublisher->publish(msg);

            if (prevMode.rfind("vid_", 0) == 0) {
                NavLevel root; root.title = "MENU PRINCIPAL";
                root.accentColor = {0,200,200}; root.items = rootMenuItems;
                navStack.push_back(root);
                for (const std::string& key : {"sub_yaren_p2", "sub_yaren_radio", "sub_yaren_videos"}) {
                    auto it = subMenuMap.find(key);
                    if (it != subMenuMap.end()) navStack.push_back(it->second);
                }
                startMenuMusic();
            }
            return;
        }
            for (int i = 0; i < (int)level.items.size(); ++i) {
                if (level.items[i].rect.contains({ x, y })) {
                    if (level.items[i].hasSubMenu) {
                        auto it = subMenuMap.find(level.items[i].subMenuKey);
                        if (it != subMenuMap.end()) {
                            navStack.push_back(it->second);
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

    void executeMode(MenuItem item) {
        if (item.id == "test_mic") { executeMicTest(); return; }

        if (item.cmd == "INTERNAL_RADIO") {
            showRadio = true;
            radioApp.reset();
            hoveredItem = -1;
            return; 
        }

        if (item.cmd == "INTERNAL_ROUTINES") {
            showRoutines = true;
            routinesApp.refresh();
            hoveredItem = -1;
            stopMenuMusic();
            return; 
        }

        navStack.clear();
        stopMenuMusic(); 
        hoveredItem = -1;
        hoveredBack = hoveredStop = hoveredExit = false;

        if (!activeMode.empty() && !activeStopCmd.empty()) {
            std::system(activeStopCmd.c_str());
            activeMode = ""; activeStopCmd = "";
            auto msg = std_msgs::msg::String(); msg.data = "idle";
            modePublisher->publish(msg);
        }

        std::string cleanCmd = item.cmd;
        size_t pos = cleanCmd.find_last_not_of(" \t&");
        if (pos != std::string::npos) cleanCmd = cleanCmd.substr(0, pos + 1);

        if (item.stopCmd.empty()) {
            auto msg = std_msgs::msg::String(); msg.data = item.id;
            modePublisher->publish(msg);
            if (!cleanCmd.empty()) {
                std::thread([this, cleanCmd]() {
                    int ret = std::system(cleanCmd.c_str());
                    if (ret != 0) {
                        RCLCPP_ERROR(get_logger(), "[CMD] Fallo comando (Exit %d): %s", ret, cleanCmd.c_str());
                        showErrorOverlay("No se ha podido realizar\nel comando.", 3.0);
                    }
                }).detach();
            }
            return;
        }

        activeMode    = item.id;
        activeStopCmd = item.stopCmd;

        auto msg = std_msgs::msg::String(); msg.data = activeMode;
        modePublisher->publish(msg);

        if (!cleanCmd.empty()) {
            std::thread([this, cleanCmd, prevMode = item.id]() {
                auto start = std::chrono::steady_clock::now();
                int ret = std::system(cleanCmd.c_str());
                auto end = std::chrono::steady_clock::now();
                double elapsedSecs = std::chrono::duration<double>(end - start).count();
                
                if (ret != 0 && elapsedSecs < 1.5) {
                    RCLCPP_ERROR(get_logger(), "[CMD] Fallo inmediato (Exit %d): %s", ret, cleanCmd.c_str());
                    showErrorOverlay("No se ha podido realizar\nel comando.", 4.0);
                }

                if (prevMode.rfind("vid_", 0) == 0 && activeMode == prevMode) {
                    std::lock_guard<std::mutex> lock(navMutex);
                    activeMode = ""; 
                    activeStopCmd = "";
                    
                    auto msg = std_msgs::msg::String(); 
                    msg.data = "idle";
                    modePublisher->publish(msg);

                    navStack.clear();
                    NavLevel root; 
                    root.title = "MENU PRINCIPAL";
                    root.accentColor = {0, 200, 200}; 
                    root.items = rootMenuItems;
                    navStack.push_back(root);

                    for (const std::string& key : {"sub_yaren_p2", "sub_yaren_radio", "sub_yaren_videos"}) {
                        auto it = subMenuMap.find(key);
                        if (it != subMenuMap.end()) navStack.push_back(it->second);
                    }
                    
                    hoveredItem = -1; 
                    hoveredBack = false; 
                    hoveredStop = false; 
                    hoveredExit = false;
                    
                    startMenuMusic(); 
                }
            }).detach();
        }
    }

    void renderSettingsButton(cv::Mat& frame) {
        int W = frame.cols, btnSz = 38, margin = 10;
        settingsButtonRect = {W - btnSz - margin, margin, btnSz, btnSz};

        cv::Scalar bg   = hoveredSettings ? cv::Scalar(30,50,70) : cv::Scalar(12,20,35);
        cv::Scalar bord = hoveredSettings ? cv::Scalar(0,200,255) : cv::Scalar(40,70,100);
        cv::rectangle(frame, settingsButtonRect, bg, cv::FILLED);
        cv::rectangle(frame, settingsButtonRect, bord, 1, cv::LINE_AA);

        auto iconIt = iconMap.find("settings");
        if (iconIt != iconMap.end()) {
            int pad = 6;
            int iconW = btnSz - pad*2, iconH = btnSz - pad*2;
            cv::Mat icon;
            cv::resize(iconIt->second, icon, {iconW, iconH}, 0, 0, cv::INTER_AREA);
            int ix0 = settingsButtonRect.x + pad;
            int iy0 = settingsButtonRect.y + pad;
            cv::Rect dst(ix0, iy0, iconW, iconH);
            cv::Rect bounds(0, 0, frame.cols, frame.rows);
            dst &= bounds;
            if (dst.area() > 0) {
                cv::Rect src(dst.x - ix0, dst.y - iy0, dst.width, dst.height);
                if (icon.channels() == 4) {
                    cv::Mat roi = frame(dst), iconCrop = icon(src);
                    for (int ry = 0; ry < dst.height; ++ry)
                        for (int rx = 0; rx < dst.width; ++rx) {
                            cv::Vec4b px = iconCrop.at<cv::Vec4b>(ry, rx);
                            float a = px[3] / 255.f;
                            if (a > 0.f) {
                                cv::Vec3b& bg2 = roi.at<cv::Vec3b>(ry, rx);
                                cv::Scalar tint = hoveredSettings ? cv::Scalar(0,220,255) : cv::Scalar(180,190,200);
                                for (int c = 0; c < 3; ++c)
                                    bg2[c] = cv::saturate_cast<uchar>(
                                        (px[c] * (tint[c]/255.f)) * a + bg2[c] * (1.f - a));
                            }
                        }
                } else {
                    icon(src).copyTo(frame(dst));
                }
            }
        } else {
            int cx = settingsButtonRect.x + btnSz/2, cy = settingsButtonRect.y + btnSz/2;
            cv::Scalar ic = hoveredSettings ? cv::Scalar(0,220,255) : cv::Scalar(80,130,160);
            cv::circle(frame, {cx,cy}, 8, ic, cv::FILLED, cv::LINE_AA);
        }
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
        cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_PLAIN, scale, 1, &bl);
        cv::putText(frame, label, { r.x + (r.width - ts.width) / 2, r.y + r.height - 8 },
                    cv::FONT_HERSHEY_PLAIN, scale, hovered ? cv::Scalar(255,255,255) : cv::Scalar(180,100,160), 1, cv::LINE_AA);
    }

    void renderPowerButton(cv::Mat& frame) {
        int btnSz = 38, margin = 10;
        powerButtonRect = {margin, margin, btnSz, btnSz}; 
        cv::Scalar bg   = hoveredPower ? cv::Scalar(30, 30, 180) : cv::Scalar(20, 20, 100);
        cv::Scalar bord = hoveredPower ? cv::Scalar(80, 80, 255) : cv::Scalar(50, 50, 200);

        cv::rectangle(frame, powerButtonRect, bg, cv::FILLED);
        cv::rectangle(frame, powerButtonRect, bord, 1, cv::LINE_AA);

        int cx = powerButtonRect.x + btnSz/2, cy = powerButtonRect.y + btnSz/2;
        cv::Scalar ic = hoveredPower ? cv::Scalar(255, 255, 255) : cv::Scalar(200, 200, 200);
        cv::ellipse(frame, {cx, cy}, {10, 10}, -90, 30, 330, ic, 2, cv::LINE_AA);
        cv::line(frame, {cx, cy - 10}, {cx, cy + 2}, ic, 2, cv::LINE_AA);
    }

    void renderMenu(cv::Mat& frame) {
        if (navStack.empty()) return;
        auto& level = navStack.back();
        int W = frame.cols, H = frame.rows, N = (int)level.items.size();
        bool isRoot = (navStack.size() == 1);
        leftNavArrowRect  = {0,0,0,0}; rightNavArrowRect = {0,0,0,0};

        cv::Mat ov = frame.clone();
        cv::rectangle(ov, { 0, 0, W, H }, cv::Scalar(5, 13, 26), cv::FILLED);
        cv::addWeighted(ov, 0.88, frame, 0.12, 0, frame);

        std::string title;
        if (isRoot) title = "SELECCIONA UN MODO";
        else if (navStack.size() == 2) title = navStack[1].title;
        else if (navStack.size() >= 3) title = navStack[navStack.size()-2].title + "  >  " + navStack.back().title;
        else title = navStack.back().title;

        cv::Scalar titleColor = isRoot ? cv::Scalar(0,160,200) : level.accentColor;
        drawCenteredText(frame, title, W, 28, cv::FONT_HERSHEY_DUPLEX, 0.70, titleColor, 1);

        if (!isRoot) {
            cv::line(frame, {(W-420)/2,46}, {(W+420)/2,46}, 
                     cv::Scalar(level.accentColor[0]*.4, level.accentColor[1]*.4, level.accentColor[2]*.4), 1, cv::LINE_AA);
        }

        int COLS = (N <= 2) ? 2 : 3, ROWS = (N + COLS - 1) / COLS;
        int CW = std::min((N<=2)?340:220, W/COLS-20), CH = std::min((N<=2)?200:150, (H-160)/std::max(ROWS,1)-14);
        int G = 14, TH = ROWS*CH + (ROWS-1)*G, SY = (H-TH)/2 - 10;

        for (int i = 0; i < N; ++i) {
            int row = i / COLS, col = i % COLS, rowItems = std::min(COLS, N - row*COLS);
            int rowW = rowItems*CW + (rowItems-1)*G, rowSX = (W-rowW)/2;
            level.items[i].rect = { rowSX + col*(CW+G), SY + row*(CH+G), CW, CH };
            drawCard(frame, level.items[i], hoveredItem == i, level.items[i].hasSubMenu);
        }

        const int btnH = 40, btnY = SY+TH+40, gap = 16, stopW = 300, navW = 150;
        bool hasStop = !activeMode.empty() && !activeStopCmd.empty() && 
                       activeMode != "yaren_emotions" && activeMode != "yaren_animales" && activeMode != "yaren_accesorios";
        bool hasBack = (navStack.size() > 1);
        int totalW = navW;
        if (hasBack) totalW += navW + gap;
        if (hasStop) totalW += stopW + gap;
        int cx = (W-totalW)/2;

        if (hasStop) {
            stopButtonRect = { cx, btnY, stopW, btnH };
            cv::Scalar c = hoveredStop ? cv::Scalar(40,40,220) : cv::Scalar(20,20,180);
            cv::rectangle(frame, stopButtonRect, c, cv::FILLED);
            cv::rectangle(frame, stopButtonRect, cv::Scalar(100,100,255), 1, cv::LINE_AA);
            drawTextInRect(frame, "APAGAR: "+activeMode, stopButtonRect, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255,255,255), 1);
            cx += stopW + gap;
        } else { stopButtonRect = {0,0,0,0}; }

        if (hasBack) {
            backButtonRect = { cx, btnY, navW, btnH };
            cv::Scalar c = hoveredBack ? cv::Scalar(60,60,60) : cv::Scalar(30,30,35);
            cv::rectangle(frame, backButtonRect, c, cv::FILLED);
            cv::rectangle(frame, backButtonRect, cv::Scalar(120,120,120), 1, cv::LINE_AA);
            drawTextInRect(frame, "VOLVER", backButtonRect, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(200,200,200), 1);
            cx += navW + gap;
        } else { backButtonRect = {0,0,0,0}; }

        if (!navStack.empty()) {
            const std::string& curKey = navStack.back().key;
            if (curKey == "sub_yaren") {
                rightNavArrowRect = { W - 58, H/2 - 70, 48, 130 };
                renderSideNavArrow(frame, rightNavArrowRect, false, "1/2", hoveredRightNav);
            }
            if (curKey == "sub_yaren_p2") {
                leftNavArrowRect = { 10, H/2 - 70, 48, 130 };
                renderSideNavArrow(frame, leftNavArrowRect, true, "2/2", hoveredLeftNav);
            }
        }
        exitButtonRect = { cx, btnY, navW, btnH };
        cv::Scalar ec = hoveredExit ? cv::Scalar(60,60,60) : cv::Scalar(30,30,35);
        cv::rectangle(frame, exitButtonRect, ec, cv::FILLED);
        cv::rectangle(frame, exitButtonRect, cv::Scalar(120,120,120), 1, cv::LINE_AA);
        drawTextInRect(frame, "SALIR", exitButtonRect, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(200,200,200), 1);
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
        cv::Mat panel = frame.clone();
        cv::Rect panelRect(W/8, H/3, 6*W/8, H/3);
        cv::rectangle(panel, panelRect, cv::Scalar(8, 15, 30), cv::FILLED);
        cv::addWeighted(panel, 0.82, frame, 0.18, 0, frame);

        cv::rectangle(frame, panelRect, (ov == FaceOverlay::ERROR_MSG) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 200, 100), 2, cv::LINE_AA);

        cv::Scalar dotColor = (ov == FaceOverlay::ERROR_MSG) ? cv::Scalar(0, 0, 255) : (ov == FaceOverlay::MIC_PLAYING ? cv::Scalar(0, 220, 80) : cv::Scalar(0, 229, 255));
        cv::circle(frame, {W/2, panelRect.y + 28}, 10, dotColor, cv::FILLED, cv::LINE_AA);
        cv::circle(frame, {W/2, panelRect.y + 28}, 10, dotColor * 0.5, 2, cv::LINE_AA);

        std::vector<std::string> lines; std::istringstream ss(msg); std::string line;
        while (std::getline(ss, line, '\n')) lines.push_back(line);

        double scale = 0.7; int lineH = 30, totalTH = (int)lines.size() * lineH;
        int startY = panelRect.y + (panelRect.height - totalTH) / 2 + lineH / 2;
        cv::Scalar textColor = (ov == FaceOverlay::ERROR_MSG) ? cv::Scalar(0, 0, 255) : cv::Scalar(200, 255, 220);

        for (int i = 0; i < (int)lines.size(); ++i) {
            int bl = 0; cv::Size ts = cv::getTextSize(lines[i], cv::FONT_HERSHEY_DUPLEX, scale, 1, &bl);
            cv::putText(frame, lines[i], { (W - ts.width) / 2, startY + i * lineH }, cv::FONT_HERSHEY_DUPLEX, scale, textColor, 1, cv::LINE_AA);
        }

        if (ov == FaceOverlay::MIC_COUNTDOWN) {
            int barW = panelRect.width - 40, barX = panelRect.x + 20, barY = panelRect.y + panelRect.height - 22, barH = 8;
            float pct = (float)countDown / 5.0;
            cv::rectangle(frame, {barX, barY, barW, barH}, cv::Scalar(30, 40, 60), cv::FILLED);
            cv::rectangle(frame, {barX, barY, (int)(barW * pct), barH}, cv::Scalar(0, 210, 255), cv::FILLED, cv::LINE_AA);
            cv::rectangle(frame, {barX, barY, barW, barH}, cv::Scalar(0, 100, 140), 1, cv::LINE_AA);
        }

        if (ov == FaceOverlay::MIC_PLAYING) {
            double t = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            int cx = W / 2, cyAnim = panelRect.y + panelRect.height - 20;
            for (int w = 0; w < 4; ++w) {
                double amp = 6.0 * std::abs(std::sin(t * 4.0 + w * 0.8));
                int x = cx - 30 + w * 20;
                cv::line(frame, {x, cyAnim - (int)amp}, {x, cyAnim + (int)amp}, cv::Scalar(0, 200, 80), 3, cv::LINE_AA);
            }
        }
    }

    void drawCard(cv::Mat& frame, const MenuItem& item, bool hovered, bool arrow) {
        int cx = item.rect.x, cy = item.rect.y, cw = item.rect.width, ch = item.rect.height;
        cv::Scalar a = item.color;

        cv::rectangle(frame, item.rect, hovered ? cv::Scalar(20,30,45) : cv::Scalar(10,18,31), cv::FILLED);
        cv::rectangle(frame, item.rect, hovered ? a : cv::Scalar(a[0]*.35, a[1]*.35, a[2]*.35), hovered ? 2 : 1, cv::LINE_AA);
        cv::line(frame, {cx+4, cy+4}, {cx+16, cy+4}, a, 1, cv::LINE_AA);
        cv::line(frame, {cx+4, cy+4}, {cx+4, cy+16}, a, 1, cv::LINE_AA);

        int icx = cx + cw/2, icy = cy + (ch - 50) / 2;
        auto iconIt = (!item.iconKey.empty()) ? iconMap.find(item.iconKey) : iconMap.end();
        if (iconIt != iconMap.end()) {
            const int iconSize = std::min(cw, ch) * 3 / 5;
            cv::Mat icon, original = iconIt->second;
            float aspectRatio = (float)original.cols / (float)original.rows;
            int iconW, iconH;
            if (aspectRatio >= 1.0f) { iconW = iconSize; iconH = (int)(iconSize / aspectRatio); }
            else { iconH = iconSize; iconW = (int)(iconSize * aspectRatio); }
            cv::resize(original, icon, {iconW, iconH}, 0, 0, cv::INTER_AREA);
            int ix0 = icx - iconW/2, iy0 = icy - iconH/2;
            cv::Rect dstRect(ix0, iy0, iconW, iconH), frameRect(0, 0, frame.cols, frame.rows);
            dstRect &= frameRect;
            if (dstRect.area() > 0) {
                cv::Rect srcRect(dstRect.x - ix0, dstRect.y - iy0, dstRect.width, dstRect.height);
                if (icon.channels() == 4) {
                    cv::Mat roi = frame(dstRect), iconCrop = icon(srcRect);
                    for (int ry = 0; ry < dstRect.height; ++ry)
                        for (int rx = 0; rx < dstRect.width; ++rx) {
                            cv::Vec4b px = iconCrop.at<cv::Vec4b>(ry, rx);
                            float alpha = px[3] / 255.f;
                            if (alpha > 0.f) {
                                cv::Vec3b& bg = roi.at<cv::Vec3b>(ry, rx);
                                for (int c = 0; c < 3; ++c) bg[c] = cv::saturate_cast<uchar>(px[c]*alpha + bg[c]*(1.f-alpha));
                            }
                        }
                } else { icon(srcRect).copyTo(frame(dstRect)); }
            }
        } else {
            cv::circle(frame, {icx, icy}, 22, cv::Scalar(a[0]*.3, a[1]*.3, a[2]*.3), 1, cv::LINE_AA);
            cv::circle(frame, {icx, icy}, hovered ? 9 : 6, a, cv::FILLED, cv::LINE_AA);
        }

        if (arrow) {
            std::vector<cv::Point> pts = { {cx+cw-22, icy-7}, {cx+cw-12, icy}, {cx+cw-22, icy+7} };
            cv::polylines(frame, pts, false, a, 1, cv::LINE_AA);
        }

        int bl = 0; cv::Size ls = cv::getTextSize(item.label, cv::FONT_HERSHEY_DUPLEX, 0.55, 1, &bl);
        cv::putText(frame, item.label, {cx+(cw-ls.width)/2, cy+ch-30}, cv::FONT_HERSHEY_DUPLEX, 0.55,
                    hovered ? a : cv::Scalar(a[0]*.7, a[1]*.7, a[2]*.7), 1, cv::LINE_AA);

        cv::Size ss = cv::getTextSize(item.sublabel, cv::FONT_HERSHEY_PLAIN, 0.85, 1, &bl);
        cv::putText(frame, item.sublabel, {cx+(cw-ss.width)/2, cy+ch-12}, cv::FONT_HERSHEY_PLAIN, 0.85, cv::Scalar(90,110,130), 1, cv::LINE_AA);
    }

    cv::Mat getFaceFrame() {
        auto now = std::chrono::system_clock::now();
        cv::Size canvasSize(800, 600);
        for (const cv::Mat* img : { &eyesOpenImg, &eyesClosedImg, &mouthOpenImg, &mouthClosedImg }) {
            if (!img->empty()) { canvasSize = img->size(); break; }
        }
        cv::Mat res = cv::Mat::zeros(canvasSize, CV_8UC3);

        double sinceBlink = std::chrono::duration<double>(now - lastBlinkTime).count();
        if (!isBlinking && sinceBlink > 4.0) { isBlinking = true; blinkStartTime = now; lastBlinkTime = now; }
        if (isBlinking && std::chrono::duration<double>(now - blinkStartTime).count() > 0.2) isBlinking = false;

        overlayImage(res, isBlinking ? eyesClosedImg : eyesOpenImg);
        overlayImage(res, ttsActive ? mouthOpenImg : mouthClosedImg);

        if (eyesOpenImg.empty() && eyesClosedImg.empty() && mouthOpenImg.empty() && mouthClosedImg.empty()) {
            cv::putText(res, "ADVERTENCIA: imagenes de cara no encontradas", { 20, canvasSize.height / 2 }, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 100, 255), 1, cv::LINE_AA);
        }
        return res;
    }

    void renderLoop() {
        rclcpp::Rate rate(30);
        while (running && rclcpp::ok()) {
            cv::Mat frame = getFaceFrame();

            if (showSettings) {
                settingsMenu.render(frame);
            } 
            else if (showRadio) {
                radioApp.render(frame);
            }
            else if (showRoutines) {
                routinesApp.render(frame);
            } 
            else {
                renderFaceOverlay(frame); 
                std::lock_guard<std::mutex> lock(navMutex);
                if (!navStack.empty()) {
                    renderMenu(frame);          
                    renderSettingsButton(frame); 
                    renderPowerButton(frame);    
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
        if (isMenuMusicPlaying) {
            if (ttsActive) Mix_PauseMusic();
            else Mix_ResumeMusic();
        }
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
    std::vector<cv::Mat> eyesFrames, mouthFrames;
    std::map<std::string, cv::Mat> iconMap;

    std::atomic<bool> ttsActive  { false };
    std::atomic<bool> isBlinking { false };
    std::atomic<bool> running    { false };
    std::atomic<int>  hoveredItem { -1 };
    std::atomic<bool> hoveredBack { false };
    std::atomic<bool> hoveredStop { false };
    std::atomic<bool> hoveredExit { false };

    std::string activeMode {};
    std::string activeStopCmd {};
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
    std::atomic<bool> first_listen_done { false };
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr faceScreenPublisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr   modePublisher;

    std::thread renderThread;
    std::thread testThread;
    std::mutex  frameMutex, navMutex;
    cv::Mat     latestFrame;
};

// =============================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoSynchronizer>();
    rclcpp::Rate rate(30);
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        node->drawWindow();
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}