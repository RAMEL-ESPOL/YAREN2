#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
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

static void drawCenteredText(cv::Mat& frame, const std::string& txt,
                             int totalW, int y,
                             int font, double scale,
                             const cv::Scalar& color, int thick = 1) {
    int bl = 0;
    cv::Size s = cv::getTextSize(txt, font, scale, thick, &bl);
    cv::putText(frame, txt, {(totalW - s.width) / 2, y},
                font, scale, color, thick, cv::LINE_AA);
}

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
    cv::Rect btnLang {0,0,0,0};
    std::function<void(const std::string&)> onMicSelected;
    std::function<void(const std::string&)> onSpkSelected;
    std::string selectedMicId;
    std::function<void()> onWifi;
    std::string selectedSpkId;

    bool eng() const { return isEnglish != nullptr && *isEnglish; }

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
        mics = parsePulseDevices(false);
        spks = parsePulseDevices(true);
        if (mics.empty()) mics.push_back({"", eng() ? "(no microphones detected)" : "(sin microfonos detectados)"});
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
        initHour = editHour;
        initMin = editMin;
    }

    void render(cv::Mat& frame) {
        if (frame.empty()) return;
        W = frame.cols;
        H = frame.rows;
        cv::Mat ov = frame.clone();
        cv::rectangle(ov, {0, 0, W, H}, cv::Scalar(4, 10, 22), cv::FILLED);
        cv::addWeighted(ov, 0.95, frame, 0.05, 0, frame);
        drawCenteredText(frame, eng() ? "SETTINGS" : "CONFIGURACION", W, 32,
                         cv::FONT_HERSHEY_DUPLEX, 0.80, cv::Scalar(0, 229, 255), 2);
        cv::line(frame, {W/2-320, 48}, {W/2+320, 48}, cv::Scalar(0, 80, 120), 1, cv::LINE_AA);
        int langW = 120, langH = 36;
        btnLang = {W - langW - 20, 15, langW, langH};
        bool hovLang = btnLang.contains(hoveredRect.tl());
        cv::Scalar langColor = hovLang ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 180, 200);
        cv::rectangle(frame, btnLang, cv::Scalar(20, 30, 45), cv::FILLED);
        cv::rectangle(frame, btnLang, langColor, 1, cv::LINE_AA);
        std::string langDisplay = eng() ? "EN - ESP" : "ESP -EN";
        drawTextInRect(frame, langDisplay, btnLang, cv::FONT_HERSHEY_DUPLEX, 0.5, langColor, 1);
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
        cv::putText(frame, eng() ? "MUSIC VOLUME" : "VOLUMEN DE MUSICA", {x + 15, y + 18}, cv::FONT_HERSHEY_PLAIN, 0.80, cv::Scalar(150, 160, 180), 1, cv::LINE_AA);
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

    // FIX-E: audioMutex protege todas las llamadas a SDL_mixer desde SettingsMenu
    void setAudioMutex(std::mutex* m) { audioMutex_ = m; }

    void handleMouse(int event, int x, int y) {
        cv::Point pt{x, y};
        
        if (event == cv::EVENT_MOUSEMOVE) {
            if (!isDraggingVolume) {
                hoveredRect = findHovered(x, y);
            } else {
                // AQUÍ ES DONDE VA EL BLOQUE QUE ME PASASTE
                int relX = std::max(0, std::min(sliderTrackRect.width, x - sliderTrackRect.x));
                volumeLevel = (relX * 128) / sliderTrackRect.width;
                isMuted = (volumeLevel == 0);

                // Actualizar volumen interno de SDL
                if (audioMutex_) {
                    std::lock_guard<std::mutex> lk(*audioMutex_);
                    Mix_VolumeMusic(volumeLevel);
                } else {
                    Mix_VolumeMusic(volumeLevel);
                }

                // NUEVO: Actualizar volumen maestro del sistema inmediatamente
                int volPorcentaje = (volumeLevel * 100) / 128;
                std::string volCmd = "pactl set-sink-volume @DEFAULT_SINK@ " + std::to_string(volPorcentaje) + "% &";
                std::system(volCmd.c_str());
            }
            return;
        }
        if (event == cv::EVENT_LBUTTONUP) {
            isDraggingVolume = false;
            return;
        }
        if (event != cv::EVENT_LBUTTONDOWN) return;
        if (btnWifi.contains(pt)) {
            if (onWifi) onWifi();
            return;
        }

        if (btnLang.contains(pt)) {
            if (isEnglish != nullptr) {
                *isEnglish = !(*isEnglish);
                RCLCPP_INFO(rclcpp::get_logger("SettingsMenu"), "Idioma cambiado a: %s", *isEnglish ? "English" : "Español");
            }
            if (onLanguageChanged) onLanguageChanged();
            return;
        }
        if (muteBtnRect.contains(pt)) {
            isMuted = !isMuted;
            if (isMuted) {
                previousVolume = volumeLevel;
                volumeLevel = 0;
            } else {
                volumeLevel = (previousVolume == 0) ? 64 : previousVolume;
            }
            // FIX-E: proteger Mix_VolumeMusic con mutex
            if (audioMutex_) {
                std::lock_guard<std::mutex> lk(*audioMutex_);
                Mix_VolumeMusic(volumeLevel);
            } else {
                Mix_VolumeMusic(volumeLevel);
            }
            return;
        }
        cv::Rect sliderHitbox = {sliderTrackRect.x, sliderTrackRect.y - 15, sliderTrackRect.width, sliderTrackRect.height + 30};
        if (sliderHitbox.contains(pt)) {
            isDraggingVolume = true;
            int relX = std::max(0, std::min(sliderTrackRect.width, x - sliderTrackRect.x));
            volumeLevel = (relX * 128) / sliderTrackRect.width;
            isMuted = (volumeLevel == 0);
            // FIX-E: proteger Mix_VolumeMusic con mutex
            if (audioMutex_) {
                std::lock_guard<std::mutex> lk(*audioMutex_);
                Mix_VolumeMusic(volumeLevel);
            } else {
                Mix_VolumeMusic(volumeLevel);
            }
            return;
        }
        if (btnMicUp.contains({x,y})) { if (micScroll > 0) micScroll--; return; }
        if (btnMicDown.contains({x,y})) { if (micScroll + VISIBLE_ROWS < (int)mics.size()) micScroll++; return; }
        for (int i = 0; i < (int)micRows.size(); ++i) {
            if (micRows[i].contains({x,y})) {
                selMic = micScroll + i;
                selectedMicId = mics[selMic].hwId;
                if (onMicSelected) onMicSelected(selectedMicId);
                return;
            }
        }
        if (btnSpkUp.contains({x,y})) { if (spkScroll > 0) spkScroll--; return; }
        if (btnSpkDown.contains({x,y})) { if (spkScroll + VISIBLE_ROWS < (int)spks.size()) spkScroll++; return; }
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
                                 editYear != editYear_init || editHour != initHour || editMin != initMin);
            if (audioChanged && dateChanged) {
                saveAudioConfig();
                applyDateTime();
                setFeedback(eng() ? "Audio & Date saved" : "Se guardo Audio y Fecha");
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
            initHour = editHour;
            initMin = editMin;
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
    cv::Rect btnWifi {0, 0, 0, 0};
    // FIX-E: puntero a mutex de audio compartido
    std::mutex* audioMutex_ {nullptr};

    void renderAudioPanel(cv::Mat& frame, int top, int panelH, bool isMic) {
        const int MARGIN = 20;
        cv::Scalar accentMic(0, 200, 255);
        cv::Scalar accentSpk(255, 180, 0);
        cv::Scalar accent = isMic ? accentMic : accentSpk;
        std::string title = isMic ? (eng() ? "MICROPHONE" : "MICROFONO")
                                  : (eng() ? "SPEAKER"    : "PARLANTE");
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
        cv::putText(frame, eng() ? "DATE" : "FECHA", {MARGIN, top + 20},
                    cv::FONT_HERSHEY_DUPLEX, 0.55, accent, 1, cv::LINE_AA);
        cv::line(frame, {MARGIN, top+26}, {MARGIN+120, top+26},
                 cv::Scalar(70,110,50), 1, cv::LINE_AA);
        std::time_t t_now = std::time(nullptr);
        char sysBuf[64];
        std::strftime(sysBuf, sizeof(sysBuf),
                      eng() ? "System: %d/%m/%Y" : "Sistema: %d/%m/%Y",
                      std::localtime(&t_now));
        cv::putText(frame, sysBuf, {W - 260, top + 20},
                    cv::FONT_HERSHEY_PLAIN, 0.85, cv::Scalar(60, 90, 60), 1, cv::LINE_AA);
        int* vals[2]   = { &editDay, &editMon };
        const char* labels[2] = { eng() ? "DAY" : "DIA",
                                  eng() ? "MONTH" : "MES" };
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
                const char* mnames_es[12] = {
                    "ENERO","FEBRERO","MARZO","ABRIL","MAYO","JUNIO",
                    "JULIO","AGOSTO","SEPT","OCTUBRE","NOV","DIC"
                };
                const char* mnames_en[12] = {
                    "JAN","FEB","MAR","APR","MAY","JUN",
                    "JUL","AUG","SEP","OCT","NOV","DEC"
                };
                const char* const* mnames = eng() ? mnames_en : mnames_es;
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
        const int btnH = 42, gap = 10;
        int btnW1 = 200, btnW2 = 140, btnW3 = 110, btnW4 = 110;
        int totalW = btnW1 + btnW2 + btnW3 + btnW4 + gap * 3;
        int startX = (W - totalW) / 2;
 
        auto drawBtn = [&](cv::Rect& r, int x, int w,
                           const std::string& txt, const cv::Scalar& accent) {
            r = {x, barY, w, btnH};
            bool hov = r.contains(hoveredRect.tl()) ||
                       r.contains({hoveredRect.x + 1, hoveredRect.y + 1});
            cv::Scalar bg = hov
                ? cv::Scalar(accent[0]*0.25, accent[1]*0.25, accent[2]*0.25)
                : cv::Scalar(12, 18, 30);
            cv::rectangle(frame, r, bg, cv::FILLED);
            cv::rectangle(frame, r,
                hov ? accent : cv::Scalar(accent[0]*0.4, accent[1]*0.4, accent[2]*0.4),
                hov ? 2 : 1, cv::LINE_AA);
            drawTextInRect(frame, txt, r, cv::FONT_HERSHEY_DUPLEX, 0.42,
                hov ? cv::Scalar(255, 255, 255) : cv::Scalar(180, 190, 200));
        };
 
        int cx = startX;
        drawBtn(btnSaveAll, cx,  btnW1, eng() ? "SAVE ALL"  : "GUARDAR TODO", cv::Scalar(0, 230, 120));
        cx += btnW1 + gap;
        drawBtn(btnRefresh, cx,  btnW2, eng() ? "REFRESH"   : "REFRESCAR",    cv::Scalar(255, 180, 0));
        cx += btnW2 + gap;
        // ── NUEVO: botón WiFi ──
        drawBtn(btnWifi,    cx,  btnW3, "WIFI",                                cv::Scalar(0, 180, 255));
        cx += btnW3 + gap;
        drawBtn(btnBack,    cx,  btnW4, eng() ? "BACK"      : "VOLVER",       cv::Scalar(150, 150, 150));
 
        // Feedback
        if (!feedbackMsg.empty()) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - feedbackTime).count();
            if (elapsed < 3.5) {
                double alpha = (elapsed < 3.0) ? 1.0 : (3.5 - elapsed) / 0.5;
                cv::Scalar fc(0, (int)(230 * alpha), (int)(140 * alpha));
                drawCenteredText(frame, feedbackMsg, W, barY - 10,
                    cv::FONT_HERSHEY_PLAIN, 0.95, fc);
            } else {
                feedbackMsg = "";
            }
        }
    }
 


    cv::Rect findHovered(int x, int y) { return {x, y, 1, 1}; }
    void clampDate() {
        editMon = std::max(1, std::min(12, editMon));
        // FIX-L: usar editYear para el cálculo de bisiesto
        int daysInMonth[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
        if (editYear % 400 == 0 || (editYear % 4 == 0 && editYear % 100 != 0)) daysInMonth[1] = 29;
        int maxDay = daysInMonth[editMon - 1];
        editDay = std::max(1, std::min(maxDay, editDay));
    }
    void applyDateTime() {
        clampDate();
        char buf[128];
        // FIX-H: usar editYear real en lugar de hardcodear 2026
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
        
        // 1. Aplicar volumen maestro del sistema (0-100%) basado en volumeLevel (0-128)
        int volPorcentaje = (volumeLevel * 100) / 128;
        std::string volCmd = "pactl set-sink-volume @DEFAULT_SINK@ " + std::to_string(volPorcentaje) + "% 2>/dev/null";
        std::system(volCmd.c_str());

        // 2. Configurar el sink (salida) predeterminado
        if (!selectedSpkId.empty()) {
            std::string cmd = "pactl set-default-sink '" + selectedSpkId + "' 2>/dev/null";
            if (std::system(cmd.c_str()) != 0) ok = false;
            
            // Mover las aplicaciones existentes al nuevo sink
            std::system(
                ("for i in $(pactl list sink-inputs short 2>/dev/null | awk '{print $1}'); "
                 "do pactl move-sink-input $i '" + selectedSpkId + "' 2>/dev/null; done").c_str());
        }

        // 3. Configurar el source (micrófono) predeterminado
        if (!selectedMicId.empty()) {
            std::string cmd = "pactl set-default-source '" + selectedMicId + "' 2>/dev/null";
            if (std::system(cmd.c_str()) != 0) ok = false;
            
            // Mover las aplicaciones existentes al nuevo source
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
        
        if (ok) setFeedback(eng() ? "Audio applied correctly" : "Audio aplicado correctamente");
        else    setFeedback(eng() ? "Warning: a device failed"  : "Advertencia: algun dispositivo fallo");
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
        logger_ = log;
        currentPage = 0;
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
            if (!img.empty()) altFaces.push_back(img);
        }
        songs = {
            { "aramsamsam",    "ARA RAM SAM SAM",  "Luli Pampin",   "Luli Pampín - ARAM SAM SAM 2021.mp3",    {220, 100, 255}, 3.0 },
            { "gorila",        "BAILE DE GORILA",  "CantaJuego",    "CantaJuego - El Baile del Gorila.mp3", {180,  60, 255}, 2.5 },
            { "barney",        "BARNEY ES UN DINOSAURIO", "Barney y sus amigos", "Intro de Barney y sus amigos.mp3", {200, 100, 100}, 2.0 },
            { "chipichapa",    "CHOPI CHOPI",      "Christell", "Chipi chipi chapa chapa dubi dubi daba daba Christell - Dubidubidu Subtitulada en español.mp3", {50,  200, 255}, 3.5 },
            { "libresoy",      "LIBRE SOY",        "Martina Stoessel", "Martina Stoessel_ Libre Soy - Frozen_ Una Aventura Congelada.mp3", {255, 180,  50}, 1.8 },
            { "sasa",          "SA SA",            "Luli Pampin",  "Luli Pampín - SASA LA SERPIENTE (Official Video).mp3", {80,  255, 180}, 2.8 },
            { "sitienesganas", "SI TIENES GANAS",  "Luli Pampin",  "Luli Pampín - SI TÚ TIENES MUCHAS GANAS DE APLAUDIR - Official Video.mp3", {60,  180, 255}, 3.2 },
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
            // Si estamos en una página posterior, simplemente regresamos a la página 0
            if (currentPage > 0) {
                currentPage = 0; 
                hovSong = -1;
            } else {
                // Si estamos en la página 0, entonces sí salimos de la Radio
                killAudio(); 
                currentPage = 0;
                if (onBack) onBack();
            }
            return;
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
        drawCenteredText(frame, eng() ? "Choose a song" : "Elige una cancion", W, 68, cv::FONT_HERSHEY_PLAIN, 0.90, cv::Scalar(100, 50, 130), 1);
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
        drawFlatButton(frame, backBtn, eng() ? "BACK" : "VOLVER", cv::Scalar(120,120,120), hovBack);
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
        drawFlatButton(frame, stopBtn, eng() ? "STOP" : "DETENER", s.color, hovStop);
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
        cv::putText(frame, eng() ? "NOW PLAYING" : "REPRODUCIENDO", {36, 27}, cv::FONT_HERSHEY_PLAIN, 0.82, cv::Scalar(140, 140, 155), 1, cv::LINE_AA);
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
            int rectW = fW + 40, rectH = fH + 40, rectX = fx - 20, rectY = fy - 20;
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
            int rectW = fW + 40, rectH = fH + 40, rectX = fx - 20, rectY = fy - 20;
            cv::rectangle(frame, {rectX, rectY, rectW, rectH}, cv::Scalar(0, 0, 0), cv::FILLED);
            cv::rectangle(frame, {rectX, rectY, rectW, rectH}, cv::Scalar(40, 40, 40), 2, cv::LINE_AA);
            if (faceResized.channels() == 4) {
                for (int y = 0; y < fH && y + fy < H; ++y) {
                    for (int x = 0; x < fW && x + fx < W; ++x) {
                        cv::Vec4b& px = faceResized.at<cv::Vec4b>(y, x);
                        float a = px[3] / 255.f * alpha;
                        if (a > 0.f) {
                            cv::Vec3b& bg = frame.at<cv::Vec3b>(y + fy, x + fx);
                            for (int c = 0; c < 3; ++c)
                                bg[c] = cv::saturate_cast<uchar>(px[c]*a + bg[c]*(1.f-a));
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
        if (state == RadioState::PLAYING) {
            cv::rectangle(frame, {0,0,frame.cols, frame.rows}, cv::Scalar(0,0,0), -1);
            drawCenteredText(frame, eng() ? "PLAYING ROUTINE..." : "REPRODUCIENDO RUTINA...", frame.cols, frame.rows/2, cv::FONT_HERSHEY_DUPLEX, 1.0, cv::Scalar(0,255,0), 2);
            return;
        }
        int W = frame.cols, H = frame.rows;
        cv::rectangle(frame, {0,0,W,H}, cv::Scalar(20, 15, 30), cv::FILLED);
        drawCenteredText(frame, eng() ? "PERSONAL ROUTINES" : "RUTINAS PERSONALES", W, 40, cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255, 150, 255), 2);
        cv::line(frame, {W/2-200, 55}, {W/2+200, 55}, cv::Scalar(150, 50, 150), 1, cv::LINE_AA);
        if (routines.empty()) {
            drawCenteredText(frame, eng() ? "No routine created yet, create one now." : "No existe ninguna rutina creada, creala ya.", W, H/2 - 30, cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(200,200,200), 1);
            continueBtn = {W/2 - 120, H/2 + 30, 240, 50};
            drawFlatButton(frame, continueBtn, eng() ? "CONTINUE" : "CONTINUAR", cv::Scalar(130, 80, 255), hovContinue);
            backBtn = {W/2 - 120, H/2 + 100, 240, 50};
            drawFlatButton(frame, backBtn, eng() ? "EXIT" : "SALIR", cv::Scalar(120,120,120), hovBack);
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
            drawFlatButton(frame, newBtn, eng() ? "NEW ROUTINE" : "NUEVA RUTINA", cv::Scalar(130, 80, 255), hovNew);
            backBtn = {W/2 + 10, H - 60, 160, 42};
            drawFlatButton(frame, backBtn, eng() ? "BACK" : "VOLVER", cv::Scalar(120,120,120), hovBack);
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
        if (btnShift_.contains({x,y})) { shiftActive_ = !shiftActive_; return false; }
        // Teclas normales
        int ki = findKey(x, y);
        if (ki >= 0 && ki < (int)keyRects_.size()) {
            std::string ch = getCurrentKeys()[ki];
            if (shiftActive_) {
                if (!ch.empty()) ch[0] = (char)std::toupper((unsigned char)ch[0]);
                shiftActive_ = false;
            }
            text += ch;
        }
        return false;
    }

    void render(cv::Mat& frame) {
        if (!visible) return;
        int W = frame.cols, H = frame.rows;

        // Panel semitransparente de fondo completo
        cv::Mat ov = frame.clone();
        cv::rectangle(ov, {0,0,W,H}, cv::Scalar(2,6,16), cv::FILLED);
        cv::addWeighted(ov, 0.88, frame, 0.12, 0, frame);

        // Panel del teclado
        const int KBW = 760, KBH = 310;
        const int KBX = (W - KBW) / 2;
        const int KBY = H - KBH - 10;
        cv::rectangle(frame, {KBX, KBY, KBW, KBH}, cv::Scalar(8,14,28), cv::FILLED);
        cv::rectangle(frame, {KBX, KBY, KBW, KBH}, cv::Scalar(0,150,200), 2, cv::LINE_AA);

        // Label + campo de texto
        int fieldY = KBY + 18;
        cv::putText(frame, prompt, {KBX+16, fieldY+14},
                    cv::FONT_HERSHEY_PLAIN, 0.95, cv::Scalar(80,180,220), 1, cv::LINE_AA);
        cv::Rect fieldRect{KBX+16, fieldY+20, KBW-32, 32};
        cv::rectangle(frame, fieldRect, cv::Scalar(4,12,26), cv::FILLED);
        cv::rectangle(frame, fieldRect, cv::Scalar(0,180,230), 1, cv::LINE_AA);
        // Mostrar texto con cursor parpadeante
        std::string display = text;
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (std::fmod(t * 2.0, 1.0) < 0.5) display += "|";
        // Mostrar asteriscos si es Clave
        std::string shown = isPassword_ ? std::string(text.size(), '*') + (display.size()>text.size()?"|":"") : display;
        cv::putText(frame, shown, {fieldRect.x+8, fieldRect.y+22},
                    cv::FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(220,235,255), 1, cv::LINE_AA);

        // Dibujar teclas
        auto& keys = getCurrentKeys();
        keyRects_.clear();
        const int ROWS_N = 4;
        // Layout QWERTY
        const std::vector<std::vector<std::string>> ROWS_ALPHA = {
            {"q","w","e","r","t","y","u","i","o","p"},
            {"a","s","d","f","g","h","j","k","l"},
            {"z","x","c","v","b","n","m"},
            {}  // fila de controles
        };
        const std::vector<std::vector<std::string>> ROWS_NUM = {
            {"1","2","3","4","5","6","7","8","9","0"},
            {"-","/",":",";","(",")","$","&","@","\""},
            {".","_","#","!","?","=","+","<",">"},
            {}
        };
        auto& ROWS = showNumbers_ ? ROWS_NUM : ROWS_ALPHA;

        const int keyH = 44, keyGap = 5;
        int rowY = KBY + 75;

        for (int r = 0; r < 3; ++r) {
            const auto& row = ROWS[r];
            int n = (int)row.size();
            int keyW = (KBW - keyGap*(n+1)) / n;
            int rowX = KBX + (KBW - (keyW*n + keyGap*(n-1))) / 2;
            for (int c = 0; c < n; ++c) {
                int kx = rowX + c*(keyW+keyGap);
                cv::Rect kr{kx, rowY, keyW, keyH};
                keyRects_.push_back(kr);
                bool hov = ((int)keyRects_.size()-1 == hovKey_);
                cv::Scalar bg   = hov ? cv::Scalar(30,80,120) : cv::Scalar(14,24,42);
                cv::Scalar bord = hov ? cv::Scalar(0,220,255) : cv::Scalar(30,60,90);
                cv::rectangle(frame, kr, bg, cv::FILLED);
                cv::rectangle(frame, kr, bord, 1, cv::LINE_AA);
                std::string label = row[c];
                if (!showNumbers_ && shiftActive_ && !label.empty())
                    label[0] = (char)std::toupper((unsigned char)label[0]);
                int bl=0; cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_DUPLEX, 0.55, 1, &bl);
                cv::putText(frame, label, {kx+(keyW-ts.width)/2, rowY+keyH/2+7},
                            cv::FONT_HERSHEY_DUPLEX, 0.55, hov?cv::Scalar(255,255,255):cv::Scalar(180,200,220),
                            1, cv::LINE_AA);
            }
            rowY += keyH + keyGap;
        }

        // Fila de controles
        int ctrlY  = rowY;
        int ctrlH  = keyH;

        // Shift / 123
        btnNumToggle_ = {KBX+keyGap, ctrlY, 90, ctrlH};
        bool hNum = btnNumToggle_.contains(hovPt_);
        cv::rectangle(frame, btnNumToggle_, hNum?cv::Scalar(30,50,80):cv::Scalar(10,20,35), cv::FILLED);
        cv::rectangle(frame, btnNumToggle_, cv::Scalar(40,80,120), 1, cv::LINE_AA);
        cv::putText(frame, showNumbers_?"ABC":"123", {btnNumToggle_.x+18, ctrlY+ctrlH/2+7},
                    cv::FONT_HERSHEY_DUPLEX, 0.55, cv::Scalar(160,200,230), 1, cv::LINE_AA);

        // Shift (solo en modo alpha)
        if (!showNumbers_) {
            btnShift_ = {KBX+keyGap+95, ctrlY, 70, ctrlH};
            bool hSh = btnShift_.contains(hovPt_);
            cv::Scalar shBord = shiftActive_ ? cv::Scalar(0,220,120) : cv::Scalar(40,80,120);
            cv::rectangle(frame, btnShift_, hSh?cv::Scalar(20,50,30):cv::Scalar(10,20,35), cv::FILLED);
            cv::rectangle(frame, btnShift_, shBord, 1, cv::LINE_AA);
            cv::putText(frame, "SHIFT", {btnShift_.x+6, ctrlY+ctrlH/2+7},
                        cv::FONT_HERSHEY_DUPLEX, 0.42, shiftActive_?cv::Scalar(0,230,120):cv::Scalar(140,180,200),
                        1, cv::LINE_AA);
        } else {
            btnShift_ = {0,0,0,0};
        }

        // Espacio
        int spX = KBX + (showNumbers_ ? 100+keyGap*2 : 175+keyGap*2);
        int spW = KBW - spX + KBX - 100 - keyGap*3 - 90;
        btnSpace_ = {spX, ctrlY, spW, ctrlH};
        bool hSp = btnSpace_.contains(hovPt_);
        cv::rectangle(frame, btnSpace_, hSp?cv::Scalar(25,50,80):cv::Scalar(12,20,38), cv::FILLED);
        cv::rectangle(frame, btnSpace_, cv::Scalar(30,70,110), 1, cv::LINE_AA);
        cv::putText(frame, "ESPACIO", {spX+spW/2-36, ctrlY+ctrlH/2+7},
                    cv::FONT_HERSHEY_DUPLEX, 0.46, cv::Scalar(120,160,190), 1, cv::LINE_AA);

        // Backspace
        btnBackspace_ = {KBX+KBW-keyGap-90, ctrlY, 90, ctrlH};
        bool hBs = btnBackspace_.contains(hovPt_);
        cv::rectangle(frame, btnBackspace_, hBs?cv::Scalar(60,20,20):cv::Scalar(25,10,10), cv::FILLED);
        cv::rectangle(frame, btnBackspace_, cv::Scalar(140,40,40), 1, cv::LINE_AA);
        cv::putText(frame, "<-", {btnBackspace_.x+20, ctrlY+ctrlH/2+7},
                    cv::FONT_HERSHEY_DUPLEX, 0.55, cv::Scalar(220,100,100), 1, cv::LINE_AA);

        // Botones OK y CANCELAR
        int btnRow2Y = ctrlY + ctrlH + keyGap;
        btnOk_ = {KBX+KBW-keyGap-200, btnRow2Y, 200, ctrlH-6};
        bool hOk = btnOk_.contains(hovPt_);
        cv::rectangle(frame, btnOk_, hOk?cv::Scalar(0,60,20):cv::Scalar(0,30,10), cv::FILLED);
        cv::rectangle(frame, btnOk_, cv::Scalar(0,180,80), hOk?2:1, cv::LINE_AA);
        cv::putText(frame, "CONECTAR", {btnOk_.x+28, btnRow2Y+ctrlH/2+1},
                    cv::FONT_HERSHEY_DUPLEX, 0.55, cv::Scalar(0,230,100), 1, cv::LINE_AA);

        btnCancel_ = {KBX+keyGap, btnRow2Y, 130, ctrlH-6};
        bool hCan = btnCancel_.contains(hovPt_);
        cv::rectangle(frame, btnCancel_, hCan?cv::Scalar(40,30,10):cv::Scalar(18,14,6), cv::FILLED);
        cv::rectangle(frame, btnCancel_, cv::Scalar(140,100,30), hCan?2:1, cv::LINE_AA);
        cv::putText(frame, "CANCELAR", {btnCancel_.x+4, btnRow2Y+ctrlH/2+1},
                    cv::FONT_HERSHEY_DUPLEX, 0.42, cv::Scalar(200,160,60), 1, cv::LINE_AA);
    }

    bool isPassword_ { true };  // mostrar *** en el campo

private:
    bool showNumbers_ { false };
    bool shiftActive_ { false };
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

        if (btnScrollUp_.contains({x,y})  && btnScrollUp_.area()>0)  { if(scroll_>0) scroll_--; return; }
        if (btnScrollDown_.contains({x,y})&& btnScrollDown_.area()>0){ scroll_++; return; }
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

    void renderMain(cv::Mat& frame) {
        int W = frame.cols, H = frame.rows;
        cv::Mat ov = frame.clone();
        cv::rectangle(ov,{0,0,W,H},cv::Scalar(4,10,22),cv::FILLED);
        cv::addWeighted(ov,0.96,frame,0.04,0,frame);

        drawCenteredText(frame, eng()?"WIFI SETUP":"CONFIGURAR WIFI",
                         W,32,cv::FONT_HERSHEY_DUPLEX,0.80,cv::Scalar(0,229,255),2);
        cv::line(frame,{W/2-300,48},{W/2+300,48},cv::Scalar(0,80,120),1,cv::LINE_AA);

        int legY = 65; 
        int legX = W/2 - 95;
        cv::circle(frame,{legX,legY},5,cv::Scalar(0,200,100),cv::FILLED,cv::LINE_AA);
        cv::putText(frame,eng()?"saved  ":"guardada  ",{legX+12,legY+5},cv::FONT_HERSHEY_PLAIN,0.80,cv::Scalar(60,140,80),1,cv::LINE_AA);
        cv::circle(frame,{legX+110,legY},5,cv::Scalar(80,80,80),1,cv::LINE_AA);
        cv::putText(frame,eng()?"new network":"nueva",{legX+122,legY+5},cv::FONT_HERSHEY_PLAIN,0.80,cv::Scalar(80,110,140),1,cv::LINE_AA);
        
        const int LIST_X=W/2-270, LIST_W=540, ROW_H=42, LIST_Y=82, VISIBLE=7;
        
        std::vector<WifiNetwork> safe_networks;
        {
            std::lock_guard<std::mutex> lock(networks_mutex_);
            safe_networks = networks_;
        }

        bool refreshing = is_refreshing_.load();

        if (refreshing) {
            double t = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
            int t_dots = (int)(t * 3.0) % 4;
            std::string loadingText = (eng() ? "Scanning networks" : "Buscando redes") + std::string(t_dots, '.');
            drawCenteredText(frame, loadingText, W, LIST_Y + 100, cv::FONT_HERSHEY_DUPLEX, 0.75, cv::Scalar(0, 200, 255), 1);
        } else {
            int total = (int)safe_networks.size();
            if (scroll_ > std::max(0,total-VISIBLE)) scroll_=std::max(0,total-VISIBLE);

            rowRects_.clear();
            for (int i=0;i<VISIBLE;++i) {
                int idx=scroll_+i;
                if (idx>=total) break;
                cv::Rect r{LIST_X,LIST_Y+i*ROW_H,LIST_W,ROW_H-3};
                rowRects_.push_back({r,idx});
                bool sel=(selectedIdx_==idx);
                bool hov=r.contains(hovPt_);
                cv::Scalar bg  =sel?cv::Scalar(0,40,70):hov?cv::Scalar(15,25,40):cv::Scalar(8,16,28);
                cv::Scalar brd =sel?cv::Scalar(0,229,255):hov?cv::Scalar(40,80,100):cv::Scalar(20,40,55);
                cv::rectangle(frame,r,bg,cv::FILLED);
                cv::rectangle(frame,r,brd,sel?2:1,cv::LINE_AA);

                if (safe_networks[idx].saved)
                    cv::circle(frame,{r.x+14,r.y+r.height/2},5,cv::Scalar(0,200,100),cv::FILLED,cv::LINE_AA);
                else
                    cv::circle(frame,{r.x+14,r.y+r.height/2},5,cv::Scalar(80,80,80),1,cv::LINE_AA);

                if (safe_networks[idx].secured) {
                    int lx=r.x+28, ly=r.y+r.height/2;
                    cv::rectangle(frame,{lx-4,ly-1,8,7},cv::Scalar(160,160,160),1,cv::LINE_AA);
                    cv::ellipse(frame,{lx,ly-1},{4,4},0,180,360,cv::Scalar(160,160,160),1,cv::LINE_AA);
                }

                cv::Scalar tc=sel?cv::Scalar(255,255,255):hov?cv::Scalar(180,200,210):cv::Scalar(120,150,170);
                cv::putText(frame,safe_networks[idx].ssid,{r.x+42,r.y+r.height/2+6},
                            cv::FONT_HERSHEY_PLAIN,1.0,tc,1,cv::LINE_AA);

                std::string badge = safe_networks[idx].saved ? (eng()?"saved":"guardada")
                                                             : (eng()?"new":"nueva");
                cv::Scalar bcolor = safe_networks[idx].saved ? cv::Scalar(0,150,80) : cv::Scalar(0,140,200);
                int bl = 0;
                cv::Size bs = cv::getTextSize(badge, cv::FONT_HERSHEY_PLAIN, 0.72, 1, &bl);
                cv::putText(frame, badge,
                            {r.x + r.width - bs.width - 10, r.y + r.height/2 + 5},
                            cv::FONT_HERSHEY_PLAIN, 0.72, bcolor, 1, cv::LINE_AA);
            }

            if (scroll_>0) {
                btnScrollUp_={LIST_X+LIST_W+10,LIST_Y,30,30};
                bool hu=btnScrollUp_.contains(hovPt_);
                cv::rectangle(frame,btnScrollUp_,hu?cv::Scalar(30,50,70):cv::Scalar(15,25,40),cv::FILLED);
                cv::rectangle(frame,btnScrollUp_,cv::Scalar(40,80,100),1,cv::LINE_AA);
                std::vector<cv::Point> tri={{btnScrollUp_.x+15,btnScrollUp_.y+5},{btnScrollUp_.x+5,btnScrollUp_.y+22},{btnScrollUp_.x+25,btnScrollUp_.y+22}};
                cv::fillPoly(frame,tri,hu?cv::Scalar(255,255,255):cv::Scalar(0,180,220));
            } else { btnScrollUp_={0,0,0,0}; }

            if (scroll_+VISIBLE<total) {
                btnScrollDown_={LIST_X+LIST_W+10,LIST_Y+VISIBLE*ROW_H-30,30,30};
                bool hd=btnScrollDown_.contains(hovPt_);
                cv::rectangle(frame,btnScrollDown_,hd?cv::Scalar(30,50,70):cv::Scalar(15,25,40),cv::FILLED);
                cv::rectangle(frame,btnScrollDown_,cv::Scalar(40,80,100),1,cv::LINE_AA);
                std::vector<cv::Point> tri={{btnScrollDown_.x+15,btnScrollDown_.y+25},{btnScrollDown_.x+5,btnScrollDown_.y+8},{btnScrollDown_.x+25,btnScrollDown_.y+8}};
                cv::fillPoly(frame,tri,hd?cv::Scalar(255,255,255):cv::Scalar(0,180,220));
            } else { btnScrollDown_={0,0,0,0}; }
        }

        int btnY=LIST_Y+VISIBLE*ROW_H+18, btnH=44;

        btnConnect_={W/2-280,btnY,220,btnH};
        bool canC=(selectedIdx_>=0)&&!connecting_&&!keyboard_.visible&&!refreshing;
        bool hc=btnConnect_.contains(hovPt_)&&canC;
        cv::Scalar cBg=connecting_?cv::Scalar(20,50,20):canC?(hc?cv::Scalar(0,60,20):cv::Scalar(0,35,12)):cv::Scalar(15,20,18);
        cv::Scalar cBrd=connecting_?cv::Scalar(0,150,80):canC?cv::Scalar(0,200,80):cv::Scalar(30,50,35);
        cv::rectangle(frame,btnConnect_,cBg,cv::FILLED);
        cv::rectangle(frame,btnConnect_,cBrd,hc?2:1,cv::LINE_AA);
        std::string cLabel=connecting_?(eng()?"Connecting...":"Conectando..."):(eng()?"CONNECT":"CONECTAR");
        drawTextInRect(frame,cLabel,btnConnect_,cv::FONT_HERSHEY_DUPLEX,0.46,canC?cv::Scalar(255,255,255):cv::Scalar(80,100,80));

        btnRefresh_={W/2-36,btnY,160,btnH};
        bool hr=btnRefresh_.contains(hovPt_) && !refreshing;
        cv::Scalar rBg=refreshing?cv::Scalar(10,10,10):hr?cv::Scalar(30,40,10):cv::Scalar(15,22,8);
        cv::Scalar rBrd=refreshing?cv::Scalar(50,50,50):hr?cv::Scalar(200,200,0):cv::Scalar(120,120,0);
        cv::rectangle(frame,btnRefresh_,rBg,cv::FILLED);
        cv::rectangle(frame,btnRefresh_,rBrd,hr?2:1,cv::LINE_AA);
        drawTextInRect(frame,eng()?"REFRESH":"REFRESCAR",btnRefresh_,cv::FONT_HERSHEY_DUPLEX,0.46,!refreshing?cv::Scalar(255,255,255):cv::Scalar(100,100,100));

        btnSkip_={W/2+144,btnY,120,btnH};
        bool hs=btnSkip_.contains(hovPt_);
        cv::rectangle(frame,btnSkip_,hs?cv::Scalar(35,35,35):cv::Scalar(18,18,22),cv::FILLED);
        cv::rectangle(frame,btnSkip_,hs?cv::Scalar(150,150,150):cv::Scalar(80,80,80),hs?2:1,cv::LINE_AA);
        drawTextInRect(frame,eng()?"SKIP":"OMITIR",btnSkip_,cv::FONT_HERSHEY_DUPLEX,0.46,hs?cv::Scalar(255,255,255):cv::Scalar(160,160,160));

        if (!statusMsg_.empty()) {
            bool isErr=statusMsg_.find("Error")!=std::string::npos||
                       statusMsg_.find("fallo")!=std::string::npos||
                       statusMsg_.find("Failed")!=std::string::npos||
                       statusMsg_.find("Wrong")!=std::string::npos||
                       statusMsg_.find("incorrecta")!=std::string::npos||
                       statusMsg_.find("SIN INTERNET")!=std::string::npos||
                       statusMsg_.find("no internet")!=std::string::npos;
            cv::Scalar sc=isErr?cv::Scalar(0,0,220):cv::Scalar(0,200,80);
            int bl2=0; cv::Size ms=cv::getTextSize(statusMsg_,cv::FONT_HERSHEY_PLAIN,0.95,1,&bl2);
            cv::putText(frame,statusMsg_,{(W-ms.width)/2, H - 25},cv::FONT_HERSHEY_PLAIN,0.95,sc,1,cv::LINE_AA);
        }
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
        eyesOpenImg    = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/7.png",  cv::IMREAD_UNCHANGED);
        eyesClosedImg  = cv::imread(pkgDir + "/faces/separate_parts_without_background/eyes_pairs/3.png",  cv::IMREAD_UNCHANGED);
        mouthClosedImg = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/13.png", cv::IMREAD_UNCHANGED);
        mouthOpenImg   = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/8.png",  cv::IMREAD_UNCHANGED);
        // Lip sync sprites
        for (int i = 0; i <= 8; ++i) {
            cv::Mat m = cv::imread(pkgDir + "/faces/separate_parts_without_background/mouths/" + std::to_string(i) + ".png", cv::IMREAD_UNCHANGED);
            mouthSprites_[i] = m;
        }
        lastInteractionTime = std::chrono::system_clock::now();
        auto checkImg = [&](const cv::Mat& m, const std::string& name) {
            if (m.empty()) RCLCPP_WARN(get_logger(), "IMAGEN NO CARGADA: %s", name.c_str());
        };
        checkImg(eyesOpenImg, "eyes_open"); checkImg(eyesClosedImg, "eyes_closed");
        checkImg(mouthClosedImg, "mouth_closed"); checkImg(mouthOpenImg, "mouth_open");
        yarenSplashImg = cv::imread(pkgDir + "/icons/yaren.png", cv::IMREAD_UNCHANGED);
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
        loadIcon("accesorios", "filtro.png"); loadIcon("settings", "settings.png"); loadIcon("piopio", "piopio.png");
        loadIcon("gallinaturuleca", "gallinaturuleca.png"); loadIcon("susanita", "susanita.png"); loadIcon("vacalola", "vacalola.png");
        loadIcon("video", "video.png"); loadIcon("musica", "musica.png"); loadIcon("radio", "radio.png");
        loadIcon("rutina_nueva", "rutina_nueva.png");
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
            if (fs::exists(bootTrack)) {
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
                        if (activeMode == "yaren_dice_con_ayuda") {
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
                                isEnglish ? "Your turn Speak now" : "Tu turno, Habla ahora",
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
                root.title = isEnglish ? "MAIN MENU" : "MENU PRINCIPAL";
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
                navStack[0].title = isEnglish ? "MAIN MENU" : "MENU PRINCIPAL";
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

        std::system("for pid in $(ps aux | grep -E 'wake_word_node|yaren_voice_menu|gestor_idioma|yaren_chat|lifecycle_node|yaren_emotions|yaren_radio|yaren_filters|yaren_dice|yaren_mimic|csi_cam_pub|fondo_virtual' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done");
        const char* home = std::getenv("HOME");
        if (home) {
            std::string python  = std::string(home) + "/robotis_ws/venv_yaren/bin/python3";
            std::string ws      = std::string(home) + "/robotis_ws";
            std::string setup   = ws + "/install/setup.bash";
            std::string venv    = ws + "/venv_yaren/bin/activate";

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
            std::string cmd_chistes = "bash -c 'source " + setup + " && source " + venv + " && ros2 run yaren_chistes chistes_node' &";
            std::string cmd_memoria = "bash -c 'source " + setup + " && ros2 run yaren_juegos memoria_node' &";
            std::string cmd_dance = "bash -c 'source " + setup + " && ros2 run yaren_juegos dance_game_node' &"; // NUEVO

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
            std::system(cmd_chistes.c_str());


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
                            ? "No internet — configure WiFi..."
                            : "Sin internet — configura el WiFi...";
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
                configure_node("tts_lifecycle_node",        "Sintesis de Voz (Cloud)");
                configure_node("stt_local_lifecycle_node",        "Reconocimiento de Voz (Local)");
                configure_node("tts_local_lifecycle_node",        "Sintesis de Voz (Local)");
                configure_node("detector",                  "Deteccion de Emociones");
                configure_node("face_filter_node",          "Filtros de Cara");
                configure_node("body_points_detector_node", "Deteccion Corporal");
                configure_node("body_points_detector_node_visual", "Deteccion Corporal con Ayuda");
                configure_node("yaren_speaker_node",        "Altavoz");
                configure_node("virtual_background_node",   "Fondo Virtual");
                configure_node("filtro_animales",           "Filtro de Animales");
                configure_node("face_landmark_publisher",   "Landmarks Faciales");
                configure_node("csi_cam_node",              "Camara Principal");
                configure_node("chistes_node", "Modulo de Chistes");
                configure_node("memoria_node",  "Juego de Memoria");
                configure_node("dance_game_node", "Juego de Baile");
 
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
        std::system("for pid in $(ps aux | grep -E 'wake_word_node|yaren_voice_menu|gestor_idioma|yaren_chat|lifecycle_node|yaren_emotions|yaren_radio|yaren_filters|yaren_dice|yaren_mimic' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done");
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
            MI("modo_prueba", isEnglish ? "TEST MODE" : "MODO PRUEBA", isEnglish ? "diagnostics and tests" : "diagnostico y tests", {255,140,0}, "", "", true, "sub_modo_prueba", "test"),
            MI("yaren", "YAREN", isEnglish ? "main modes" : "modos principales", {0,229,255}, "", "", true, "sub_yaren", "yaren"),
        };
        subMenuMap["sub_modo_prueba"] = { isEnglish ? "TEST MODE" : "MODO PRUEBA", {255,140,0}, {
            MI("test_camara", isEnglish ? "CAMERA" : "CAMARA", isEnglish ? "test camera" : "probar camara", {0,200,255}, ("python " + absCwd + "/src/YAREN2/CSI-Camera/simple_camera.py &").c_str(), "", false, "", "camera"),
            MI("test_mic", isEnglish ? "MICROPHONE" : "MICROFONO", isEnglish ? "test microphone" : "probar microfono", {0,255,128}, "", "", false, "", "microfono"),
            MI("test_motores", isEnglish ? "MOTORS" : "MOTORES", isEnglish ? "test motors" : "probar motores", {255,140,0}, "", "", true, "sub_motores", "motores"),
        }};
        subMenuMap["sub_motores"] = { isEnglish ? "TEST MOTORS" : "PROBAR MOTORES", {255,140,0}, {
            MI("motor_pos_orig", isEnglish ? "HOME POS." : "POS. ORIG.", isEnglish ? "initial position" : "posicion inicial", {255,180,50}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "yaren"),
            MI("motor_brazo_izq", isEnglish ? "LEFT ARM" : "BRAZO IZQ", isEnglish ? "left arm" : "brazo izquierdo", {220,100,50}, "", "", true, "sub_brazo_izq", "brazo_izq"),
            MI("motor_brazo_der", isEnglish ? "RIGHT ARM" : "BRAZO DER", isEnglish ? "right arm" : "brazo derecho", {200,120,60}, "", "", true, "sub_brazo_der", "brazo_der"),
            MI("motor_base", "BASE", isEnglish ? "base rotation" : "giro de base", {180,100,80}, "", "", true, "sub_base", "girar_base"),
            MI("motor_cabeza", isEnglish ? "HEAD" : "CABEZA", isEnglish ? "head movement" : "movimiento cabeza", {160,80,100}, "", "", true, "sub_cabeza", "girar_cabeza"),
        }};
        subMenuMap["sub_brazo_izq"] = { isEnglish ? "LEFT ARM" : "BRAZO IZQUIERDO", {220,100,50}, {
            MI("brazo_izq_alto", isEnglish ? "UP" : "ARRIBA", isEnglish ? "up position" : "posicion arriba", {220,110,55}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5,-3.0, 0.0, 3.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "arriba"),
            MI("brazo_izq_med", isEnglish ? "MIDDLE" : "MEDIO", isEnglish ? "middle position" : "posicion media", {180,80,40}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, -1.5, 0.0, 3.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "medio"),
            MI("brazo_izq_bajo", isEnglish ? "DOWN" : "BAJO", isEnglish ? "down position" : "posicion baja", {200,90,45}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "abajo"),
        }};
        subMenuMap["sub_brazo_der"] = { isEnglish ? "RIGHT ARM" : "BRAZO DERECHO", {200,120,60}, {
            MI("brazo_der_alto", isEnglish ? "UP" : "ARRIBA", isEnglish ? "up position" : "posicion arriba", {200,130,65}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 3.0, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "arriba"),
            MI("brazo_der_med", isEnglish ? "MIDDLE" : "MEDIO", isEnglish ? "middle position" : "posicion media", {160,100,50}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 1.5, 0.0, -3.0, 0.0, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "medio"),
            MI("brazo_der_bajo", isEnglish ? "DOWN" : "BAJO", isEnglish ? "down position" : "posicion baja", {180,110,55}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "abajo"),
        }};
        subMenuMap["sub_base"] = { "BASE", {180,100,80}, {
            MI("base_izq", isEnglish ? "LEFT" : "IZQUIERDA", isEnglish ? "turn left" : "girar izquierda", {180,110,85}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [-1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "izquierda"),
            MI("base_der", isEnglish ? "RIGHT" : "DERECHA", isEnglish ? "turn right" : "girar derecha", {170,90,75}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "derecha"),
            MI("base_orig", isEnglish ? "HOME POS." : "POS. ORIGINAL", isEnglish ? "original position" : "posicion original", {160,80,70}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "yaren"),
        }};
        subMenuMap["sub_cabeza"] = { isEnglish ? "HEAD" : "CABEZA", {160,80,100}, {
            MI("cabeza_izq", isEnglish ? "LEFT" : "IZQUIERDA", isEnglish ? "turn left" : "girar izquierda", {165,85,105}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, -0.8, 0.2, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "izquierda"),
            MI("cabeza_der", isEnglish ? "RIGHT" : "DERECHA", isEnglish ? "turn right" : "girar derecha", {150,70,90}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.8, 0.2, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "derecha"),
            MI("cabeza_orig", isEnglish ? "HOME POS." : "POS. ORIGINAL", isEnglish ? "original position" : "posicion original", {140,60,80}, "timeout 5 ros2 topic pub --once /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \"{joint_names: ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7','joint_8', 'joint_9', 'joint_10', 'joint_11', 'joint_12'], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.5], time_from_start: {sec: 2, nanosec: 0}}]}\"", "", false, "", "yaren"),
        }};
        subMenuMap["sub_yaren"] = { "YAREN", {0,229,255}, {
            MI("yaren_mimic", "MIMIC", isEnglish ? "Yaren imitates you" : "Yaren te Imita", {0,229,255}, "ros2 launch yaren_arm_mimic yaren_mimic.launch.py &", "for pid in $(ps aux | grep -E 'yaren_mimic' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", false, "", "mimic"),
            MI("yaren_ai", "IA", isEnglish ? "Artificial Intelligence" : "Inteligencia Artificial", {29,233,22}, "", "", true, "sub_yaren_ai", "chat"),            
            MI("yaren_movements", isEnglish ? "MOVEMENTS" : "MOVIMIENTOS", isEnglish ? "Yaren moves" : "Yaren se mueve", {251,64,224}, "", "", true, "sub_yaren_movements", "movements"),
            MI("yaren_emotions", isEnglish ? "EMOTIONS" : "EMOCIONES", isEnglish ? "Detects your emotion" : "Yaren detecta tu emocion", {82,82,255}, "", "", false, "", "emotions", {"csi_cam_node","detector"}),
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
            isEnglish ? "No visual guide" : "Sin guia visual",
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
            isEnglish ? "SESSION"      : "SESION",
            isEnglish ? "Coming soon"  : "Proximamente",
            {80,80,80},
            "", "", false, "", "dice"),
        }};
        subMenuMap["sub_yaren"].key = "sub_yaren";
        subMenuMap["sub_yaren_movements"] = { isEnglish ? "MOVEMENTS" : "MOVIMIENTOS", {251,64,224}, {
            MI("yaren_rutina1", isEnglish ? "ROUTINE 1" : "RUTINA 1", isEnglish ? "Routines" : "Rutinas", {251,64,224}, ("python3 " + mvDir + "yaren_movement.py &").c_str(), "for pid in $(ps aux | grep -E 'yaren_rutina1' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", false, "", "rutina1"),
            MI("yaren_rutina2", isEnglish ? "ROUTINE 2" : "RUTINA 2", isEnglish ? " Routines" : "Rutinas", {220,80,200}, ("python3 " + mvDir + "yaren_fullmovement.py &").c_str(), "for pid in $(ps aux | grep -E 'yaren_fullmovement' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", false, "", "rutina2"),
            MI("yaren_rutinanueva", isEnglish ? "PERSONAL ROUTINES" : "RUTINAS PERSONALES", isEnglish ? "manage and record" : "gestionar y grabar", {130, 80, 255}, "INTERNAL_ROUTINES", "", false, "", "rutina_nueva"),
        }};
        subMenuMap["sub_yaren_filtros"] = { isEnglish ? "FILTERS" : "FILTROS", {105,240,174}, {
            MI("yaren_animales", isEnglish ? "ANIMALS" : "ANIMALES", isEnglish ? "animal filter" : "filtro animal", {105,240,174}, "", "", false, "", "animales", {"csi_cam_node","face_landmark_publisher", "filtro_animales"}),
            MI("yaren_accesorios", isEnglish ? "ACCESSORIES" : "ACCESORIOS", isEnglish ? "accessories filter" : "filtro accesorios", {60,200,130}, "", "", false, "", "accesorios", {"csi_cam_node","face_landmark_publisher", "face_filter_node"}),
            MI("yaren_fondo", isEnglish ? "VIRT. BACKGROUNDS" : "FONDOS VIRTUAL", isEnglish ? "virtual background" : "filtro fondo virtual", {60,200,130}, "", "", false, "", "accesorios", {"csi_cam_node","virtual_background_node"}),
        }};
        subMenuMap["sub_yaren_p2"] = { "YAREN", {0,229,255}, {
            MI("yaren_radio", "YAREN RADIO", isEnglish ? "music and animation" : "musica y animacion", {255,80,160}, "", "", true, "sub_yaren_radio", "radio"),
            MI("yaren_juegos", isEnglish ? "GAMES" : "JUEGOS", isEnglish ? "Play with Yaren" : "Juega con Yaren", {255, 140, 50}, "", "", true, "sub_yaren_juegos", "dice"),
        }};
        subMenuMap["sub_yaren_p2"].key = "sub_yaren_p2";

        subMenuMap["sub_yaren_juegos"] = { isEnglish ? "GAMES" : "JUEGOS", {255, 140, 50}, {
            MI("yaren_dice", isEnglish ? "SAYS" : "DICE", isEnglish ? "Play Yaren Says" : "Jugar Yaren Dice", {64,171,255}, "", "", true, "sub_yaren_dice", "dice"),
            MI("yaren_chistes", isEnglish ? "JOKES" : "CHISTES", isEnglish ? "Yaren tells jokes" : "Yaren cuenta chistes", {255, 200, 50}, "", "for pid in $(ps aux | grep -E 'chistes_node' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", false, "", "chistes", {"chistes_node"}),
            MI("yaren_memoria", isEnglish ? "MEMORY GAME" : "JUEGO MEMORIA", isEnglish ? "Remember the colors" : "Acuerdate de los colores", {80, 200, 255}, "", "", false, "", "dice", {"memoria_node"}),
            MI("yaren_dance", isEnglish ? "DANCE GAME" : "JUEGO DE BAILE", isEnglish ? "Follow the rhythm" : "Sigue el ritmo", {50, 255, 150}, "", "", false, "", "movements", {"dance_game_node"}),
        }};
        subMenuMap["sub_yaren_juegos"].key = "sub_yaren_juegos";
        subMenuMap["sub_yaren_radio"] = { "YAREN RADIO", {255,80,160}, {
            MI("radio_musica", isEnglish ? "MUSIC" : "MUSICA", isEnglish ? "play music" : "reproducir musica", {255,120,200}, "INTERNAL_RADIO", "", false, "", "musica"),
            MI("radio_videos", "VIDEOS", isEnglish ? "play videos" : "reproducir videos", {200,60,140}, "", "", true, "sub_yaren_videos", "video"),
        }};
        subMenuMap["sub_yaren_radio"].key = "sub_yaren_radio";
        subMenuMap["sub_yaren_videos"] = { "VIDEOS", {200,60,140}, {
            MI("vid_pollito", isEnglish ? "LITTLE CHICK PIO" : "POLLITO PIO", isEnglish ? "Farm Songs" : "Canciones de la Granja", {0, 200, 255}, ("python3 " + rdDir + "pollitopio.py &").c_str(), "for pid in $(ps aux | grep -E 'pollitopio.py' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", false, "", "piopio"),
            MI("vid_gallina", isEnglish ? "TURULECA HEN" : "GALLINA TURULECA", isEnglish ? "Yaren Songs" : "Canciones de Yaren", {255, 150, 50}, ("python3 " + rdDir + "gallinaturuleca.py &").c_str(), "for pid in $(ps aux | grep -E 'gallinaturuleca.py' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", false, "", "gallinaturuleca"),
            MI("vid_vaca", isEnglish ? "LOLA THE COW" : "LA VACA LOLA", isEnglish ? "Kids Songs" : "Canciones Infantiles", {100, 255, 100}, ("python3 " + rdDir + "vacalola.py &").c_str(), "for pid in $(ps aux | grep -E 'vacalola.py' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", false, "", "vacalola"),
            MI("vid_susanita", isEnglish ? "LITTLE SUSAN" : "SUSANITA", isEnglish ? "Zenon's Farm" : "La Granja de Zenon", {255, 100, 200}, ("python3 " + rdDir + "susanita.py &").c_str(), "for pid in $(ps aux | grep -E 'susanita.py' | grep -v grep | awk '{print $2}'); do kill -15 $pid; done", false, "", "susanita"),
        }};
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
                            : "Fallo al grabar audio.\nRevisa el microfono.", 4.0);
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
                root.title       = isEnglish ? "MAIN MENU" : "MENU PRINCIPAL";
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
                root.title       = isEnglish ? "MAIN MENU" : "MENU PRINCIPAL";
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
                    NavLevel root; root.title = isEnglish ? "MAIN MENU" : "MENU PRINCIPAL";
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
            {
                std::lock_guard<std::mutex> lk(modeFlagMutex);
                showRadio_ = true;
            }
            radioApp.reset();
            hoveredItem = -1;
            return;
        }
        if (item.cmd == "INTERNAL_ROUTINES") {
            {
                std::lock_guard<std::mutex> lk(modeFlagMutex);
                showRoutines_ = true;
            }
            routinesApp.refresh();
            hoveredItem = -1;
            stopMenuMusic();
            return;
        }
        navStack.clear();
        stopMenuMusic();
        hoveredItem = -1;
        hoveredBack = hoveredStop = hoveredExit = false;

        if (!activeMode.empty() && (!activeStopCmd.empty() || !active_lifecycle_nodes.empty())) {
            std::vector<std::string> to_keep, to_deactivate;
            for (const auto& node : active_lifecycle_nodes) {
                if (std::find(item.lifecycle_nodes.begin(), item.lifecycle_nodes.end(), node) != item.lifecycle_nodes.end())
                    to_keep.push_back(node);
                else
                    to_deactivate.push_back(node);
            }
            for (const auto& node : to_deactivate)
                change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
            for (const auto& node : to_keep)
                change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

            if (!active_lifecycle_nodes.empty())
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

            active_lifecycle_nodes.clear();
            // FIX-D: lanzar stop command en thread separado
            std::string prevStop = activeStopCmd;
            if (!prevStop.empty()) {
                std::thread([prevStop]() { std::system(prevStop.c_str()); }).detach();
            }
            activeMode = ""; activeStopCmd = "";
            if (publish_mode) {
                auto msg = std_msgs::msg::String(); msg.data = "idle";
                modePublisher->publish(msg);
            }

            activeMode    = item.id;
            activeStopCmd = item.stopCmd;
            active_lifecycle_nodes = item.lifecycle_nodes;
            for (const auto& node : item.lifecycle_nodes)
                change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        } else {
            activeMode    = item.id;
            activeStopCmd = item.stopCmd;
            active_lifecycle_nodes = item.lifecycle_nodes;
            for (const auto& node : active_lifecycle_nodes)
                change_lifecycle_state(node, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
        }

        std::string cleanCmd = item.cmd;
        size_t pos = cleanCmd.find_last_not_of(" \t&");
        if (pos != std::string::npos) cleanCmd = cleanCmd.substr(0, pos + 1);

        if (item.stopCmd.empty() && item.lifecycle_nodes.empty()) {
            if (publish_mode) {
                auto msg = std_msgs::msg::String(); msg.data = item.id;
                modePublisher->publish(msg);
            }
            if (!cleanCmd.empty()) {
                std::thread([this, cleanCmd]() {
                    int ret = std::system(cleanCmd.c_str());
                    if (ret != 0) {
                        RCLCPP_ERROR(get_logger(), "[CMD] Fallo comando (Exit %d): %s", ret, cleanCmd.c_str());
                        showErrorOverlay(isEnglish ? "Command could not be executed." : "No se ha podido realizar\nel comando.", 3.0);
                    }
                }).detach();
            }
            return;
        }

        if (publish_mode) {
            auto msg = std_msgs::msg::String(); msg.data = activeMode;
            modePublisher->publish(msg);
        }

        if (!cleanCmd.empty()) {
            std::thread([this, cleanCmd, prevMode = item.id]() {
                auto start = std::chrono::steady_clock::now();
                int ret = std::system(cleanCmd.c_str());
                auto end = std::chrono::steady_clock::now();
                double elapsedSecs = std::chrono::duration<double>(end - start).count();
                if (ret != 0 && elapsedSecs < 1.5) {
                    RCLCPP_ERROR(get_logger(), "[CMD] Fallo inmediato (Exit %d): %s", ret, cleanCmd.c_str());
                    showErrorOverlay(isEnglish ? "Command failed immediately." : "No se ha podido realizar\nel comando.", 4.0);
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
                    root.title = isEnglish ? "MAIN MENU" : "MENU PRINCIPAL";
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
        bool hs = hoveredSettings.load(); // FIX-M: leer atomic
        cv::Scalar bg   = hs ? cv::Scalar(30,50,70) : cv::Scalar(12,20,35);
        cv::Scalar bord = hs ? cv::Scalar(0,200,255) : cv::Scalar(40,70,100);
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
                                cv::Scalar tint = hs ? cv::Scalar(0,220,255) : cv::Scalar(180,190,200);
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
            cv::Scalar ic = hs ? cv::Scalar(0,220,255) : cv::Scalar(80,130,160);
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
        bool hp = hoveredPower.load(); // FIX-M: leer atomic
        cv::Scalar bg   = hp ? cv::Scalar(30, 30, 180) : cv::Scalar(20, 20, 100);
        cv::Scalar bord = hp ? cv::Scalar(80, 80, 255) : cv::Scalar(50, 50, 200);
        cv::rectangle(frame, powerButtonRect, bg, cv::FILLED);
        cv::rectangle(frame, powerButtonRect, bord, 1, cv::LINE_AA);
        int cx = powerButtonRect.x + btnSz/2, cy = powerButtonRect.y + btnSz/2;
        cv::Scalar ic = hp ? cv::Scalar(255, 255, 255) : cv::Scalar(200, 200, 200);
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
        if (isRoot) title = isEnglish ? "SELECT A MODE" : "SELECCIONA UN MODO";
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
        bool hasStop = !activeMode.empty() && (!activeStopCmd.empty() || !active_lifecycle_nodes.empty()) &&
                       activeMode != "yaren_emotions" &&
                       activeMode != "yaren_animales" &&
                       activeMode != "yaren_accesorios" &&
                       activeMode != "yaren_fondo" &&
                       activeMode != "yaren_radio" &&
                       activeMode != "radio_musica" &&
                       activeMode != "yaren_rutina1" &&         
                       activeMode != "yaren_rutina2" &&         
                       activeMode != "yaren_dice_con_ayuda" &&  
                       activeMode.rfind("vid_", 0) != 0;
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
            drawTextInRect(frame, (isEnglish ? "STOP: " : "DETENER: ") + activeMode, stopButtonRect, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255,255,255), 1);
            cx += stopW + gap;
        } else { stopButtonRect = {0,0,0,0}; }
        if (hasBack) {
            backButtonRect = { cx, btnY, navW, btnH };
            cv::Scalar c = hoveredBack ? cv::Scalar(60,60,60) : cv::Scalar(30,30,35);
            cv::rectangle(frame, backButtonRect, c, cv::FILLED);
            cv::rectangle(frame, backButtonRect, cv::Scalar(120,120,120), 1, cv::LINE_AA);
            drawTextInRect(frame, isEnglish ? "BACK" : "VOLVER", backButtonRect, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(200,200,200), 1);
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
        drawTextInRect(frame, isEnglish ? "EXIT" : "SALIR", exitButtonRect, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(200,200,200), 1);
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
        bool blocked = (item.id == "yaren_chat") && !chatAvailable_;
        if (blocked) hovered = false;  // no hover visual si bloqueado
        int cx = item.rect.x, cy = item.rect.y, cw = item.rect.width, ch = item.rect.height;
        cv::Scalar a = blocked ? cv::Scalar(70,70,70) : item.color;
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
        // Overlay bloqueado
        if (blocked) {
            // Panel oscuro semitransparente
            cv::Mat ov2 = frame.clone();
            cv::rectangle(ov2, item.rect, cv::Scalar(0,0,0), cv::FILLED);
            cv::addWeighted(ov2, 0.35, frame, 0.65, 0, frame);
            // Icono candado
            int lcx = cx + cw - 22, lcy = cy + 16;
            cv::rectangle(frame, {lcx-7, lcy, 14, 11}, cv::Scalar(160,160,160), 1, cv::LINE_AA);
            cv::ellipse(frame, {lcx, lcy}, {5,5}, 0, 180, 360, cv::Scalar(160,160,160), 1, cv::LINE_AA);
            // Razón
            std::string reason = isEnglish ? "No internet" : "Sin internet";
            int rbl=0; cv::Size rs = cv::getTextSize(reason, cv::FONT_HERSHEY_PLAIN, 0.75, 1, &rbl);
            cv::putText(frame, reason, {cx+(cw-rs.width)/2, cy+ch-2},
                        cv::FONT_HERSHEY_PLAIN, 0.75, cv::Scalar(130,130,130), 1, cv::LINE_AA);
        }
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
        updateViseme();
        {
            int idxA = currentVisemeIdx_.load();
            int idxB = nextVisemeIdx_.load();
            float bl = visemeBlend_;
            if (ttsActive.load()
                && !mouthSprites_[idxA].empty()
                && !mouthSprites_[idxB].empty()) {
                overlayImageAlpha(res, mouthSprites_[idxA], 1.0f - bl);
                overlayImageAlpha(res, mouthSprites_[idxB], bl);
            } else {
                overlayImage(res, ttsActive ? mouthOpenImg : mouthClosedImg);
            }
        }        
        if (eyesOpenImg.empty() && eyesClosedImg.empty() && mouthOpenImg.empty() && mouthClosedImg.empty()) {
            cv::putText(res, isEnglish ? "WARNING: face images not found" : "ADVERTENCIA: imagenes de cara no encontradas", { 20, canvasSize.height / 2 }, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 100, 255), 1, cv::LINE_AA);
        }
        return res;
    }
    // =============================================================================
    //  renderLoadingScreen — "ROBOT AWAKENING" v2
    //  La imagen de Yaren se revela de abajo → arriba sincronizada con la barra
    //  de progreso: 0% = imagen oculta, 100% = imagen completamente visible.
    //  Reemplaza el bloque completo de renderLoadingScreen() en face_screen.cpp
    // =============================================================================
    void renderLoadingScreen(cv::Mat& frame) {
        int W = frame.cols, H = frame.rows;
        frame.setTo(cv::Scalar(4, 5, 10));

        double t = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        std::string status;
        int prog;
        {
            std::lock_guard<std::mutex> lk(configStatusMutex);
            status = configStatus;
            prog   = configProgress.load();
        }
        float pct = std::min(1.0f, (float)prog / configTotal);

        const int cx = W / 2, cy = H / 2 - 15;

        // ─────────────────────────────────────────────────────────────────────────
        // 1. FONDO RADIAL CÁLIDO
        // ─────────────────────────────────────────────────────────────────────────
        {
            float maxR = std::sqrt((float)(W*W + H*H)) * 0.55f;
            for (int y = 0; y < H; y += 2) {
                for (int x = 0; x < W; x += 2) {
                    float dx = (float)(x - cx), dy = (float)(y - cy);
                    float norm = std::min(1.0f, std::sqrt(dx*dx+dy*dy) / maxR);
                    float heat = (1.0f - norm)*(1.0f - norm) * 0.20f;
                    uchar r2 = (uchar)(heat * 245);
                    uchar g2 = (uchar)(heat * 62);
                    uchar b2 = (uchar)(heat * 10);
                    for (int dy2 = 0; dy2 < 2 && y+dy2 < H; dy2++)
                        for (int dx2 = 0; dx2 < 2 && x+dx2 < W; dx2++)
                            frame.at<cv::Vec3b>(y+dy2, x+dx2) = cv::Vec3b(b2, g2, r2);
                }
            }
        }

        // ─────────────────────────────────────────────────────────────────────────
        // 2. ONDAS DE SONAR
        // ─────────────────────────────────────────────────────────────────────────
        for (int w = 0; w < 4; ++w) {
            double phase    = std::fmod(t * 0.55 + w * 0.5, 2.0) / 2.0;
            int    radius   = (int)(phase * 260);
            double alpha    = (1.0 - phase)*(1.0 - phase) * 0.45;
            if (alpha < 0.015 || radius < 2) continue;
            cv::Mat ov = frame.clone();
            uchar r2 = (uchar)(200 * (1.0 - phase * 0.5));
            uchar g2 = (uchar)(85  * (1.0 - phase * 0.6));
            cv::circle(ov, {cx, cy}, radius, cv::Scalar(0, g2, r2),
                    1 + (int)(2.0*(1.0-phase)), cv::LINE_AA);
            cv::addWeighted(ov, alpha, frame, 1.0-alpha, 0, frame);
        }

        // ─────────────────────────────────────────────────────────────────────────
        // 3. NÚCLEO CENTRAL PULSANTE (Se dibuja antes para quedar en el fondo)
        // ─────────────────────────────────────────────────────────────────────────
        {
            double beatFreq = 1.2 + pct * 2.8;
            double beat = 0.68 + 0.32*std::sin(t * beatFreq * CV_PI * 2.0);
            // Halo exterior
            {
                cv::Mat ov = frame.clone();
                cv::circle(ov, {cx,cy}, (int)(40*beat),
                        cv::Scalar(0,(int)(45*beat),(int)(200*beat)), cv::FILLED, cv::LINE_AA);
                cv::addWeighted(ov, 0.18*beat, frame, 1.0-0.18*beat, 0, frame);
            }
            // Corona
            {
                cv::Mat ov = frame.clone();
                cv::circle(ov, {cx,cy}, (int)(27*beat),
                        cv::Scalar(0,(int)(100*beat),(int)(230*beat)), 2, cv::LINE_AA);
                cv::addWeighted(ov, 0.55, frame, 0.45, 0, frame);
            }
            // Núcleo sólido
            {
                int coreR = 13 + (int)(pct*5);
                uchar cr  = (uchar)(200 + 55*beat);
                uchar cg  = (uchar)((80 + pct*130) * beat);
                uchar cb  = (uchar)(pct * 25 * beat);
                cv::circle(frame, {cx,cy}, coreR, cv::Scalar(cb,cg,cr), cv::FILLED, cv::LINE_AA);
                cv::circle(frame, {cx-3,cy-3}, (int)(4*beat),
                        cv::Scalar(180,220,255), cv::FILLED, cv::LINE_AA);
            }
        }

        // ─────────────────────────────────────────────────────────────────────────
        // 4. IMAGEN DE YAREN (Más alta, tapando núcleo y recuperando color)
        // ─────────────────────────────────────────────────────────────────────────
        if (!yarenSplashImg.empty() && pct > 0.01f) {
            const int imgH = 280; 
            float aspect = (float)yarenSplashImg.cols / yarenSplashImg.rows;
            const int imgW = (int)(imgH * aspect);
            const int imgX = cx - imgW / 2;
            const int imgY = cy - imgH / 2 - 50; // ¡Ajustado para subirlo y no chocar con el texto!

            cv::Mat resized;
            cv::resize(yarenSplashImg, resized, {imgW, imgH}, 0, 0, cv::INTER_AREA);

            int revealedH = (int)(imgH * pct);
            int revealStartRow = imgH - revealedH;
            int revealStartY   = imgY + revealStartRow;

            // Transición suave al color original al llegar al final
            float colorBlend = 0.0f;
            if (pct > 0.85f) { // Empieza a recuperar color del 85% al 100%
                colorBlend = std::min(1.0f, (pct - 0.85f) / 0.15f);
            }

            for (int iy = 0; iy < imgH; ++iy) {
                for (int ix = 0; ix < imgW; ++ix) {
                    int fy = imgY + iy, fx = imgX + ix;
                    if (fy < 0 || fy >= H || fx < 0 || fx >= W) continue;
                    
                    float srcA = 1.0f;
                    cv::Vec3b srcPx(0,0,0);
                    
                    if (resized.channels() == 4) {
                        cv::Vec4b p4 = resized.at<cv::Vec4b>(iy,ix);
                        srcPx = cv::Vec3b(p4[0], p4[1], p4[2]);
                        srcA = p4[3] / 255.f;
                    } else {
                        srcPx = resized.at<cv::Vec3b>(iy,ix);
                    }
                    
                    if (srcA < 0.04f) continue;

                    cv::Vec3b& bg = frame.at<cv::Vec3b>(fy,fx);

                    if (iy < revealStartRow) {
                        // Zona NO revelada: Sombra casi invisible
                        bg[0] = cv::saturate_cast<uchar>(bg[0]*(1.f-srcA*0.8f));
                        bg[1] = cv::saturate_cast<uchar>(bg[1]*(1.f-srcA*0.8f) + 10 * srcA);
                        bg[2] = cv::saturate_cast<uchar>(bg[2]*(1.f-srcA*0.8f) + 20 * srcA);
                    } else {
                        // Zona REVELADA: Silueta monocromática
                        cv::Vec3b silColor(0, 160, 255); // BGR dorado/ámbar
                        
                        int distToEdge = iy - revealStartRow;
                        if (distToEdge < 6 && pct < 0.99f) {
                            double scanPulse = 0.65 + 0.35*std::sin(t*9.0);
                            float scanStr = (1.0f - distToEdge/6.0f) * (float)scanPulse;
                            silColor[0] = cv::saturate_cast<uchar>(silColor[0] + 50 * scanStr);
                            silColor[1] = cv::saturate_cast<uchar>(silColor[1] + 95 * scanStr);
                            silColor[2] = 255;
                        }

                        // Mezcla: empieza en 100% silueta y transiciona a 100% colores originales
                        cv::Vec3b finalColor;
                        for (int ch = 0; ch < 3; ++ch) {
                            finalColor[ch] = cv::saturate_cast<uchar>(srcPx[ch] * colorBlend + silColor[ch] * (1.0f - colorBlend));
                            bg[ch] = cv::saturate_cast<uchar>(finalColor[ch]*srcA + bg[ch]*(1.f-srcA));
                        }
                    }
                }
            }

            // ── Línea del scanline (borde exacto) ──
            if (pct < 0.99f && revealStartY >= imgY && revealStartY < imgY+imgH) {
                double scanPulse = 0.65 + 0.35*std::sin(t*9.0);
                cv::line(frame,
                        {imgX - 4, revealStartY},
                        {imgX + imgW + 4, revealStartY},
                        cv::Scalar(0, (int)(200*scanPulse), (int)(255*scanPulse)),
                        2, cv::LINE_AA);
                for (int p = 0; p < 5; ++p) {
                    double pPhase = std::fmod(t*1.8 + p*0.38, 1.0);
                    int px2 = imgX + (int)(pPhase * imgW);
                    int py2 = revealStartY - 1 - (p % 3);
                    if (px2 >= 0 && px2 < W && py2 >= 0 && py2 < H)
                        cv::circle(frame, {px2, py2}, 2,
                                cv::Scalar(0,(int)(160*scanPulse),(int)(255*scanPulse)),
                                cv::FILLED, cv::LINE_AA);
                }
            }
        }
        // ─────────────────────────────────────────────────────────────────────────
        // 5. TÍTULO "YAREN" — letras que se escriben una a una
        // ─────────────────────────────────────────────────────────────────────────
        {
            const std::string title = "YAREN";
            const int titleY = H - 128;
            const int fontFace = cv::FONT_HERSHEY_DUPLEX;
            const double fontScale = 1.95;
            const int thickness = 2;
            int lettersToShow = std::min(5, std::max(1, (int)(pct * 10.0f)));

            int bl = 0;
            cv::Size fullSz = cv::getTextSize(title, fontFace, fontScale, thickness, &bl);
            int startX = (W - fullSz.width) / 2;

            // Sombra naranja
            for (int s = 4; s >= 1; --s) {
                double a = 0.09*(5-s)*pct;
                cv::putText(frame, title.substr(0, lettersToShow),
                            {startX+s, titleY+s}, fontFace, fontScale,
                            cv::Scalar(0,(int)(55*a),(int)(175*a)),
                            thickness+s*2, cv::LINE_AA);
            }
            // Letras individuales con color
            std::string prefix;
            for (int i = 0; i < lettersToShow; ++i) {
                bool isLast = (i==lettersToShow-1) && (lettersToShow<5);
                double lp = isLast ? (0.55+0.45*std::sin(t*7.5)) : 1.0;
                int pw = prefix.empty() ? 0
                    : cv::getTextSize(prefix,fontFace,fontScale,thickness,&bl).width;
                std::string letter(1, title[i]);
                cv::putText(frame, letter,
                            {startX+pw, titleY}, fontFace, fontScale,
                            cv::Scalar((uchar)(12*lp),(uchar)((95+pct*110)*lp),(uchar)(200*lp)),
                            thickness, cv::LINE_AA);
                prefix += title[i];
            }
            // Cursor
            if (lettersToShow < 5) {
                double cp = std::fmod(t*3.0, 1.0);
                if (cp < 0.52) {
                    int pw = cv::getTextSize(prefix,fontFace,fontScale,thickness,&bl).width;
                    cv::line(frame, {startX+pw+5, titleY-32}, {startX+pw+5, titleY+6},
                            cv::Scalar(0,140,220), 2, cv::LINE_AA);
                }
            }
            // Subtítulo
            if (pct > 0.25f) {
                float a2 = std::min(1.0f, (pct-0.25f)/0.18f);
                cv::Mat ov = frame.clone();
                const std::string sub = "ROBOT SOCIAL";
                cv::Size ss = cv::getTextSize(sub,cv::FONT_HERSHEY_PLAIN,1.1,1,&bl);
                cv::putText(ov, sub, {(W-ss.width)/2, titleY+26},
                            cv::FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0,60,110), 1, cv::LINE_AA);
                cv::addWeighted(ov, a2, frame, 1.0-a2, 0, frame);
            }
            // Separador con diamante
            if (pct > 0.32f) {
                float sp = std::min(1.0f, (pct-0.32f)/0.22f);
                int sepY = titleY+44, ll = (int)(155*sp);
                double dp = 0.5+0.5*std::sin(t*2.8);
                cv::line(frame,{cx-8,sepY},{cx-ll,sepY},cv::Scalar(0,45,85),1,cv::LINE_AA);
                cv::line(frame,{cx+8,sepY},{cx+ll,sepY},cv::Scalar(0,45,85),1,cv::LINE_AA);
                std::vector<cv::Point> diamond={{cx,sepY-5},{cx+5,sepY},{cx,sepY+5},{cx-5,sepY}};
                cv::fillPoly(frame,diamond,cv::Scalar(0,(int)(95*dp),(int)(195*dp)));
            }
        }

        // ─────────────────────────────────────────────────────────────────────────
        // 6. BARRA DE PROGRESO con partículas en el frente
        // ─────────────────────────────────────────────────────────────────────────
        {
            int barY = H - 72;
            int barW = W * 72 / 100;
            int barX = (W - barW) / 2;
            int barH = 7;

            cv::rectangle(frame, {barX, barY, barW, barH}, cv::Scalar(6,14,22), cv::FILLED);

            int fillW = (int)(barW * pct);
            if (fillW > 0) {
                // Relleno naranja con extremo brillante
                for (int x = 0; x < fillW; ++x) {
                    float r2 = (float)x / barW;
                    uchar fc_r = (uchar)(110 + r2*90);
                    uchar fc_g = (uchar)(42  + r2*65);
                    uchar fc_b = 0;
                    float edgeDist = (float)(fillW - x) / (barW * 0.07f);
                    if (edgeDist < 1.0f) {
                        double ep = 0.7+0.3*std::sin(t*8.0);
                        fc_r = cv::saturate_cast<uchar>(fc_r + (255-fc_r)*(1.0f-edgeDist)*ep);
                        fc_g = cv::saturate_cast<uchar>(fc_g + (155-fc_g)*(1.0f-edgeDist)*ep);
                        fc_b = (uchar)((1.0f-edgeDist)*55*ep);
                    }
                    for (int yy = 0; yy < barH; ++yy)
                        frame.at<cv::Vec3b>(barY+yy, barX+x) = cv::Vec3b(fc_b, fc_g, fc_r);
                }
                // Brillo superior
                for (int x = 0; x < fillW; ++x)
                    if (barX+x < W) {
                        cv::Vec3b& px = frame.at<cv::Vec3b>(barY, barX+x);
                        px[0] = std::min(255, px[0]+40);
                        px[1] = std::min(255, px[1]+55);
                        px[2] = std::min(255, px[2]+30);
                    }
                // Partículas escapando del frente
                if (pct < 0.99f) {
                    for (int p = 0; p < 3; ++p) {
                        double pPhase = std::fmod(t*(2.0+p*0.6)+p*1.2, 1.5);
                        if (pPhase > 1.0) continue;
                        int px2 = barX + fillW + (int)(pPhase*14);
                        int py2 = barY + barH/2 + (p-1)*2;
                        double pa = 1.0 - pPhase;
                        if (px2>=0 && px2<W && py2>=0 && py2<H)
                            cv::circle(frame, {px2,py2}, 2,
                                    cv::Scalar(0,(int)(100*pa),(int)(220*pa)),
                                    cv::FILLED, cv::LINE_AA);
                    }
                }
            }

            cv::rectangle(frame, {barX-1,barY-1,barW+2,barH+2}, cv::Scalar(0,35,65), 1, cv::LINE_AA);

            for (int tick = 1; tick < 10; ++tick) {
                int tx = barX + (int)(barW * tick / 10.0f);
                uchar ta = (pct*10 >= tick) ? 55 : 22;
                cv::line(frame,{tx,barY-2},{tx,barY-1},cv::Scalar(0,ta,ta*2),1,cv::LINE_AA);
            }

            char pctBuf[8];
            snprintf(pctBuf, sizeof(pctBuf), "%d%%", (int)(pct*100));
            cv::putText(frame, pctBuf, {barX+barW+14, barY+barH},
                        cv::FONT_HERSHEY_DUPLEX, 0.55, cv::Scalar(0,140,210), 1, cv::LINE_AA);
        }

        // ─────────────────────────────────────────────────────────────────────────
        // 7. TEXTO DE ESTADO estilo terminal
        // ─────────────────────────────────────────────────────────────────────────
        {
            bool isReady = (status.find("listo")!=std::string::npos ||
                            status.find("ready")!=std::string::npos);
            
            std::string displayText = status; 
            
            if (!isReady && std::fmod(t*2.2,1.0) < 0.5) displayText += "_";
            cv::Scalar textColor = isReady ? cv::Scalar(40,210,100) : cv::Scalar(30,140,210);
            int bl = 0;
            cv::Size ts = cv::getTextSize(displayText, cv::FONT_HERSHEY_PLAIN, 1.0, 1, &bl);
            cv::putText(frame, displayText, {(W-ts.width)/2, H-36},
                        cv::FONT_HERSHEY_PLAIN, 1.0, textColor, 1, cv::LINE_AA);
            // Destello verde al terminar
            if (isReady) {
                double rp = 0.04*std::max(0.0,std::sin(t*5.0));
                if (rp > 0.005) {
                    cv::Mat ov = frame.clone();
                    ov.setTo(cv::Scalar(20,80,160));
                    cv::addWeighted(ov, rp, frame, 1.0-rp, 0, frame);
                }
            }
        }

        // ─────────────────────────────────────────────────────────────────────────
        // 8. BORDE EXTERIOR CON ESQUINAS DECORATIVAS
        // ─────────────────────────────────────────────────────────────────────────
        {
            double bp = 0.38 + 0.18*std::sin(t*1.4);
            cv::rectangle(frame, {2,2,W-4,H-4},
                        cv::Scalar(0,(int)(48*bp),(int)(95*bp)), 1, cv::LINE_AA);
            int cl = 18;
            cv::Scalar cc(0,(int)(95*bp),(int)(175*bp));
            cv::line(frame,{2,2},    {2+cl,2},    cc,2,cv::LINE_AA);
            cv::line(frame,{2,2},    {2,2+cl},    cc,2,cv::LINE_AA);
            cv::line(frame,{W-2,2},  {W-2-cl,2},  cc,2,cv::LINE_AA);
            cv::line(frame,{W-2,2},  {W-2,2+cl},  cc,2,cv::LINE_AA);
            cv::line(frame,{2,H-2},  {2+cl,H-2},  cc,2,cv::LINE_AA);
            cv::line(frame,{2,H-2},  {2,H-2-cl},  cc,2,cv::LINE_AA);
            cv::line(frame,{W-2,H-2},{W-2-cl,H-2},cc,2,cv::LINE_AA);
            cv::line(frame,{W-2,H-2},{W-2,H-2-cl},cc,2,cv::LINE_AA);
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
                renderLoadingScreen(frame);
            } else {
                frame = getFaceFrame();
            }

            // --- ESTO ES LO QUE FALTABA ---
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
                    cv::putText(frame, "Zzz...", {30, frame.rows - 30}, 
                                cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
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
    std::vector<cv::Mat> eyesFrames, mouthFrames;
    std::map<std::string, cv::Mat> iconMap;
    std::atomic<bool> ttsActive  { false };
    std::atomic<bool> isBlinking { false };
    std::atomic<bool> running    { false };
    std::atomic<bool> configuring{true};
    // FIX-B: configProgress como atomic<int> para acceso seguro desde múltiples hilos
    std::atomic<int>  configProgress{0};
    static constexpr int configTotal{19};
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
    // FIX: arrancar renderThread DESPUÉS de la construcción completa del nodo
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
