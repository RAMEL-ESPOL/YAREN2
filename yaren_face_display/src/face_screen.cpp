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
static std::vector<AlsaDevice> parseAlsaDevices(const std::string& cmd) {
    std::vector<AlsaDevice> result;
    FILE* pipe = popen(cmd.c_str(), "r");
    if (!pipe) return result;
    char buf[512];
    while (fgets(buf, sizeof(buf), pipe)) {
        std::string line(buf);
        if (line.rfind("card ", 0) != 0) continue;
        int cardN = -1, devN = -1;
        if (sscanf(buf, "card %d:", &cardN) != 1) continue;
        auto devPos = line.find(", device ");
        if (devPos == std::string::npos) continue;
        if (sscanf(line.c_str() + devPos, ", device %d:", &devN) != 1) continue;
        auto colonPos = line.find(": ");
        std::string labelRaw = (colonPos != std::string::npos)
            ? line.substr(colonPos + 2, devPos - colonPos - 2)
            : line;
        while (!labelRaw.empty() && (labelRaw.back() == '\n' || labelRaw.back() == ' '))
            labelRaw.pop_back();

        AlsaDevice d;
        d.hwId  = "plughw:" + std::to_string(cardN) + "," + std::to_string(devN);
        d.label = d.hwId + "  " + labelRaw;
        result.push_back(d);
    }
    pclose(pipe);
    return result;
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
        mics = parseAlsaDevices("arecord -l 2>/dev/null");
        spks = parseAlsaDevices("aplay  -l 2>/dev/null");
        if (mics.empty()) mics.push_back({"plughw:0,0", "(sin micrófonos detectados)"});
        if (spks.empty()) spks.push_back({"plughw:0,0", "(sin altavoces detectados)"});
        
        if (selectedMicId.empty()) { selMic = 0; selectedMicId = mics[0].hwId; }
        if (selectedSpkId.empty()) { selSpk = 0; selectedSpkId = spks[0].hwId; }

        std::time_t t = std::time(nullptr);
        std::tm* tm = std::localtime(&t);
        editDay  = tm->tm_mday;
        editMon  = tm->tm_mon + 1;
        editYear = tm->tm_year + 1900;
        editHour = tm->tm_hour;
        editMin  = tm->tm_min;

        initialMicId = selectedMicId;
        initialSpkId = selectedSpkId;
        initDay  = editDay;
        initMon  = editMon;
        editYear_init = editYear;
        initHour = editHour;
        initMin  = editMin;
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

        int available = H - 58 - 70;
        int micPanelH = available * 35 / 100;
        int spkPanelH = available * 35 / 100;
        int datPanelH = available - micPanelH - spkPanelH;

        int micTop = 58;
        int spkTop = micTop + micPanelH;
        int datTop = spkTop + spkPanelH;

        renderAudioPanel(frame, micTop, micPanelH, true);
        renderAudioPanel(frame, spkTop, spkPanelH, false);
        renderDatePanel (frame, datTop, datPanelH);
        renderBottomBar(frame, H - 62);
    }

    void handleMouse(int event, int x, int y) {
        if (event == cv::EVENT_MOUSEMOVE) {
            hoveredRect = findHovered(x, y);
            return;
        }
        if (event != cv::EVENT_LBUTTONDOWN) return;

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
        if (btnYearUp.contains({x,y}))  { editYear++; return; }
        if (btnYearDown.contains({x,y})){ editYear = std::max(2000, editYear-1); return; }
        if (btnHourUp.contains({x,y}))  { editHour = (editHour+1)%24; return; }
        if (btnHourDown.contains({x,y})){ editHour = (editHour+23)%24; return; }
        if (btnMinUp.contains({x,y}))   { editMin = (editMin+1)%60; return; }
        if (btnMinDown.contains({x,y})) { editMin = (editMin+59)%60; return; }

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
    int currentFaceIndex { 0 };  // 0=original, 1=money, 2=open, 3=ready, 4=tongue
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
            int maxChars = listW / 7;
            if ((int)label.size() > maxChars) label = label.substr(0, maxChars-2) + "..";

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

        cv::putText(frame, "FECHA Y HORA", {MARGIN, top + 20}, cv::FONT_HERSHEY_DUPLEX, 0.55, accent, 1, cv::LINE_AA);
        cv::line(frame, {MARGIN, top+26}, {MARGIN+200, top+26}, cv::Scalar(70,110,50), 1, cv::LINE_AA);

        std::time_t t_now = std::time(nullptr);
        char sysBuf[64];
        std::strftime(sysBuf, sizeof(sysBuf), "Sistema: %d/%m/%Y  %H:%M:%S", std::localtime(&t_now));
        cv::putText(frame, sysBuf, {W - 320, top + 20}, cv::FONT_HERSHEY_PLAIN, 0.85, cv::Scalar(60, 90, 60), 1, cv::LINE_AA);

        int* vals[5] = { &editDay, &editMon, &editYear, &editHour, &editMin };
        const char* labels[5] = { "DAY", "MONTH", "YEAR", "HOUR", "MIN" };
        cv::Rect* upBtns[5] = { &btnDayUp,  &btnMonUp,  &btnYearUp,  &btnHourUp,  &btnMinUp  };
        cv::Rect* dnBtns[5] = { &btnDayDown, &btnMonDown, &btnYearDown, &btnHourDown, &btnMinDown };

        int spinCount = 5;
        int spinW     = 110; 
        int spinH     = panelH - 44;
        int gap       = (W - MARGIN*2 - spinW*spinCount) / (spinCount + 1);
        int spinTop   = top + 34;

        for (int s = 0; s < spinCount; ++s) {
            int sx = MARGIN + gap + s * (spinW + gap);
            cv::Rect spinRect{sx, spinTop, spinW, spinH};
            cv::rectangle(frame, spinRect, cv::Scalar(10, 18, 32), cv::FILLED);
            cv::rectangle(frame, spinRect, cv::Scalar(50, 80, 50), 1, cv::LINE_AA);

            int btnSz   = 24;
            int marginL = 10;
            int arrowX  = sx + marginL;
            cv::Rect up{arrowX, spinTop + spinH/2 - btnSz - 2, btnSz, btnSz};
            cv::Rect dn{arrowX, spinTop + spinH/2 + 2, btnSz, btnSz};
            
            *upBtns[s] = up;
            *dnBtns[s] = dn;

            bool hovU = up.contains(hoveredRect.tl());
            cv::rectangle(frame, up, hovU ? cv::Scalar(45, 90, 45) : cv::Scalar(18, 30, 18), cv::FILLED);
            cv::rectangle(frame, up, cv::Scalar(60, 120, 60), 1, cv::LINE_AA);
            drawArrow(frame, {up.x + btnSz/2, up.y + btnSz/2}, true, hovU ? cv::Scalar(255, 255, 255) : accent, 5);

            bool hovD = dn.contains(hoveredRect.tl());
            cv::rectangle(frame, dn, hovD ? cv::Scalar(45, 90, 45) : cv::Scalar(18, 30, 18), cv::FILLED);
            cv::rectangle(frame, dn, cv::Scalar(60, 120, 60), 1, cv::LINE_AA);
            drawArrow(frame, {dn.x + btnSz/2, dn.y + btnSz/2}, false, hovD ? cv::Scalar(255, 255, 255) : accent, 5);

            int contentX = arrowX + btnSz + 5;
            int contentW = sx + spinW - contentX - 5;
            int lblBl = 0;
            cv::Size ls = cv::getTextSize(labels[s], cv::FONT_HERSHEY_PLAIN, 0.75, 1, &lblBl);
            cv::putText(frame, labels[s], {contentX + (contentW - ls.width)/2, spinTop + 16},  
                        cv::FONT_HERSHEY_PLAIN, 0.75, cv::Scalar(100, 150, 100), 1, cv::LINE_AA);

            char vbuf[16];
            if (s == 1) {
                const char* mnames[12] = {"ENE","FEB","MAR","ABR","MAY","JUN","JUL","AGO","SEP","OCT","NOV","DIC"};
                snprintf(vbuf, sizeof(vbuf), "%s", mnames[(*vals[s]-1)%12]);
            } else if (s == 2) {
                snprintf(vbuf, sizeof(vbuf), "%d", *vals[s]);
            } else {
                snprintf(vbuf, sizeof(vbuf), "%02d", *vals[s]);
            }

            int valBl = 0;
            double valScale = (s == 1 || s == 2) ? 0.7 : 0.85;
            cv::Size vs = cv::getTextSize(vbuf, cv::FONT_HERSHEY_DUPLEX, valScale, 1, &valBl);
            cv::putText(frame, vbuf, {contentX + (contentW - vs.width)/2, spinTop + spinH/2 + 10},  
                        cv::FONT_HERSHEY_DUPLEX, valScale, cv::Scalar(220, 255, 200), 1, cv::LINE_AA);
        }
        cv::line(frame, {MARGIN, top+panelH-2}, {W-MARGIN, top+panelH-2}, cv::Scalar(20,35,55), 1, cv::LINE_AA);
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
        snprintf(buf, sizeof(buf), "sudo timedatectl set-time '%04d-%02d-%02d %02d:%02d:00' 2>/dev/null",
                 editYear, editMon, editDay, editHour, editMin);
        int ret = std::system(buf);
        if (ret == 0) setFeedback("Fecha y hora aplicadas correctamente");
        else setFeedback("Error: verifica permisos sudo NOPASSWD para timedatectl");
    }

    void saveAudioConfig() {
        {
            FILE* f = fopen("/tmp/yaren_mic_device.txt", "w");
            if (f) { fprintf(f, "%s\n", selectedMicId.c_str()); fclose(f); }
        }
        {
            FILE* f = fopen("/tmp/yaren_spk_device.txt", "w");
            if (f) { fprintf(f, "%s\n", selectedSpkId.c_str()); fclose(f); }
        }
        setFeedback("Audio guardado: MIC=" + selectedMicId + "  SPK=" + selectedSpkId);
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

    void init(const std::string& pkg, rclcpp::Logger log) {
        pkgDir = pkg;
        logger_ = log;
        // Después de inicializar altFaces y songs:
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
                RCLCPP_INFO(logger_, "Cargada cara alternativa: %s", path.c_str());
            } else {
                RCLCPP_WARN(logger_, "No se pudo cargar: %s", path.c_str());
            }
        }
        
        if (!altFaces.empty()) {
            RCLCPP_INFO(logger_, "%d caras alternativas disponibles para el modo Radio", (int)altFaces.size());
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
        RCLCPP_INFO(logger_, "%d canciones cargadas en Yaren Radio", (int)songs.size());

        if (SDL_Init(SDL_INIT_AUDIO) < 0) {
            RCLCPP_ERROR(logger_, "No se pudo inicializar SDL: %s", SDL_GetError());
        } else if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 2048) < 0) {
            RCLCPP_ERROR(logger_, "No se pudo inicializar SDL_mixer: %s", Mix_GetError());
            SDL_Quit();
        } else {
            audioInitialized = true;
            RCLCPP_INFO(logger_, "SDL2_mixer inicializado correctamente");
        }


        lastBlinkTime   = std::chrono::steady_clock::now();
        lastFaceSwitch  = std::chrono::steady_clock::now();
        altFaceStartTime = std::chrono::steady_clock::now();
        
        showingAltFace      = false;
        currentAltFaceIndex = 0;
        isBlinking          = false;
        currentSongIndex    = -1;
        
        RCLCPP_INFO(logger_, "RadioApp inicializada correctamente");
    }

    // Destructor para limpiar recursos de SDL2
    ~RadioApp() {
        killAudio();
        if (audioInitialized) {
            Mix_CloseAudio();
            SDL_Quit();
            RCLCPP_INFO(logger_, "SDL2_mixer cerrado correctamente");
        }
    }

    void killAudio() {
        if (audioInitialized && Mix_PlayingMusic()) {
            Mix_FadeOutMusic(300);  // Fade out de 300ms para suavizar
            SDL_Delay(350);          // Esperar a que termine el fade
            Mix_HaltMusic();
        }
        if (currentMusic) {
            Mix_FreeMusic(currentMusic);
            currentMusic = nullptr;
        }
        currentSongIndex = -1;
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
            RCLCPP_WARN(logger_, "Audio no encontrado: %s", audioPath.c_str());
            state = RadioState::SELECTING;
            currentSong = -1;
            currentSongIndex = -1;
            return;
        }
        
        if (audioInitialized) {
            currentMusic = Mix_LoadMUS(audioPath.c_str());
            if (!currentMusic) {
                RCLCPP_ERROR(logger_, "No se pudo cargar el audio: %s", Mix_GetError());
                state = RadioState::SELECTING;
                currentSong = -1;
                currentSongIndex = -1;
                return;
            }
            
            if (Mix_FadeInMusic(currentMusic, -1, 500) < 0) {
                RCLCPP_ERROR(logger_, "No se pudo reproducir: %s", Mix_GetError());
                Mix_FreeMusic(currentMusic);
                currentMusic = nullptr;
                state = RadioState::SELECTING;
                currentSong = -1;
                currentSongIndex = -1;
                return;
            }
            
            RCLCPP_INFO(logger_, "Reproduciendo con SDL2: %s", songs[idx].title.c_str());
        } else {
            RCLCPP_WARN(logger_, "SDL2 no inicializado, usando fallback");
            std::string cmd = "mpg123 -q \"" + audioPath + "\" &";
            std::thread([cmd]() { std::system(cmd.c_str()); }).detach();
        }
    }

    void handleMouse(int ev, int x, int y) {
        mousePos = {x, y, 1, 1};

        if (ev == cv::EVENT_MOUSEMOVE) {
            hovSong = -1;
            for (int i = 0; i < (int)cardRects.size(); ++i)
                if (cardRects[i].contains({x,y})) { hovSong = i; break; }
            hovStop = stopBtn.area() > 0 && stopBtn.contains({x,y});
            hovBack = backBtn.area() > 0 && backBtn.contains({x,y});
            return;
        }

        if (ev != cv::EVENT_LBUTTONDOWN) return;

        if (hovBack) {
            killAudio();
            if (onBack) onBack();
            return;
        }
        if (hovStop && state == RadioState::PLAYING) {
            killAudio();
            state = RadioState::SELECTING;
            currentSong = -1;
            currentSongIndex = -1;
            return;
        }
        if (state == RadioState::SELECTING) {
            for (int i = 0; i < (int)cardRects.size(); ++i)
                if (cardRects[i].contains({x,y})) { playSong(i); return; }
        }
    }

    void render(cv::Mat& frame) {
        if (state == RadioState::SELECTING) renderSelector(frame);
        else renderNowPlaying(frame);
    }

private:
    std::string pkgDir;
    rclcpp::Logger logger_ = rclcpp::get_logger("RadioApp");
    
    //Variables para caras alternativas:
    std::vector<cv::Mat> altFaces;
    size_t currentAltFaceIndex { 0 };
    bool showingAltFace { false };
    int currentFaceIndex { 0 };
    std::chrono::steady_clock::time_point altFaceStartTime;
    std::chrono::steady_clock::time_point lastFaceSwitch;
    double faceSwitchInterval { 2.0 };
    
    // 🎵 Variables para SDL2_mixer:
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
        cv::putText(frame, "", {W/2-15, 54}, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200,60,255), 1, cv::LINE_AA);
        cv::line(frame, {W/2+20, 50}, {W/2+240, 50}, cv::Scalar(120, 30, 160), 1, cv::LINE_AA);
        drawCenteredText(frame, "Elige una cancion", W, 68, cv::FONT_HERSHEY_PLAIN, 0.90, cv::Scalar(100, 50, 130), 1);

        const int COLS = 3;
        const int N    = (int)songs.size();
        const int CW   = 220, CH = 128, G = 12;
        const int SY   = 80;

        cardRects.clear();
        for (int i = 0; i < N; ++i) {
            int row      = i / COLS;
            int col      = i % COLS;
            int rowItems = std::min(COLS, N - row * COLS);
            int rowW     = rowItems * CW + (rowItems - 1) * G;
            int rowSX    = (W - rowW) / 2;
            cv::Rect r{rowSX + col*(CW+G), SY + row*(CH+G), CW, CH};
            cardRects.push_back(r);
            drawSongCard(frame, r, songs[i], hovSong == i, t);
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
        
        // 🔄 Lógica de rotación cíclica de caras
        double elapsed = std::chrono::duration<double>(now - lastFaceSwitch).count();
        
        // Cambiar de cara cada X segundos
        if (elapsed > faceSwitchInterval) {
            currentFaceIndex = (currentFaceIndex + 1) % (altFaces.size() + 1);  // +1 para incluir la original
            lastFaceSwitch = now;
            RCLCPP_DEBUG(logger_, "🔄 Cambiando a cara índice %d", currentFaceIndex);
        }
        
        // ─────────────────────────────────────────
        // CASO 0: Cara ORIGINAL (ojos + boca separados)
        // ─────────────────────────────────────────
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
            const cv::Mat& mouthSrc = mouthClosed;  // Boca siempre cerrada

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

            // Rectángulo negro de fondo
            int rectW = fW + 40;
            int rectH = fH + 40;
            int rectX = fx - 20;
            int rectY = fy - 20;
            
            cv::rectangle(frame, {rectX, rectY, rectW, rectH}, cv::Scalar(0, 0, 0), cv::FILLED);
            cv::rectangle(frame, {rectX, rectY, rectW, rectH}, cv::Scalar(40, 40, 40), 2, cv::LINE_AA);

            // Dibujar cara
            cv::Rect dst(fx, fy, fW, fH);
            cv::Rect bounds(0, 0, W, H);
            dst &= bounds;
            if (dst.area() > 0) {
                cv::Rect src(dst.x - fx, dst.y - fy, dst.width, dst.height);
                if (src.x >= 0 && src.y >= 0 && src.x + src.width <= faceRot.cols && src.y + src.height <= faceRot.rows)
                    faceRot(src).copyTo(frame(dst));
            }

            // Aura
            double auraA = 0.08 * std::max(0.0, std::sin(t * beat * CV_PI));
            if (auraA > 0.01) {
                cv::Mat aov = frame.clone();
                cv::rectangle(aov, {rectX - 10, rectY - 10, rectW + 20, rectH + 20}, s.color, 3, cv::LINE_AA);
                cv::addWeighted(aov, auraA, frame, 1-auraA, 0, frame);
            }
            return;
        }
        
        // ─────────────────────────────────────────
        // CASO 1-4: Caras ALTERNATIVAS (money, open, ready, tongue)
        // ─────────────────────────────────────────
        int altIdx = currentFaceIndex - 1;  // Convertir índice: 1→0, 2→1, 3→2, 4→3
        if (altIdx >= 0 && altIdx < (int)altFaces.size()) {
            float alpha = 1.0f;  // Sin fade, transición directa
            
            double bounceY = std::sin(t * beat * CV_PI) * 8.0;
            double bounceX = std::cos(t * beat * CV_PI * 0.5) * 4.0;
            
            int fW = W * 60 / 100;
            int fH = (int)((float)fW * altFaces[altIdx].rows / altFaces[altIdx].cols);
            
            cv::Mat faceResized;
            cv::resize(altFaces[altIdx], faceResized, {fW, fH}, 0, 0, cv::INTER_LINEAR);
            
            int fx = (W - fW)/2 + (int)bounceX;
            int fy = (H - fH)/2 - 10 + (int)bounceY;
            
            // Rectángulo negro de fondo (DIBUJADO PRIMERO)
            int rectW = fW + 40;
            int rectH = fH + 40;
            int rectX = fx - 20;
            int rectY = fy - 20;
            
            cv::rectangle(frame, {rectX, rectY, rectW, rectH}, cv::Scalar(0, 0, 0), cv::FILLED);
            cv::rectangle(frame, {rectX, rectY, rectW, rectH}, cv::Scalar(40, 40, 40), 2, cv::LINE_AA);
            
            // Dibujar cara alternativa SOBRE el rectángulo
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
            
            // Aura pulsante
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
        loadIcon("accesorios", "filtro.png"); loadIcon("settings", "settings.png");

        ttsActive = false; isBlinking = false; hoveredItem = -1;
        hoveredBack = hoveredStop = hoveredExit = false;
        activeMode = ""; activeStopCmd = ""; running = true;
        showSettings = false; showRadio = false;
        lastBlinkTime = std::chrono::system_clock::now();

        faceOverlay = FaceOverlay::NONE; overlayMessage = ""; micCountdownSecs = 0;

        settingsMenu.onBack = [this]() { showSettings = false; };

        radioApp.init(pkgDir, this->get_logger());
        radioApp.onBack = [this]() { showRadio = false; };

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
                } else { activeMode = msg->data; }
            });

        cv::namedWindow("Yaren Face", cv::WINDOW_NORMAL);
        cv::setWindowProperty("Yaren Face", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::moveWindow("Yaren Face", 0, 0);
        cv::setMouseCallback("Yaren Face", VideoSynchronizer::mouseCallbackStatic, this);

        buildMenus();

        renderThread = std::thread(&VideoSynchronizer::renderLoop, this);
        RCLCPP_INFO(get_logger(), "face_screen listo con Radio Interna integrada (SDL2_mixer).");
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

                std::lock_guard<std::mutex> nlock(navMutex);
                if (!navStack.empty()) {
                    if (navStack.size() > 1) navStack.pop_back(); else navStack.clear();
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
            MI("yaren_rutina3", "RUTINA 3", "movimiento libre", {190,100,180}, "python3 src/YAREN2/yaren_movements/yaren_movements/yaren_movement.py &", "for pid in $(ps aux | grep -E 'yaren_movement' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "rutina3"),
            MI("yaren_rutinanueva", "NUEVA RUTINA", "imita y graba", {130, 80, 255}, "python3 src/YAREN2/yaren_movements/yaren_movements/yaren_rutinanueva.py &", "for pid in $(ps aux | grep -E 'yaren_movement' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "rutina_nueva"),

        }};

        subMenuMap["sub_yaren_filtros"] = { "FILTROS", {105,240,174}, {
            MI("yaren_animales", "ANIMALES", "filtro animal", {105,240,174}, "ros2 launch yaren_filters yaren_animales.launch.py &", "for pid in $(ps aux | grep -E 'yaren_animales|AnimalFaceNode|animal_filter|face_landmark|csi_cam' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "animales"),
            MI("yaren_accesorios", "ACCESORIOS", "filtro accesorios", {60,200,130}, "ros2 launch yaren_filters yaren_accesorios.launch.py &", "for pid in $(ps aux | grep -E 'yaren_accesorios|face_filter|face_landmark|csi_cam' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "accesorios"),
        }};

        subMenuMap["sub_yaren_p2"] = { "YAREN", {0,229,255}, {
            MI("yaren_radio", "YAREN RADIO", "musica y animacion", {255,80,160}, "", "", true, "sub_yaren_radio", "yaren"),
        }};
        subMenuMap["sub_yaren_p2"].key = "sub_yaren_p2";

        subMenuMap["sub_yaren_radio"] = { "YAREN RADIO", {255,80,160}, {
            MI("radio_musica", "MUSICA", "reproducir musica", {255,120,200}, "INTERNAL_RADIO", "", false, "", "rutina1"),
            MI("radio_videos", "VIDEOS", "reproducir videos", {200,60,140}, "", "", true, "sub_yaren_videos", "rutina2"),
        }};
        subMenuMap["sub_yaren_radio"].key = "sub_yaren_radio";  

        subMenuMap["sub_yaren_videos"] = { "VIDEOS", {200,60,140}, {
            MI("vid_pollito", "POLLITO PIO", "Canciones de la Granja", {0, 200, 255}, "python3 src/YAREN2/yaren_radio/yaren_radio/pollitopio.py &", "for pid in $(ps aux | grep -E 'pollitopio.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "rutina1"),
            MI("vid_gallina", "GALLINA TURULECA", "Canciones de Yaren", {255, 150, 50}, "python3 src/YAREN2/yaren_radio/yaren_radio/gallinaturuleca.py &", "for pid in $(ps aux | grep -E 'gallinaturuleca.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "rutina2"),
            MI("vid_vaca", "LA VACA LOLA", "Canciones Infantiles", {100, 255, 100}, "python3 src/YAREN2/yaren_radio/yaren_radio/vacalola.py &", "for pid in $(ps aux | grep -E 'vacalola.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "rutina3"),
            MI("vid_susanita", "SUSANITA", "La Granja de Zenon", {255, 100, 200}, "python3 src/YAREN2/yaren_radio/yaren_radio/susanita.py &", "for pid in $(ps aux | grep -E 'susanita.py' | grep -v grep | awk '{print $2}'); do kill -9 $pid; done", false, "", "rutina4"),
        }};
    }

    bool runCommand(const std::string& cmd) {
        if (cmd.empty()) return false;
        int ret = std::system(cmd.c_str());
        return (ret == 0);
    }

    void showErrorOverlay(const std::string& msg, double secs = 3.0) {
        {
            std::lock_guard<std::mutex> lock(overlayMutex);
            faceOverlay    = FaceOverlay::ERROR_MSG;
            overlayMessage = msg;
        }
        std::this_thread::sleep_for(std::chrono::duration<double>(secs));
        {
            std::lock_guard<std::mutex> lock(overlayMutex);
            faceOverlay    = FaceOverlay::NONE;
            overlayMessage = "";
        }
    }

    void executeMicTest() {
        if (micTestRunning.load()) return;
        micTestRunning = true;

        navStack.clear();
        hoveredItem = -1;
        hoveredBack = hoveredStop = hoveredExit = false;

        if (testThread.joinable()) testThread.join();
        testThread = std::thread([this]() {
            std::string micId = settingsMenu.selectedMicId.empty() ? "plughw:0,0" : settingsMenu.selectedMicId;
            std::string spkId = "default"; 

            const std::string recordCmd = "arecord -D " + micId + " -f S16_LE -r 44100 -c 1 -d 5 /tmp/yaren_mic_test.wav 2>/tmp/yaren_arecord.log";

            RCLCPP_INFO(get_logger(), "[MIC TEST] Iniciando grabacion con: %s", micId.c_str());

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
                showErrorOverlay("No se ha podido realizar\nel comando.", 4.0);
                micTestRunning = false;
                return;
            }

            {
                std::lock_guard<std::mutex> lock(overlayMutex);
                faceOverlay    = FaceOverlay::NONE;
                overlayMessage = "";
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
            {
                std::lock_guard<std::mutex> lock(overlayMutex);
                faceOverlay    = FaceOverlay::MIC_PLAYING;
                overlayMessage = "Reproduciendo audio...";
            }

            const std::string playCmd = "aplay -D " + spkId + " /tmp/yaren_mic_test.wav 2>/tmp/yaren_aplay.log";
            int playRet = std::system(playCmd.c_str());

            if (playRet != 0) {
                RCLCPP_ERROR(get_logger(), "[MIC TEST] Fallo reproduccion (exit %d)", playRet);
                showErrorOverlay("No se ha podido realizar\nel comando.", 4.0);
            }

            {
                std::lock_guard<std::mutex> lock(overlayMutex);
                faceOverlay      = FaceOverlay::NONE;
                overlayMessage   = "";
                micCountdownSecs = 0;
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
                return;
            }
            if (navStack.size() > 1 && backButtonRect.contains({ x, y })) {
                navStack.pop_back(); hoveredItem = -1; hoveredBack = false;
                return;
            }
            if (!activeMode.empty() && stopButtonRect.contains({ x, y })) {
                if (!activeStopCmd.empty()) std::system(activeStopCmd.c_str());
                activeMode = ""; activeStopCmd = "";
                navStack.clear(); hoveredItem = -1; hoveredBack = hoveredStop = hoveredExit = false;
                auto msg = std_msgs::msg::String(); msg.data = "idle";
                modePublisher->publish(msg);
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

        navStack.clear();
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
            std::thread([this, cleanCmd]() {
                auto start = std::chrono::steady_clock::now();
                int ret = std::system(cleanCmd.c_str());
                auto end = std::chrono::steady_clock::now();
                double elapsedSecs = std::chrono::duration<double>(end - start).count();
                if (ret != 0 && elapsedSecs < 1.5) {
                    RCLCPP_ERROR(get_logger(), "[CMD] Fallo inmediato (Exit %d): %s", ret, cleanCmd.c_str());
                    showErrorOverlay("No se ha podido realizar\nel comando.", 4.0);
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

        int cx = settingsButtonRect.x + btnSz/2, cy = settingsButtonRect.y + btnSz/2;
        cv::Scalar ic = hoveredSettings ? cv::Scalar(0,220,255) : cv::Scalar(80,130,160);
        cv::circle(frame, {cx,cy}, 8, ic, 1, cv::LINE_AA);
        cv::circle(frame, {cx,cy}, 4, ic, cv::FILLED, cv::LINE_AA);
        for (int d = 0; d < 8; ++d) {
            double ang = d * CV_PI / 4.0;
            cv::line(frame, {(int)(cx + 8*std::cos(ang)), (int)(cy + 8*std::sin(ang))}, 
                            {(int)(cx + 13*std::cos(ang)), (int)(cy + 13*std::sin(ang))}, ic, 2, cv::LINE_AA);
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
                renderSideNavArrow(frame, rightNavArrowRect, false, "RADIO", hoveredRightNav);
            }
            if (curKey == "sub_yaren_p2") {
                leftNavArrowRect = { 10, H/2 - 70, 48, 130 };
                renderSideNavArrow(frame, leftNavArrowRect, true, "YAREN", hoveredLeftNav);
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

    void audioPlayingCallback(const std_msgs::msg::Bool::SharedPtr msg) { ttsActive = msg->data; }

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