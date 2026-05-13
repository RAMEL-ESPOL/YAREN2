#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaren_interfaces/msg/landmarks.hpp"
#include "filters/GlassesFilter.hpp"
#include "filters/MouthFilter.hpp"
#include "filters/NoseFilter.hpp"
#include "filters/HatFilter.hpp"
#include "filters/FaceMaskFilter.hpp"
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <thread>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace message_filters;
using ImageMsg    = sensor_msgs::msg::Image;
using Landmarks   = yaren_interfaces::msg::Landmarks;
using ApproximateTimePolicy =
    sync_policies::ApproximateTime<ImageMsg, Landmarks>;

// ================================================================
//  CONSTANTES DE LAYOUT  (800 x 480)
// ================================================================
static const int  WIN_W       = 800;
static const int  WIN_H       = 480;
static const char MENU_WIN[]  = "Selecciona tus accesorios";
static const char CAM_WIN[]   = "Face Filter";

// Botones de accion
static const cv::Rect BTN_APPLY(270, 430, 160, 44);
static const cv::Rect BTN_CLOSE(WIN_W - 55, 8, 45, 45);

// ================================================================
//  CATEGORIAS DE FILTROS
// ================================================================
struct Category {
    std::string id;
    std::string label;
    std::string icon_char;   // emoji-like text
    cv::Scalar  color;       // BGR acento
    std::string assets_subdir;
};

static const std::vector<Category> CATS = {
    { "hat",      "Sombrero", "HAT",  { 50,  180, 255 }, "hats"    },
    { "glasses",  "Gafas",    "GLASS",{ 200, 100,  50 }, "glasses" },
    { "nose",     "Nariz",    "NOSE", {  60, 200, 100 }, "noses"   },
    { "mouth",    "Boca",     "MOUTH",{ 100,  60, 220 }, "mouths"  },
    { "mask",     "Mascara",  "MASK", { 180,  30, 180 }, "faces"   },
};

// Estado por categoria: indice seleccionado (-1 = desactivado)
struct CatState {
    int  index     = -1;   // -1 = off
    int  max_idx   =  0;   // cuantos assets hay
    bool enabled   = false;
    std::vector<cv::Mat> previews;  // miniaturas para el menu
};

// ================================================================
//  HELPERS DE DIBUJO
// ================================================================
static void overlay_rgba_menu(cv::Mat& bg, const cv::Mat& fg, int ox, int oy)
{
    if (fg.empty()) return;
    int y1 = std::max(oy, 0),        y2 = std::min(oy + fg.rows, bg.rows);
    int x1 = std::max(ox, 0),        x2 = std::min(ox + fg.cols, bg.cols);
    if (x1 >= x2 || y1 >= y2) return;
    cv::Mat fgc = fg(cv::Rect(x1-ox, y1-oy, x2-x1, y2-y1));
    cv::Mat bgc = bg(cv::Rect(x1,    y1,     x2-x1, y2-y1));
    std::vector<cv::Mat> ch; cv::split(fgc, ch);
    bool alpha = (ch.size() == 4);
    cv::Mat a32, inv;
    if (alpha) { ch[3].convertTo(a32, CV_32F, 1./255.); inv = 1.f - a32; }
    cv::Mat fg_bgr;
    if (fgc.channels() == 4) cv::cvtColor(fgc, fg_bgr, cv::COLOR_BGRA2BGR);
    else fg_bgr = fgc;
    cv::Mat bf; bgc.convertTo(bf, CV_32FC3, 1./255.);
    cv::Mat ff; fg_bgr.convertTo(ff, CV_32FC3, 1./255.);
    cv::Mat out;
    if (alpha) {
        std::vector<cv::Mat> bc, fc, r(3);
        cv::split(bf, bc); cv::split(ff, fc);
        for (int c = 0; c < 3; ++c) r[c] = fc[c].mul(a32) + bc[c].mul(inv);
        cv::merge(r, out);
    } else { out = ff; }
    cv::Mat o8; out.convertTo(o8, CV_8UC3, 255.);
    o8.copyTo(bgc);
}

// Carga miniaturas de una carpeta
static std::vector<cv::Mat> load_thumbs(const std::string& dir, int tw, int th)
{
    std::vector<cv::Mat> out;
    try {
        namespace fs = std::filesystem;
        if (!fs::exists(dir)) return out;
        std::vector<std::string> files;
        for (auto& e : fs::directory_iterator(dir))
            if (e.path().extension() == ".png" || e.path().extension() == ".jpg")
                files.push_back(e.path().string());
        std::sort(files.begin(), files.end());
        for (auto& f : files) {
            cv::Mat img = cv::imread(f, cv::IMREAD_UNCHANGED);
            if (img.empty()) continue;
            // recortar transparencia
            if (img.channels() == 4) {
                std::vector<cv::Mat> ch; cv::split(img, ch);
                cv::Rect bb = cv::boundingRect(ch[3]);
                if (bb.area() > 0) img = img(bb).clone();
            }
            // escalar proporcional
            double s = std::min((double)tw/img.cols, (double)th/img.rows);
            cv::Mat thumb;
            cv::resize(img, thumb, cv::Size((int)(img.cols*s), (int)(img.rows*s)), 0, 0, cv::INTER_AREA);
            out.push_back(thumb);
        }
    } catch (...) {}
    return out;
}

// ================================================================
//  MENU STATE
// ================================================================
struct MenuMouseState {
    int  hovered_cat   = -1;
    int  hovered_arrow = -1;
    bool apply_hit     = false;
    bool close_hit     = false;
    
    // Variables para el scroll
    int  scroll_y      = 0;
    int  max_scroll_y  = 0;

    // Variables para arrastrar con el clic
    bool is_dragging   = false;
    int  last_mouse_y  = 0;
};
// ================================================================
//  DIBUJO DEL MENU
// ================================================================
//  Layout: 5 filas (una por categoria), cabecera, boton aplicar/X
//
//  Cada fila:
//   [ON/OFF]  [Label]  [< idx/max >]  [preview thumbnail]
// ================================================================

// Rectangulos interactivos (se calculan en draw_menu y se guardan aqui)
struct RowRects {
    cv::Rect toggle;
    cv::Rect prev;
    cv::Rect next;
    cv::Rect row;
};
static std::vector<RowRects> g_row_rects;

// --- COLORES DEL TEMA OSCURO (Formato BGR para OpenCV) ---
const cv::Scalar BG_MAIN(20, 13, 13);        // #0d0d14
const cv::Scalar BG_HEADER(31, 19, 19);      // #13131f
const cv::Scalar BORDER_COLOR(58, 42, 42);   // #2a2a3a
const cv::Scalar TEXT_LIGHT(240, 226, 226);  // #e2e2f0
const cv::Scalar TEXT_MUTED(160, 124, 124);  // #7c7ca0
const cv::Scalar COLOR_PURPLE(250, 139, 167); // #a78bfa (Sombrero)
const cv::Scalar COLOR_GREEN(128, 222, 74);   // #4ade80 (Boca)

struct UIRects {
    cv::Rect toggle[5];
    cv::Rect prev[5];
    cv::Rect next[5];
    cv::Rect apply;
    cv::Rect close;
};
static UIRects g_ui; // Hitboxes globales para el mouse

// Helper para dibujar rectángulos con esquinas redondeadas
static void drawRoundedRect(cv::Mat& img, cv::Rect rec, cv::Scalar color, int radius, int thickness = -1) {
    if (radius < 0) radius = 0;
    radius = std::min({radius, rec.width / 2, rec.height / 2});
    
    if (thickness < 0) { // Relleno
        cv::rectangle(img, cv::Point(rec.x + radius, rec.y), cv::Point(rec.x + rec.width - radius, rec.y + rec.height), color, -1, cv::LINE_AA);
        cv::rectangle(img, cv::Point(rec.x, rec.y + radius), cv::Point(rec.x + rec.width, rec.y + rec.height - radius), color, -1, cv::LINE_AA);
        cv::circle(img, cv::Point(rec.x + radius, rec.y + radius), radius, color, -1, cv::LINE_AA);
        cv::circle(img, cv::Point(rec.x + rec.width - radius, rec.y + radius), radius, color, -1, cv::LINE_AA);
        cv::circle(img, cv::Point(rec.x + radius, rec.y + rec.height - radius), radius, color, -1, cv::LINE_AA);
        cv::circle(img, cv::Point(rec.x + rec.width - radius, rec.y + rec.height - radius), radius, color, -1, cv::LINE_AA);
    } else { // Borde
        cv::line(img, cv::Point(rec.x + radius, rec.y), cv::Point(rec.x + rec.width - radius, rec.y), color, thickness, cv::LINE_AA);
        cv::line(img, cv::Point(rec.x + radius, rec.y + rec.height), cv::Point(rec.x + rec.width - radius, rec.y + rec.height), color, thickness, cv::LINE_AA);
        cv::line(img, cv::Point(rec.x, rec.y + radius), cv::Point(rec.x, rec.y + rec.height - radius), color, thickness, cv::LINE_AA);
        cv::line(img, cv::Point(rec.x + rec.width, rec.y + radius), cv::Point(rec.x + rec.width, rec.y + rec.height - radius), color, thickness, cv::LINE_AA);
        cv::ellipse(img, cv::Point(rec.x + radius, rec.y + radius), cv::Size(radius, radius), 180, 0, 90, color, thickness, cv::LINE_AA);
        cv::ellipse(img, cv::Point(rec.x + rec.width - radius, rec.y + radius), cv::Size(radius, radius), 270, 0, 90, color, thickness, cv::LINE_AA);
        cv::ellipse(img, cv::Point(rec.x + rec.width - radius, rec.y + rec.height - radius), cv::Size(radius, radius), 0, 0, 90, color, thickness, cv::LINE_AA);
        cv::ellipse(img, cv::Point(rec.x + radius, rec.y + rec.height - radius), cv::Size(radius, radius), 90, 0, 90, color, thickness, cv::LINE_AA);
    }
}
static void draw_menu(cv::Mat& canvas, const std::vector<CatState>& states, MenuMouseState* ms)
{
    canvas.setTo(BG_MAIN);

    // ── Colores del tema (BGR) ──
    const cv::Scalar BG_CARD(46, 26, 26);
    const cv::Scalar CAT_HEADER_BG(30, 14, 30);
    const cv::Scalar PILL_PURPLE_BG(80, 36, 80);
    const cv::Scalar PILL_GREEN_BG(24, 36, 21);
    const cv::Scalar APPLY_GREEN(52, 101, 22);

    // ── Ícono por categoría (texto corto) y colores de acento ──
    static const char*      CAT_ICO[]  = { "HAT", "EYE", "NSE", "MCH", "MSK" };
    static const cv::Scalar ICON_BG[]  = {
        {30,  60, 160},  // Sombrero → azul
        {20, 120, 200},  // Gafas    → celeste
        {40, 160,  80},  // Nariz    → verde
        {160, 40, 120},  // Boca     → rosa
        {100,  30, 180}  // Mascara  → morado
    };
    static const cv::Scalar ACCENT[]   = {
        {30,  60, 160},  // Sombrero → azul
        {20, 120, 200},  // Gafas    → celeste
        {40, 160,  80},  // Nariz    → verde
        {160, 40, 120},  // Boca     → rosa
        {100,  30, 180}  // Mascara  → morado
    };
    static const cv::Scalar ROW_BG[]   = {
        {30,  60, 160},  // Sombrero → azul
        {20, 120, 200},  // Gafas    → celeste
        {40, 160,  80},  // Nariz    → verde
        {160, 40, 120},  // Boca     → rosa
        {100,  30, 180}  // Mascara  → morado
    };
    static const cv::Scalar ROW_BDR[]  = {
        {30,  60, 160},  // Sombrero → azul
        {20, 120, 200},  // Gafas    → celeste
        {40, 160,  80},  // Nariz    → verde
        {160, 40, 120},  // Boca     → rosa
        {100,  30, 180}  // Mascara  → morado
    };

    // ════════════════════════════════════════
    //  1. CABECERA
    // ════════════════════════════════════════
    cv::rectangle(canvas, cv::Rect(0,0,WIN_W,55), BG_HEADER, cv::FILLED);
    cv::line(canvas, {0,55}, {WIN_W,55}, BORDER_COLOR, 1);

    // Pequeño cuadro acento antes del título
    drawRoundedRect(canvas, cv::Rect(18,17,20,20), cv::Scalar(74,37,83), 4);
    cv::putText(canvas, "", {22,32}, cv::FONT_HERSHEY_SIMPLEX, 0.45, COLOR_PURPLE, 1, cv::LINE_AA);
    cv::putText(canvas, "Configura tus accesorios", {46,35},
                cv::FONT_HERSHEY_SIMPLEX, 0.6, TEXT_LIGHT, 1, cv::LINE_AA);

    // Botón X
    g_ui.close = cv::Rect(WIN_W-45, 12, 32, 32);
    drawRoundedRect(canvas, g_ui.close, cv::Scalar(26,26,58), 8);
    cv::line(canvas, {g_ui.close.x+10,g_ui.close.y+10},
                     {g_ui.close.x+22,g_ui.close.y+22}, cv::Scalar(113,113,248), 2, cv::LINE_AA);
    cv::line(canvas, {g_ui.close.x+22,g_ui.close.y+10},
                     {g_ui.close.x+10,g_ui.close.y+22}, cv::Scalar(113,113,248), 2, cv::LINE_AA);

    // ════════════════════════════════════════
    //  2. PANEL IZQUIERDO – categorías
    // ════════════════════════════════════════
    const int LEFT_W = 380;
    cv::line(canvas, {LEFT_W,55}, {LEFT_W,WIN_H-55}, BORDER_COLOR, 1);

    // Sub-cabecera "CATEGORIAS"
    cv::rectangle(canvas, cv::Rect(0,55,LEFT_W,28), CAT_HEADER_BG, cv::FILLED);
    cv::line(canvas, {0,83}, {LEFT_W,83}, cv::Scalar(30,30,46), 1);
    cv::putText(canvas, "CATEGORIAS", {14,73},
                cv::FONT_HERSHEY_SIMPLEX, 0.33, TEXT_MUTED, 1, cv::LINE_AA);

    int row_y = 90;
    for (int i = 0; i < (int)CATS.size(); ++i) {
        const auto& cat = CATS[i];
        const auto& st  = states[i];

        g_ui.toggle[i] = cv::Rect(8, row_y, LEFT_W-16, 54);

        if (st.enabled) {
            drawRoundedRect(canvas, g_ui.toggle[i], ROW_BG[i],  8);
            drawRoundedRect(canvas, g_ui.toggle[i], ROW_BDR[i], 8, 1);

            // Icon box activo
            cv::Rect ib(g_ui.toggle[i].x+10, row_y+10, 34, 34);
            drawRoundedRect(canvas, ib, ICON_BG[i], 8);
            cv::putText(canvas, CAT_ICO[i], {ib.x+2,ib.y+22},
                        cv::FONT_HERSHEY_SIMPLEX, 0.28, ACCENT[i], 1, cv::LINE_AA);

            // Texto
            cv::putText(canvas, cat.label, {g_ui.toggle[i].x+54, row_y+24},
                        cv::FONT_HERSHEY_SIMPLEX, 0.48, TEXT_LIGHT, 1, cv::LINE_AA);
            char buf[32];
            snprintf(buf, sizeof(buf), "%d de %d", st.index+1, st.max_idx);
            cv::putText(canvas, buf, {g_ui.toggle[i].x+54, row_y+40},
                        cv::FONT_HERSHEY_SIMPLEX, 0.36, TEXT_MUTED, 1, cv::LINE_AA);

            // Check circle con tilde
            cv::Point cc{g_ui.toggle[i].x + g_ui.toggle[i].width - 20, row_y+27};
            cv::circle(canvas, cc, 11, ROW_BDR[i], -1, cv::LINE_AA);
            cv::line(canvas, {cc.x-6, cc.y},   {cc.x-2, cc.y+5}, TEXT_LIGHT, 2, cv::LINE_AA);
            cv::line(canvas, {cc.x-2, cc.y+5}, {cc.x+6, cc.y-5}, TEXT_LIGHT, 2, cv::LINE_AA);
        } else {
            drawRoundedRect(canvas, g_ui.toggle[i], cv::Scalar(30,20,30), 8);

            // Icon box inactivo
            cv::Rect ib(g_ui.toggle[i].x+10, row_y+10, 34, 34);
            cv::Scalar ib_dim(ICON_BG[i][0]/2, ICON_BG[i][1]/2, ICON_BG[i][2]/2);
            drawRoundedRect(canvas, ib, ib_dim, 8);
            cv::putText(canvas, CAT_ICO[i], {ib.x+2,ib.y+22},
                        cv::FONT_HERSHEY_SIMPLEX, 0.28, TEXT_MUTED, 1, cv::LINE_AA);

            cv::putText(canvas, cat.label, {g_ui.toggle[i].x+54, row_y+24},
                        cv::FONT_HERSHEY_SIMPLEX, 0.48, TEXT_MUTED, 1, cv::LINE_AA);
            cv::putText(canvas, "desactivado", {g_ui.toggle[i].x+54, row_y+40},
                        cv::FONT_HERSHEY_SIMPLEX, 0.36, cv::Scalar(90,58,90), 1, cv::LINE_AA);
        }
        row_y += 59;
    }

    // ════════════════════════════════════════
    //  3. PANEL DERECHO – editores (con scroll)
    // ════════════════════════════════════════
    const int RX = LEFT_W + 16;
    const int RY = 65;
    const int RW = WIN_W - RX - 16;
    const int RH = (WIN_H - 55) - RY;

    cv::Rect right_view(RX, RY, RW, RH);
    cv::Mat  right_canvas(RH, RW, CV_8UC3, BG_MAIN);

    // Altura de cada tarjeta: título(20) + preview(100) + nav(44) + padding
    const int CARD_H  = 176;
    const int CARD_SP = 10;

    int active_count = 0;
    for (const auto& s : states) if (s.enabled) active_count++;
    int total_h = active_count * (CARD_H + CARD_SP);

    ms->max_scroll_y = std::max(0, total_h - RH);
    ms->scroll_y     = std::clamp(ms->scroll_y, 0, ms->max_scroll_y);

    int cur_y = -ms->scroll_y;

    for (int i = 0; i < (int)CATS.size(); ++i) {
        g_ui.prev[i] = g_ui.next[i] = cv::Rect(0,0,0,0);
        if (!states[i].enabled) continue;
        if (cur_y + CARD_H < 0 || cur_y > RH) { cur_y += CARD_H + CARD_SP; continue; }

        const auto& cat = CATS[i];
        const auto& st  = states[i];

        cv::Rect card(0, cur_y, RW-15, CARD_H);
        drawRoundedRect(right_canvas, card, cv::Scalar(26,14,26), 10);
        drawRoundedRect(right_canvas, card, BORDER_COLOR, 10, 1);

        // Título
        std::string title = cat.label + " activo";
        std::transform(title.begin(), title.end(), title.begin(), ::toupper);
        cv::putText(right_canvas, title, {card.x+12, card.y+18},
                    cv::FONT_HERSHEY_SIMPLEX, 0.33, TEXT_MUTED, 1, cv::LINE_AA);

        // Preview box (100px alto)
        cv::Rect pbox(card.x+12, card.y+24, card.width-24, 100);
        drawRoundedRect(right_canvas, pbox, cv::Scalar(19,19,31), 8);
        drawRoundedRect(right_canvas, pbox, BORDER_COLOR, 8, 1);

        if (st.index >= 0 && st.index < (int)st.previews.size() && !st.previews[st.index].empty()) {
            const cv::Mat& thumb = st.previews[st.index];
            double sc = std::min((double)(pbox.width-16)/thumb.cols, 82.0/thumb.rows);
            cv::Mat ts;
            cv::resize(thumb, ts, cv::Size((int)(thumb.cols*sc),(int)(thumb.rows*sc)),
                       0, 0, cv::INTER_AREA);
            overlay_rgba_menu(right_canvas, ts,
                              pbox.x+(pbox.width-ts.cols)/2,
                              pbox.y+(pbox.height-ts.rows)/2);
        }

        // Fila de navegación
        cv::Scalar btn_bg  = (i==3) ? cv::Scalar(24,34,21) : cv::Scalar(80,36,80);
        cv::Scalar arr_col = ACCENT[i];

        cv::Rect bprev(card.x+12,       card.y+134, 32, 32);
        cv::Rect bnext(card.x+card.width-44, card.y+134, 32, 32);

        drawRoundedRect(right_canvas, bprev, btn_bg, 8);
        drawRoundedRect(right_canvas, bnext, btn_bg, 8);
        cv::putText(right_canvas, "<", {bprev.x+10,bprev.y+22},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, arr_col, 2, cv::LINE_AA);
        cv::putText(right_canvas, ">", {bnext.x+10,bnext.y+22},
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, arr_col, 2, cv::LINE_AA);

        char ibuf[16]; snprintf(ibuf, sizeof(ibuf), "%d / %d", st.index+1, st.max_idx);
        int tw = cv::getTextSize(ibuf, cv::FONT_HERSHEY_SIMPLEX, 0.45, 1, nullptr).width;
        cv::putText(right_canvas, ibuf, {card.x+(card.width-tw)/2, card.y+154},
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(144,144,176), 1, cv::LINE_AA);

        // Hitboxes absolutas en pantalla
        g_ui.prev[i] = cv::Rect(RX+bprev.x, RY+bprev.y, bprev.width, bprev.height) & right_view;
        g_ui.next[i] = cv::Rect(RX+bnext.x, RY+bnext.y, bnext.width, bnext.height) & right_view;

        cur_y += CARD_H + CARD_SP;
    }

    // Scrollbar
    if (ms->max_scroll_y > 0) {
        float vr  = (float)RH / total_h;
        int   hh  = std::max(30, (int)(RH*vr));
        float sr  = (float)ms->scroll_y / ms->max_scroll_y;
        int   hy  = (int)(sr*(RH-hh));
        drawRoundedRect(right_canvas, cv::Rect(RW-7, 0, 6, RH), cv::Scalar(30,20,30), 3);
        drawRoundedRect(right_canvas, cv::Rect(RW-7, hy, 6, hh), cv::Scalar(80,60,80), 3);
    }

    right_canvas.copyTo(canvas(right_view));

    // ════════════════════════════════════════
    //  4. FOOTER – pills + Aplicar
    // ════════════════════════════════════════
    cv::rectangle(canvas, cv::Rect(0,WIN_H-55,WIN_W,55), BG_HEADER, cv::FILLED);
    cv::line(canvas, {0,WIN_H-55}, {WIN_W,WIN_H-55}, BORDER_COLOR, 1);

    // Pills de categorías activas
    int px = 14, py = WIN_H-55+13;
    for (int i = 0; i < (int)CATS.size(); ++i) {
        if (!states[i].enabled) continue;
        cv::Scalar pbg(ICON_BG[i][0]/3, ICON_BG[i][1]/3, ICON_BG[i][2]/3);
        cv::Scalar ptxt = ACCENT[i];
        int lw = cv::getTextSize(CATS[i].label, cv::FONT_HERSHEY_SIMPLEX, 0.33, 1, nullptr).width;
        cv::Rect pill(px, py, lw+26, 26);
        drawRoundedRect(canvas, pill, pbg, 6);
        cv::circle(canvas, {pill.x+10, pill.y+13}, 4, ptxt, -1, cv::LINE_AA);
        cv::putText(canvas, CATS[i].label, {pill.x+18, pill.y+18},
                    cv::FONT_HERSHEY_SIMPLEX, 0.33, ptxt, 1, cv::LINE_AA);
        px += pill.width + 6;
    }

    // Botón Aplicar (con triángulo play)
    g_ui.apply = cv::Rect(WIN_W-140, WIN_H-45, 120, 35);
    drawRoundedRect(canvas, g_ui.apply, APPLY_GREEN, 8);
    std::vector<cv::Point> tri = {
        {g_ui.apply.x+18, g_ui.apply.y+10},
        {g_ui.apply.x+18, g_ui.apply.y+25},
        {g_ui.apply.x+30, g_ui.apply.y+17}
    };
    cv::fillPoly(canvas, tri, cv::Scalar(255,255,255));
    cv::putText(canvas, "Aplicar", {g_ui.apply.x+36, g_ui.apply.y+23},
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255,255,255), 1, cv::LINE_AA);
}

// ================================================================
//  CONTEXTO DEL MOUSE (evita void*[] peligroso)
// ================================================================
struct MenuContext {
    MenuMouseState*         ms;
    std::vector<CatState>*  states;
};

// ================================================================
//  MOUSE CALLBACK DEL MENU
// ================================================================
static void on_menu_mouse(int event, int x, int y, int flags, void* ud)
{
    auto* ctx = reinterpret_cast<MenuContext*>(ud);
    auto* ms = ctx->ms;
    auto* states = ctx->states;

    // --- 1. SCROLL CON LA RUEDA (Si el sistema lo permite) ---
    if (event == cv::EVENT_MOUSEWHEEL) {
        int delta = cv::getMouseWheelDelta(flags);
        if (delta > 0) ms->scroll_y -= 40; 
        if (delta < 0) ms->scroll_y += 40; 
        ms->scroll_y = std::clamp(ms->scroll_y, 0, ms->max_scroll_y);
        return;
    }

    // --- 2. SCROLL ARRASTRANDO (Solución infalible) ---
    if (event == cv::EVENT_MOUSEMOVE) {
        if (ms->is_dragging) {
            int dy = y - ms->last_mouse_y;
            ms->scroll_y -= dy; // Desplazar según lo que se movió el ratón
            ms->scroll_y = std::clamp(ms->scroll_y, 0, ms->max_scroll_y);
            ms->last_mouse_y = y;
        }
        return;
    }

    // Al soltar el clic, dejamos de arrastrar
    if (event == cv::EVENT_LBUTTONUP) {
        ms->is_dragging = false;
        return;
    }

    if (event != cv::EVENT_LBUTTONDOWN) return;

    // --- Botones fijos primero ---
    if (g_ui.close.contains({x,y})) { ms->close_hit = true; return; }
    if (g_ui.apply.contains({x,y})) { ms->apply_hit = true; return; }

    for (int i = 0; i < (int)CATS.size(); ++i) {
        if (g_ui.toggle[i].contains({x,y})) {
            states->at(i).enabled = !states->at(i).enabled;
            if (states->at(i).enabled) {
                if (i == 4) {
                    for (int j = 0; j < 4; ++j) states->at(j).enabled = false;
                } else {
                    if (states->size() > 4) states->at(4).enabled = false;
                }
            }
            if (states->at(i).enabled && states->at(i).index < 0 && states->at(i).max_idx > 0)
                states->at(i).index = 0;
            return;
        }
        if (states->at(i).enabled) {
            if (g_ui.prev[i].contains({x,y})) {
                auto& s = states->at(i);
                s.index = (s.index - 1 + s.max_idx) % s.max_idx;
                return;
            }
            if (g_ui.next[i].contains({x,y})) {
                auto& s = states->at(i);
                s.index = (s.index + 1) % s.max_idx;
                return;
            }
        }
    }

    // --- Solo si ningún botón fue tocado: iniciar drag ---
    if (x > 380 && ms->max_scroll_y > 0) {
        ms->is_dragging = true;
        ms->last_mouse_y = y;
    }
}
// ================================================================
//  MOUSE CALLBACK CAMARA
// ================================================================
static void on_cam_mouse(int event, int, int, int, void* ud)
{
    if (event == cv::EVENT_LBUTTONDOWN)
        *reinterpret_cast<std::atomic<bool>*>(ud) = true;
}

// ================================================================
//  show_menu()  — bloquea hasta Aplicar o X
//  Retorna false si el usuario presionó X.
// ================================================================
static bool show_menu(std::vector<CatState>& states)
{
    cv::namedWindow(MENU_WIN, cv::WINDOW_NORMAL);
    cv::setWindowProperty(MENU_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    MenuMouseState ms; // Aquí se guarda el estado del scroll
    MenuContext ctx{ &ms, &states };
    cv::setMouseCallback(MENU_WIN, on_menu_mouse, &ctx);

    cv::Mat canvas(WIN_H, WIN_W, CV_8UC3);
    while (!ms.apply_hit && !ms.close_hit) {
        draw_menu(canvas, states, &ms);  // <-- AQUÍ AGREGAMOS &ms
        cv::imshow(MENU_WIN, canvas);
        cv::waitKey(16);
    }
    cv::destroyWindow(MENU_WIN);
    return ms.apply_hit;
}

// ================================================================
//  NODO PRINCIPAL
// ================================================================
class FaceFilterNode : public rclcpp::Node {
public:
    FaceFilterNode() : Node("face_filter_node"), running_(true)
    {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        std::string ap = ament_index_cpp::get_package_share_directory("yaren_filters") + "/imgs";

        glasses_filter_   = std::make_shared<GlassesFilter>(ap + "/glasses");
        mouth_filter_     = std::make_shared<MouthFilter>  (ap + "/mouths");
        nose_filter_      = std::make_shared<NoseFilter>   (ap + "/noses");
        hat_filter_       = std::make_shared<HatFilter>    (ap + "/hats");
        face_mask_filter_ = std::make_shared<FaceMaskFilter>(ap + "/faces");

        image_sub_.subscribe(this, "/csi_camera/image_raw");
        landmarks_sub_.subscribe(this, "face_landmarks");
        sync_ = std::make_shared<message_filters::Synchronizer<ApproximateTimePolicy>>(
            ApproximateTimePolicy(10), image_sub_, landmarks_sub_);
        sync_->registerCallback(&FaceFilterNode::callback, this);

        image_pub_ = image_transport::create_publisher(
            this, "filtered_image", qos.get_rmw_qos_profile());

        RCLCPP_INFO(get_logger(), "FaceFilterNode listo (modo mouse).");
    }

    // Sincroniza los filtros con el estado del menú
    void apply_states(const std::vector<CatState>& states)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < (int)CATS.size(); ++i) {
            const auto& st = states[i];
            if (!st.enabled || st.index < 0) continue;
            const std::string& id = CATS[i].id;
            if      (id == "hat")     hat_filter_->setCurrentIndex(st.index);
            else if (id == "glasses") glasses_filter_->setCurrentIndex(st.index);
            else if (id == "nose")    nose_filter_->setCurrentIndex(st.index);
            else if (id == "mouth")   mouth_filter_->setCurrentIndex(st.index);
            else if (id == "mask")    face_mask_filter_->setCurrentIndex(st.index);
        }
        active_states_ = states;
    }

    bool get_last_frame(cv::Mat& out)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_frame_) return false;
        out = last_frame_;
        has_frame_ = false;
        return true;
    }

    std::atomic<bool> cam_clicked_{ false };

private:
    void callback(const ImageMsg::ConstSharedPtr& img_msg,
                  const Landmarks::ConstSharedPtr& lm_msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;

            std::vector<cv::Point2f> landmarks;
            for (auto& p : lm_msg->landmarks)
                landmarks.emplace_back(p.x * frame.cols, p.y * frame.rows);

            {
                std::lock_guard<std::mutex> lock(mutex_);
                cv::flip(frame, frame, 0);

                if (!landmarks.empty()) {
                    // Indice 4 = mascara
                    bool mask_on = (active_states_.size() > 4 && active_states_[4].enabled);

                    if (mask_on) {
                        frame = face_mask_filter_->applyFilter(frame, landmarks, frame.size());
                    } else {
                        for (int i = 0; i < (int)CATS.size(); ++i) {
                            if (!active_states_[i].enabled) continue;
                            const std::string& id = CATS[i].id;
                            if      (id == "hat")     frame = hat_filter_->applyFilter(frame, landmarks, frame.size());
                            else if (id == "glasses") frame = glasses_filter_->applyFilter(frame, landmarks, frame.size());
                            else if (id == "nose")    frame = nose_filter_->applyFilter(frame, landmarks, frame.size());
                            else if (id == "mouth")   frame = mouth_filter_->applyFilter(frame, landmarks, frame.size());
                        }
                    }
                }
            }

            // Zoom + recorte centrado sin bordes negros
            const int TW = WIN_W, TH = WIN_H;
            double sc = std::max((double)TW/frame.cols, (double)TH/frame.rows);
            cv::Mat zoomed;
            cv::resize(frame, zoomed, cv::Size(), sc, sc, cv::INTER_AREA);
            int cx = (zoomed.cols - TW)/2, cy = (zoomed.rows - TH)/2;
            cv::Mat cropped = zoomed(cv::Rect(cx, cy, TW, TH));

            {
                std::lock_guard<std::mutex> lock(mutex_);
                last_frame_ = cropped.clone();
                has_frame_  = true;
            }

            image_pub_.publish(
                cv_bridge::CvImage(img_msg->header, "bgr8", cropped).toImageMsg());

        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
        }
    }

    std::shared_ptr<GlassesFilter>   glasses_filter_;
    std::shared_ptr<MouthFilter>     mouth_filter_;
    std::shared_ptr<NoseFilter>      nose_filter_;
    std::shared_ptr<HatFilter>       hat_filter_;
    std::shared_ptr<FaceMaskFilter>  face_mask_filter_;

    image_transport::Publisher       image_pub_;
    Subscriber<ImageMsg>             image_sub_;
    Subscriber<Landmarks>            landmarks_sub_;
    std::shared_ptr<message_filters::Synchronizer<ApproximateTimePolicy>> sync_;

    std::mutex               mutex_;
    std::atomic<bool>        running_;
    std::vector<CatState>    active_states_{ CATS.size() };
    cv::Mat                  last_frame_;
    bool                     has_frame_{ false };
};

// ================================================================
//  MAIN
// ================================================================
static void publish_idle(rclcpp::Node::SharedPtr node)
{
    auto pub = node->create_publisher<std_msgs::msg::String>("/yaren_mode", 10);
    for (int i = 0; i < 5; ++i) {
        auto msg = std_msgs::msg::String();
        msg.data = "idle";
        pub->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
    }
    RCLCPP_INFO(node->get_logger(), "Publicado 'idle' -> face_screen detendra el proceso.");
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceFilterNode>();

    // -- Cargar thumbnails de cada categoria --
    std::string ap = ament_index_cpp::get_package_share_directory("yaren_filters") + "/imgs";
    std::vector<std::string> subdirs = {"hats","glasses","noses","mouths","faces"};

    std::vector<CatState> states(CATS.size());
    for (int i = 0; i < (int)CATS.size(); ++i) {
        states[i].previews = load_thumbs(ap + "/" + subdirs[i], 100, 70);
        states[i].max_idx  = (int)states[i].previews.size();
        states[i].index    = states[i].max_idx > 0 ? 0 : -1;
        states[i].enabled  = false;
    }

    // -- Executor en hilo separado --
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // -- Loop principal: menu -> camara -> menu -> ... --
    while (rclcpp::ok()) {

        // 1. Mostrar menu
        bool applied = show_menu(states);

        if (!applied) {
            // Usuario presiono X
            cv::destroyAllWindows();
            publish_idle(node);
            break;
        }

        // 2. Aplicar configuracion al nodo
        node->apply_states(states);
        node->cam_clicked_ = false;

        // 3. Ventana de camara (fullscreen)
        cv::namedWindow(CAM_WIN, cv::WINDOW_NORMAL);
        cv::setWindowProperty(CAM_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::setMouseCallback(CAM_WIN, on_cam_mouse, &node->cam_clicked_);

        // 4. Loop de camara
        while (rclcpp::ok() && !node->cam_clicked_) {
            cv::Mat frame;
            if (node->get_last_frame(frame))
                cv::imshow(CAM_WIN, frame);
            cv::waitKey(16);
        }

        // 5. Click en camara -> volver al menu
        cv::destroyWindow(CAM_WIN);
    }

    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
