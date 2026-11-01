// AnimalFaceNode.cpp  —  LifecycleNode
#include "AnimalFaceNode.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <cmath>
#include <sstream>
#include <atomic>
#include <algorithm>
#include <chrono>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <cstdlib>
#include <unistd.h>    // fork, execl, _exit
#include <signal.h>    // kill, SIGTERM
#include <sys/wait.h>  // waitpid

static const int  WIN_W      = 800;
static const int  WIN_H      = 480;
static const char MENU_WIN[] = "Selecciona tu filtro";
static const char CAM_WIN[]  = "Animal Filter";

static const int COLS       = 3;
static const int CARD_W     = 210;
static const int CARD_H     = 310;
static const int CARD_PAD   = 30;
static const int TOP_OFFSET = 90;
static const cv::Rect BTN_PREV(10, TOP_OFFSET + CARD_H/2 - 25, 40, 50);
static const cv::Rect BTN_NEXT(WIN_W - 50, TOP_OFFSET + CARD_H/2 - 25, 40, 50);
static const cv::Rect BTN_APPLY(300, 430, 200, 48);
static const cv::Rect BTN_CLOSE(WIN_W - 55, 10, 45, 45);

// ── Cooldown entre detecciones (segundos) ─────────────────────────────────
static const double TONGUE_COOLDOWN = 3.0;

// ── Config por animal ──────────────────────────────────────────────────────
struct AnimalConfig {
    std::string bg_file;
    std::string sound_file;
};

static const std::map<std::string, AnimalConfig> ANIMAL_CONFIG = {
    { "bear",   { "fondopanda.png",  "panda.mp3" } },
    { "cat",    { "fondogato.png",   "gato.mp3"  } },
    { "monkey", { "fondomono2.jpg",  "mono.mp3"  } },
};

struct Texts {
    std::string menu_title, btn_apply;
    std::vector<std::string> animal_labels;
    std::string log_ready, log_filter_set, log_filter_error,
                log_cb_error, log_finished, log_lang_updated;
};

static Texts make_texts(bool en) {
    Texts t;
    if (en) {
        t.menu_title      = "Choose your animal filter";
        t.btn_apply       = "Apply";
        t.animal_labels   = { "Bear", "Cat", "Monkey" };
        t.log_ready       = "Animal filter node ready.";
        t.log_filter_set  = "Active filter: ";
        t.log_filter_error= "Error loading filter";
        t.log_cb_error    = "Callback error: ";
        t.log_finished    = "Animal filter closed by user.";
        t.log_lang_updated= "Language updated to: English";
    } else {
        t.menu_title      = "Elige tu filtro de animal";
        t.btn_apply       = "Aplicar";
        t.animal_labels   = { "Oso", "Gato", "Mono" };
        t.log_ready       = "Nodo de filtros animales listo.";
        t.log_filter_set  = "Filtro activo: ";
        t.log_filter_error= "Error al cargar filtro";
        t.log_cb_error    = "Error en callback: ";
        t.log_finished    = "Filtro de animales finalizado por usuario.";
        t.log_lang_updated= "Idioma actualizado a: Español";
    }
    return t;
}

struct FilterOption {
    std::string id;
    cv::Scalar  border_color;
};

static const std::vector<FilterOption> FILTERS = {
    { "bear",   { 43,  130, 210 } },
    { "cat",    { 200,  80,  80 } },
    { "monkey", {  40, 180,  60 } }
};

static std::atomic<bool> g_animal_filter_ready{false};

// ══════════════════════════════════════════════════════════════
//  Helpers de UI
// ══════════════════════════════════════════════════════════════

static void overlay_rgba(cv::Mat& bg, const cv::Mat& fg, int ox, int oy) {
    if (fg.empty()) return;
    int y1 = std::max(oy, 0), y2 = std::min(oy + fg.rows, bg.rows);
    int x1 = std::max(ox, 0), x2 = std::min(ox + fg.cols, bg.cols);
    if (x1 >= x2 || y1 >= y2) return;
    cv::Mat fc = fg(cv::Rect(x1-ox, y1-oy, x2-x1, y2-y1));
    cv::Mat bc = bg(cv::Rect(x1, y1, x2-x1, y2-y1));
    std::vector<cv::Mat> ch; cv::split(fc, ch);
    bool ha = (ch.size() == 4);
    cv::Mat a32, inv;
    if (ha) { ch[3].convertTo(a32, CV_32F, 1./255.); inv = 1.f - a32; }
    cv::Mat fg_bgr;
    if (fc.channels() == 4) cv::cvtColor(fc, fg_bgr, cv::COLOR_BGRA2BGR);
    else fg_bgr = fc;
    cv::Mat bf, ff;
    bc.convertTo(bf, CV_32FC3, 1./255.);
    fg_bgr.convertTo(ff, CV_32FC3, 1./255.);
    cv::Mat out;
    if (ha) {
        std::vector<cv::Mat> bc3, fc3, r(3);
        cv::split(bf, bc3); cv::split(ff, fc3);
        for (int c = 0; c < 3; ++c) r[c] = fc3[c].mul(a32) + bc3[c].mul(inv);
        cv::merge(r, out);
    } else { out = ff; }
    cv::Mat o8; out.convertTo(o8, CV_8UC3, 255.);
    o8.copyTo(bc);
}

[[maybe_unused]] static cv::Rect card_rect(int idx) {
    int col = idx % COLS, row = idx / COLS;
    int total_w = COLS * CARD_W + (COLS - 1) * CARD_PAD;
    int x0 = (WIN_W - total_w) / 2 + col * (CARD_W + CARD_PAD);
    int y0 = TOP_OFFSET + row * (CARD_H + CARD_PAD);
    return { x0, y0, CARD_W, CARD_H };
}

static cv::Mat crop_transparent(const cv::Mat& img) {
    if (img.empty() || img.channels() != 4) return img;
    std::vector<cv::Mat> ch; cv::split(img, ch);
    cv::Rect bb = cv::boundingRect(ch[3]);
    return (bb.width > 0 && bb.height > 0) ? img(bb).clone() : img;
}

static std::vector<cv::Mat> load_previews() {
    std::vector<cv::Mat> previews(FILTERS.size());
    try {
        std::string pkg = ament_index_cpp::get_package_share_directory("yaren_filters");
        for (int i = 0; i < (int)FILTERS.size(); ++i) {
            std::string path = pkg + "/imgs/animals_mask/" + FILTERS[i].id
                             + "/" + FILTERS[i].id + "_open_open.png";
            cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
            if (img.empty()) continue;
            img = crop_transparent(img);
            int mw = CARD_W - 30, mh = CARD_H - 95;
            double sc = std::min((double)mw / img.cols, (double)mh / img.rows);
            cv::resize(img, previews[i],
                       cv::Size((int)(img.cols*sc),(int)(img.rows*sc)),
                       0, 0, cv::INTER_AREA);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error cargando previews: " << e.what() << std::endl;
    }
    return previews;
}

// ══════════════════════════════════════════════════════════════
//  Utilidades visuales
// ══════════════════════════════════════════════════════════════
static double clampd(double v, double a, double b) { return std::max(a, std::min(b, v)); }
static double out_cubic_d(double x) { x = clampd(x, 0, 1); return 1.0 - std::pow(1.0 - x, 3); }
static double out_back_d(double x) {
    x = clampd(x, 0, 1);
    const double c1 = 1.70158, c3 = c1 + 1.0;
    return 1.0 + c3 * std::pow(x - 1.0, 3) + c1 * std::pow(x - 1.0, 2);
}
static double now_sec() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

static std::vector<cv::Mat> load_raw_animals() {
    std::vector<cv::Mat> out(FILTERS.size());
    try {
        std::string pkg = ament_index_cpp::get_package_share_directory("yaren_filters");
        for (size_t i = 0; i < FILTERS.size(); ++i) {
            std::string path = pkg + "/imgs/animals_mask/" + FILTERS[i].id
                             + "/" + FILTERS[i].id + "_open_open.png";
            cv::Mat img = cv::imread(path, cv::IMREAD_UNCHANGED);
            if (img.empty()) continue;
            if (img.channels() == 3) cv::cvtColor(img, img, cv::COLOR_BGR2BGRA);
            out[i] = crop_transparent(img);
        }
    } catch (...) {}
    return out;
}

static void draw_animal(cv::Mat& canvas, const cv::Mat& img, const cv::Scalar& col,
                        int cx, int cy, int h, double alpha)
{
    if (alpha <= 0.02 || h < 8) return;
    cv::Mat rs;
    int w;
    if (img.empty()) {
        w = h;
        rs = cv::Mat(h, w, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        cv::circle(rs, cv::Point(w / 2, h / 2), h / 2 - 2,
                   cv::Scalar(col[0], col[1], col[2], 255), cv::FILLED, cv::LINE_AA);
    } else {
        double sc = std::min((double)h / img.rows, (double)(h * 1.3) / img.cols);
        w = std::max(1, (int)(img.cols * sc));
        h = std::max(1, (int)(img.rows * sc));
        cv::resize(img, rs, cv::Size(w, h), 0, 0, cv::INTER_AREA);
    }
    int ox = cx - w / 2, oy = cy - h / 2;
    cv::Rect clip = cv::Rect(ox, oy, w, h) & cv::Rect(0, 0, canvas.cols, canvas.rows);
    if (clip.area() <= 0) return;
    if (alpha >= 0.99) { overlay_rgba(canvas, rs, ox, oy); return; }
    cv::Mat roi = canvas(clip);
    cv::Mat tmp = roi.clone();
    overlay_rgba(tmp, rs, ox - clip.x, oy - clip.y);
    cv::addWeighted(tmp, alpha, roi, 1.0 - alpha, 0, roi);
}

static void draw_paw(cv::Mat& f, cv::Point c, int r, const cv::Scalar& col)
{
    cv::ellipse(f, cv::Point(c.x, c.y + r / 3), cv::Size(r, (int)(r * 0.8)), 0, 0, 360,
                col, cv::FILLED, cv::LINE_AA);
    const double tx[4] = {-1.3, -0.45, 0.45, 1.3};
    const double ty[4] = {-0.4, -1.1, -1.1, -0.4};
    for (int k = 0; k < 4; ++k)
        cv::circle(f, cv::Point(c.x + (int)(tx[k] * r), c.y + (int)(ty[k] * r)),
                   std::max(1, (int)(r * 0.38)), col, cv::FILLED, cv::LINE_AA);
}

static void pixel_box(cv::Mat& f, const cv::Rect& r, int cut, const cv::Scalar& col)
{
    std::vector<cv::Point> p = {
        {r.x + cut, r.y}, {r.x + r.width - cut, r.y},
        {r.x + r.width - cut, r.y + cut}, {r.x + r.width, r.y + cut},
        {r.x + r.width, r.y + r.height - cut}, {r.x + r.width - cut, r.y + r.height - cut},
        {r.x + r.width - cut, r.y + r.height}, {r.x + cut, r.y + r.height},
        {r.x + cut, r.y + r.height - cut}, {r.x, r.y + r.height - cut},
        {r.x, r.y + cut}, {r.x + cut, r.y + cut}};
    cv::fillPoly(f, p, col);
}

static void pixel_arrow(cv::Mat& f, cv::Point c, int dir, const cv::Scalar& col, double sc)
{
    auto P = [&](double x, double y) {
        return cv::Point(c.x + (int)(dir * x * sc), c.y + (int)(y * sc));
    };
    std::vector<cv::Point> p = {
        P(-24, -8), P(2, -8), P(2, -20), P(10, -20), P(10, -12), P(18, -12), P(18, -4), P(26, -4),
        P(26, 4), P(18, 4), P(18, 12), P(10, 12), P(10, 20), P(2, 20), P(2, 8), P(-24, 8)};
    cv::fillPoly(f, p, col);
}

static void draw_sky(cv::Mat& f, double t)
{
    for (int y = 0; y < f.rows; ++y) {
        double k = (double)y / (f.rows - 1);
        f.row(y).setTo(cv::Scalar(225, 150, 10) * (1.0 - k) + cv::Scalar(255, 215, 140) * k);
    }
    struct C { double x, y, s, v; };
    static const C cl[4] = {{60, 60, 6, 6}, {330, 115, 5, 4}, {560, 40, 7, 5}, {700, 150, 5, 7}};
    const cv::Scalar white(255, 255, 255);
    for (const auto& c : cl) {
        int x = (int)std::fmod(c.x + t * c.v, f.cols + 160.0) - 80;
        int s = (int)c.s, y = (int)c.y;
        cv::rectangle(f, cv::Rect(x + 2 * s, y, 3 * s, s), white, cv::FILLED);
        cv::rectangle(f, cv::Rect(x + s, y + s, 7 * s, s), white, cv::FILLED);
        cv::rectangle(f, cv::Rect(x, y + 2 * s, 9 * s, 2 * s), white, cv::FILLED);
    }
}

// ══════════════════════════════════════════════════════════════
//  Selector de animal (estilo selector de personajes)
// ══════════════════════════════════════════════════════════════
struct MenuState {
    int selected = 0;
    bool apply_hit = false;
    bool close_hit = false;
    double slide = 0.0;
    cv::Point mouse{-1, -1};
};

static const cv::Rect ARW_L(236, 342, 70, 60);
static const cv::Rect ARW_R(494, 342, 70, 60);
static const cv::Rect BTN_START(250, 410, 300, 62);

static void menu_go(MenuState* s, int dir)
{
    int n = (int)FILTERS.size();
    s->selected = ((s->selected + dir) % n + n) % n;
    s->slide = dir;
}

static void on_menu_mouse(int event, int x, int y, int, void* ud)
{
    auto* s = reinterpret_cast<MenuState*>(ud);
    s->mouse = cv::Point(x, y);
    if (event != cv::EVENT_LBUTTONDOWN) return;
    cv::Point p(x, y);
    if (BTN_CLOSE.contains(p)) { s->close_hit = true; return; }
    if (BTN_START.contains(p)) { s->apply_hit = true; return; }
    if (ARW_L.contains(p)) { menu_go(s, -1); return; }
    if (ARW_R.contains(p)) { menu_go(s, +1); return; }
    if (y > 70 && y < 335) {
        if (x < 270) menu_go(s, -1);
        else if (x > 530) menu_go(s, +1);
        else s->apply_hit = true;
    }
}

static void on_cam_mouse(int event, int, int, int, void* ud) {
    if (event == cv::EVENT_LBUTTONDOWN)
        *reinterpret_cast<std::atomic<bool>*>(ud) = true;
}

static void draw_menu(cv::Mat& canvas, MenuState& ms, const std::vector<cv::Mat>& raws,
                      const Texts& txt)
{
    const double t = now_sec();
    const int n = (int)FILTERS.size();
    const bool eng = (txt.btn_apply == "Apply");
    draw_sky(canvas, t);

    ms.slide *= 0.80;
    if (std::fabs(ms.slide) < 0.01) ms.slide = 0.0;

    const int SP = 250;
    std::vector<std::pair<double, int>> order;
    for (int rel = -2; rel <= 2; ++rel) order.push_back({std::fabs(rel + ms.slide), rel});
    std::sort(order.begin(), order.end(),
              [](const std::pair<double, int>& a, const std::pair<double, int>& b) { return a.first > b.first; });

    for (const auto& o : order) {
        int rel = o.second;
        double pos = rel + ms.slide, d = std::fabs(pos);
        if (d > 1.7) continue;
        int idx = (((ms.selected + rel) % n) + n) % n;
        double tt = clampd(1.0 - d, 0.0, 1.0);
        int h = (int)(125 + 125 * tt);
        int cx = 400 + (int)(pos * SP);
        int cy = (int)(225 - 40 * tt);
        double alpha = (0.55 + 0.45 * tt) * clampd((1.7 - d) / 0.7, 0.0, 1.0);
        if (alpha > 0.3)
            cv::ellipse(canvas, cv::Point(cx, cy + h / 2 + 6), cv::Size((int)(h * 0.4), 8), 0, 0, 360,
                        cv::Scalar(185, 120, 20), cv::FILLED, cv::LINE_AA);
        int cyb = cy + (int)(5.0 * std::sin(t * 3.0) * tt);
        draw_animal(canvas, raws[idx], FILTERS[idx].border_color, cx, cyb, h, alpha);
    }

    // huellita sobre el animal central
    draw_paw(canvas, cv::Point(400, 38 + (int)(4 * std::sin(t * 3.0))), 11, cv::Scalar(40, 30, 30));

    // nombre y flechas
    std::string name = txt.animal_labels[ms.selected];
    for (auto& ch : name) ch = (char)std::toupper((unsigned char)ch);
    int base = 0;
    cv::Size ts = cv::getTextSize(name, cv::FONT_HERSHEY_DUPLEX, 1.1, 2, &base);
    cv::Point org(400 - ts.width / 2, 384);
    cv::putText(canvas, name, org, cv::FONT_HERSHEY_DUPLEX, 1.1, cv::Scalar(255, 255, 255), 6, cv::LINE_AA);
    cv::putText(canvas, name, org, cv::FONT_HERSHEY_DUPLEX, 1.1, cv::Scalar(60, 30, 20), 2, cv::LINE_AA);

    cv::Point cl(ARW_L.x + ARW_L.width / 2, ARW_L.y + ARW_L.height / 2);
    cv::Point cr(ARW_R.x + ARW_R.width / 2, ARW_R.y + ARW_R.height / 2);
    bool hl = ARW_L.contains(ms.mouse), hr = ARW_R.contains(ms.mouse);
    pixel_arrow(canvas, cv::Point(cl.x + 3, cl.y + 3), -1, cv::Scalar(150, 90, 5), 1.2);
    pixel_arrow(canvas, cl, -1, hl ? cv::Scalar(120, 70, 235) : cv::Scalar(30, 20, 20), 1.2);
    pixel_arrow(canvas, cv::Point(cr.x + 3, cr.y + 3), +1, cv::Scalar(150, 90, 5), 1.2);
    pixel_arrow(canvas, cr, +1, hr ? cv::Scalar(120, 70, 235) : cv::Scalar(30, 20, 20), 1.2);

    // boton START
    bool hs = BTN_START.contains(ms.mouse);
    cv::Rect r = BTN_START;
    r.y += (int)(2.0 * std::sin(t * 4.0));
    cv::Scalar border(70, 30, 90), pink = hs ? cv::Scalar(150, 110, 250) : cv::Scalar(120, 70, 235);
    pixel_box(canvas, cv::Rect(r.x - 5 + 4, r.y - 5 + 6, r.width + 10, r.height + 10), 9, cv::Scalar(150, 90, 5));
    pixel_box(canvas, cv::Rect(r.x - 5, r.y - 5, r.width + 10, r.height + 10), 9, border);
    pixel_box(canvas, r, 6, pink);
    cv::rectangle(canvas, cv::Rect(r.x + 12, r.y + 8, 30, 6), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::rectangle(canvas, cv::Rect(r.x + 12, r.y + 8, 6, 22), cv::Scalar(255, 255, 255), cv::FILLED);
    std::string sl = eng ? "START" : "INICIAR";
    cv::Size bs = cv::getTextSize(sl, cv::FONT_HERSHEY_DUPLEX, 1.5, 3, &base);
    cv::putText(canvas, sl, cv::Point(r.x + (r.width - bs.width) / 2, r.y + (r.height + bs.height) / 2),
                cv::FONT_HERSHEY_DUPLEX, 1.5, border, 3, cv::LINE_AA);

    // cerrar
    cv::rectangle(canvas, BTN_CLOSE, cv::Scalar(180, 30, 30), cv::FILLED);
    int m = 12;
    cv::line(canvas, {BTN_CLOSE.x + m, BTN_CLOSE.y + m},
             {BTN_CLOSE.x + BTN_CLOSE.width - m, BTN_CLOSE.y + BTN_CLOSE.height - m}, {255, 255, 255}, 3, cv::LINE_AA);
    cv::line(canvas, {BTN_CLOSE.x + BTN_CLOSE.width - m, BTN_CLOSE.y + m},
             {BTN_CLOSE.x + m, BTN_CLOSE.y + BTN_CLOSE.height - m}, {255, 255, 255}, 3, cv::LINE_AA);
}

static std::string show_menu(const std::vector<cv::Mat>& previews, const Texts& txt) {
    (void)previews;
    std::vector<cv::Mat> raws = load_raw_animals();
    cv::namedWindow(MENU_WIN, cv::WINDOW_NORMAL);
    cv::setWindowProperty(MENU_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    std::thread([]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        std::string cmd = std::string("xdotool search --sync --name '")
                        + MENU_WIN
                        + "' windowactivate --sync windowraise 2>/dev/null";
        std::system(cmd.c_str());
    }).detach();
    MenuState ms;
    cv::setMouseCallback(MENU_WIN, on_menu_mouse, &ms);
    cv::Mat canvas(WIN_H, WIN_W, CV_8UC3);
    while (!ms.apply_hit && !ms.close_hit) {
        draw_menu(canvas, ms, raws, txt);
        cv::imshow(MENU_WIN, canvas);
        if (cv::waitKey(16) == 27) ms.close_hit = true;
    }
    cv::destroyWindow(MENU_WIN);
    if (ms.close_hit || ms.selected < 0) return "";
    return FILTERS[ms.selected].id;
}

// ══════════════════════════════════════════════════════════════
//  TongueDetector — detecta boca muy abierta (toggle con cooldown)
// ══════════════════════════════════════════════════════════════
class TongueDetector {
public:
    struct Result {
        bool triggered;
        bool turning_on;
    };

    Result check(const std::vector<cv::Point2f>& lm) {
        if ((int)lm.size() < 468) return {false, false};

        float upper_lip_y  = lm[13].y;
        float lower_lip_y  = lm[14].y;
        float mouth_width  = std::abs(lm[308].x - lm[78].x);
        float mouth_open   = lower_lip_y - upper_lip_y;

        bool wide_open = (mouth_open > mouth_width * 0.20f);
        if (!wide_open) return {false, false};

        auto now     = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_trigger_).count();
        if (elapsed < TONGUE_COOLDOWN) return {false, false};

        last_trigger_ = now;
        bool turning_on = !bg_active_;
        bg_active_      = !bg_active_;
        return {true, turning_on};
    }

    bool is_bg_active() const { return bg_active_; }

private:
    std::chrono::steady_clock::time_point last_trigger_{};
    bool bg_active_ = false;
};

// ══════════════════════════════════════════════════════════════
//  VirtualBackground — activar/desactivar con fade
// ══════════════════════════════════════════════════════════════
class VirtualBackground {
public:
    void load(const std::string& pkg_path, const std::string& filename) {
        std::string path = pkg_path + "/imgs/backgrounds/" + filename;
        bg_orig_ = cv::imread(path, cv::IMREAD_COLOR);
        if (bg_orig_.empty()) {
            std::cerr << "[VirtualBG] No se encontró: " << path << std::endl;
        } else {
            bg_ready_.release();
            std::cout << "[VirtualBG] Cargado: " << path << std::endl;
        }
    }

    void set_active(bool on) {
        if (on != active_) {
            transition_start_ = std::chrono::steady_clock::now();
        }
        active_ = on;
    }

    bool is_active() const { return active_; }

    cv::Mat apply(const cv::Mat& frame) {
        if (bg_orig_.empty()) return frame;

        if (bg_ready_.size() != frame.size()) {
            cv::resize(bg_orig_, bg_ready_, frame.size());
        }

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - transition_start_).count();

        double alpha;
        if (active_) {
            alpha = std::min(elapsed / 0.4, 1.0);
        } else {
            alpha = std::max(0.0, 1.0 - elapsed / 0.4);
            if (alpha <= 0.0) return frame;
        }

        cv::Mat result;
        cv::addWeighted(bg_ready_, alpha, frame, 1.0 - alpha, 0, result);
        return result;
    }

private:
    cv::Mat    bg_orig_;
    cv::Mat    bg_ready_;
    bool       active_ = false;
    std::chrono::steady_clock::time_point transition_start_{};
};

// ══════════════════════════════════════════════════════════════
//  AnimalFaceNode — implementación
// ══════════════════════════════════════════════════════════════
AnimalFaceNode::AnimalFaceNode()
    : rclcpp_lifecycle::LifecycleNode("filtro_animales"),
      is_english_(false),
      sound_pid_(-1)
{
    get_logger();
}

CallbackReturn AnimalFaceNode::on_configure(const rclcpp_lifecycle::State&)
{
    rclcpp::QoS lang_qos(1);
    lang_qos.transient_local();
    language_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/yaren/is_english", lang_qos,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            bool prev = is_english_.load();
            is_english_ = msg->data;
            language_received_ = true;
            if (prev != is_english_.load()) {
                Texts txt = make_texts(is_english_);
                RCLCPP_INFO(get_logger(), "%s", txt.log_lang_updated.c_str());
            }
        });

    previews_ = load_previews();
    mode_pub_ = this->create_publisher<std_msgs::msg::String>("/yaren_mode", 10);
    RCLCPP_INFO(get_logger(), "Configurado. Previews cargados.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn AnimalFaceNode::on_activate(const rclcpp_lifecycle::State&)
{
    image_sub_.subscribe(this, "/csi_camera/image_raw", rmw_qos_profile_default);
    landmarks_sub_.subscribe(this, "/face_landmarks");

    sync_ = std::make_shared<Synchronizer>(
        ApproximateTimePolicy(10), image_sub_, landmarks_sub_);
    sync_->registerCallback(&AnimalFaceNode::callback, this);

    image_pub_ = this->create_publisher<ImageMsg>("filtered_image", 10);

    tongue_detector_ = std::make_unique<TongueDetector>();
    virtual_bg_      = std::make_unique<VirtualBackground>();

    auto wait_start = std::chrono::steady_clock::now();
    while (!language_received_.load() &&
           std::chrono::steady_clock::now() - wait_start < std::chrono::milliseconds(500)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    ui_running_  = true;
    cam_clicked_ = false;
    ui_thread_   = std::thread(&AnimalFaceNode::run_ui, this);

    RCLCPP_INFO(get_logger(), "ACTIVADO — mostrando menú de animales.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn AnimalFaceNode::on_deactivate(const rclcpp_lifecycle::State&)
{
    ui_running_  = false;
    cam_clicked_ = true;

    // ── Detener audio al desactivar ──────────────────────────
    stop_sound();

    if (ui_thread_.joinable()) ui_thread_.join();

    sync_.reset();
    image_pub_.reset();
    tongue_detector_.reset();
    virtual_bg_.reset();

    cv::destroyAllWindows();
    RCLCPP_INFO(get_logger(), "DESACTIVADO.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn AnimalFaceNode::on_cleanup(const rclcpp_lifecycle::State&)
{
    language_sub_.reset();
    previews_.clear();
    RCLCPP_INFO(get_logger(), "Limpieza completada.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn AnimalFaceNode::on_shutdown(const rclcpp_lifecycle::State&)
{
    ui_running_  = false;
    cam_clicked_ = true;

    // ── Detener audio al apagar ──────────────────────────────
    stop_sound();

    if (ui_thread_.joinable()) ui_thread_.join();
    cv::destroyAllWindows();
    return CallbackReturn::SUCCESS;
}

void AnimalFaceNode::load_animal_assets(const std::string& animal)
{
    try {
        std::string pkg = ament_index_cpp::get_package_share_directory("yaren_filters");
        auto it = ANIMAL_CONFIG.find(animal);
        if (it == ANIMAL_CONFIG.end()) return;

        const AnimalConfig& cfg = it->second;

        if (virtual_bg_) {
            virtual_bg_->load(pkg, cfg.bg_file);
        }

        current_sound_path_ = pkg + "/imgs/sounds/" + cfg.sound_file;
        RCLCPP_INFO(get_logger(), "Assets cargados para: %s", animal.c_str());
        RCLCPP_INFO(get_logger(), "Sonido: %s", current_sound_path_.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error cargando assets: %s", e.what());
    }
}

// ── Para el proceso de audio si existe ────────────────────────────────────
void AnimalFaceNode::stop_sound()
{
    if (sound_pid_ > 0) {
        kill(sound_pid_, SIGTERM);
        waitpid(sound_pid_, nullptr, WNOHANG);
        sound_pid_ = -1;
    }
}

// ── Toggle de audio: turn_on=true arranca mpg123 en loop,
//                    turn_on=false lo mata ────────────────────────────────
void AnimalFaceNode::play_sound(bool turn_on)
{
    // Siempre matar el proceso anterior primero
    stop_sound();

    if (!turn_on || current_sound_path_.empty()) return;

    // Lanzar mpg123 en loop (-1 = infinito) y guardar el PID
    sound_pid_ = fork();
    if (sound_pid_ == 0) {
        // Proceso hijo — reemplaza la imagen con mpg123
        execl("/usr/bin/mpg123", "mpg123", "-q", "--loop", "-1",
              current_sound_path_.c_str(), nullptr);
        _exit(1);  // Solo llega aquí si execl falla
    }
}
static void anim_pill(cv::Mat& f, const cv::Rect& r, const cv::Scalar& col)
{
    int rad = r.height / 2;
    cv::rectangle(f, cv::Rect(r.x + rad, r.y, r.width - 2 * rad, r.height), col, cv::FILLED, cv::LINE_AA);
    cv::circle(f, cv::Point(r.x + rad, r.y + rad), rad, col, cv::FILLED, cv::LINE_AA);
    cv::circle(f, cv::Point(r.x + r.width - rad, r.y + rad), rad, col, cv::FILLED, cv::LINE_AA);
}

static void shadow_txt(cv::Mat& f, const std::string& s, cv::Point o, int font, double sc,
                       const cv::Scalar& col, int th)
{
    cv::putText(f, s, cv::Point(o.x + 2, o.y + 2), font, sc, cv::Scalar(10, 8, 20), th + 2, cv::LINE_AA);
    cv::putText(f, s, o, font, sc, col, th, cv::LINE_AA);
}

static bool show_intro_screen(bool is_english)
{
    const int W = 800, H = 480;
    const char* WIN = "Animal Filter Intro";
    struct State { bool clicked = false; bool hover = false; cv::Rect btn; } st;
    st.btn = cv::Rect(50, 334, 390, 70);
    bool closed = false;

    cv::namedWindow(WIN, cv::WINDOW_NORMAL);
    cv::setWindowProperty(WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    cv::setWindowProperty(WIN, cv::WND_PROP_TOPMOST, 1);
    cv::setMouseCallback(WIN, [](int event, int x, int y, int, void* ud) {
        auto* s = reinterpret_cast<State*>(ud);
        bool in = s->btn.contains(cv::Point(x, y));
        if (event == cv::EVENT_MOUSEMOVE) s->hover = in;
        else if (event == cv::EVENT_LBUTTONDOWN && in) s->clicked = true;
    }, &st);

    std::vector<cv::Mat> raws = load_raw_animals();
    const cv::Scalar WHITE(255, 255, 255);
    const int FD = cv::FONT_HERSHEY_DUPLEX, FS = cv::FONT_HERSHEY_SIMPLEX;

    std::string t1 = is_english ? "Animal" : "Filtros de";
    std::string t2 = is_english ? "Filters" : "Animales";
    std::string desc = is_english
        ? "Pick an animal and wear its mask. Open your mouth to turn on the virtual background and sound."
        : "Elige un animal y ponte su mascara. Abre la boca para activar el fondo virtual y el sonido.";
    std::string btn_lbl = is_english ? "Start" : "Comenzar";

    int base = 0;
    std::vector<std::string> lines;
    {
        std::istringstream iss(desc);
        std::string word, line;
        while (iss >> word) {
            std::string test = line.empty() ? word : line + " " + word;
            if (cv::getTextSize(test, FS, 0.5, 1, &base).width > 380 && !line.empty()) {
                lines.push_back(line); line = word;
            } else line = test;
        }
        if (!line.empty()) lines.push_back(line);
    }

    auto tone = [&](int i) -> cv::Scalar {
        cv::Scalar c = FILTERS[i].border_color, o;
        for (int k = 0; k < 3; ++k) o[k] = std::min(175.0, c[k] * 0.6 + 50.0);
        return o;
    };
    cv::Scalar cur = tone(0);
    const double t0 = now_sec();
    const double CYCLE = 2.4;

    while (!st.clicked && !closed) {
        double t = now_sec() - t0;
        int idx = ((int)(t / CYCLE)) % (int)FILTERS.size();
        double local = std::fmod(t, CYCLE) / CYCLE;
        cur = cur + (tone(idx) - cur) * 0.07;

        cv::Mat frame(H, W, CV_8UC3);
        cv::Scalar top = cur * 0.72, bot;
        for (int k = 0; k < 3; ++k) bot[k] = std::min(185.0, cur[k] * 0.95 + 8.0);
        for (int y = 0; y < H; ++y) {
            double k = (double)y / (H - 1);
            frame.row(y).setTo(top * (1.0 - k) + bot * k);
        }

        // aro y halo
        {
            cv::Mat ov = frame.clone();
            cv::circle(ov, cv::Point(630, 255), 175, WHITE, 16, cv::LINE_AA);
            cv::circle(ov, cv::Point(630, 255), 140, WHITE, -1, cv::LINE_AA);
            cv::addWeighted(ov, 0.10, frame, 0.90, 0, frame);
        }
        // burbujas
        {
            cv::Mat ov = frame.clone();
            for (int i = 0; i < 14; ++i) {
                int bx = (int)std::fmod(i * 61 + 30 * std::sin(t * 0.6 + i) + W * 2.0, (double)W);
                int by = (int)(H - std::fmod(i * 53 + t * (14 + (i % 4) * 6), H + 40.0) + 20);
                cv::circle(ov, cv::Point(bx, by), 4 + (i % 4) * 3, WHITE, -1, cv::LINE_AA);
            }
            cv::addWeighted(ov, 0.16, frame, 0.84, 0, frame);
        }

        // logo
        {
            cv::Size ls = cv::getTextSize("YAREN", FD, 1.1, 2, &base);
            shadow_txt(frame, "YAREN", cv::Point((W - ls.width) / 2, 58), FD, 1.1, WHITE, 2);
        }

        // animal grande (derecha)
        {
            double pop = out_back_d(local / 0.22);
            double bob = 9.0 * std::sin(t * 2.2);
            int h = (int)(250 * std::max(0.05, pop));
            int sw = (int)(95 * std::max(0.3, pop));
            cv::Mat ov = frame.clone();
            cv::ellipse(ov, cv::Point(630, 418), cv::Size(sw, 12), 0, 0, 360, cv::Scalar(0, 0, 0), -1, cv::LINE_AA);
            cv::addWeighted(ov, 0.25, frame, 0.75, 0, frame);
            draw_animal(frame, raws[idx], FILTERS[idx].border_color, 630, 250 + (int)bob, h, 1.0);
        }

        // columna izquierda
        cv::Mat baseFrame = frame.clone();
        double a_in = out_cubic_d(t / 0.9);
        int dx = (int)(-60 * (1.0 - a_in));

        shadow_txt(frame, t1, cv::Point(50 + dx, 140), FD, 1.15, WHITE, 2);
        shadow_txt(frame, t2, cv::Point(50 + dx, 184), FD, 1.15, WHITE, 2);

        cv::Scalar darkc = cur * 0.45;
        for (int i = 0; i < 3; ++i) {
            cv::Point c(68 + i * 46 + dx, 226);
            cv::circle(frame, c, 17, WHITE, -1, cv::LINE_AA);
            cv::circle(frame, c, 14, darkc, -1, cv::LINE_AA);
            if (i == 0) {
                draw_paw(frame, cv::Point(c.x, c.y + 1), 5, WHITE);
            } else if (i == 1) {
                cv::circle(frame, cv::Point(c.x - 4, c.y - 2), 1, WHITE, -1, cv::LINE_AA);
                cv::circle(frame, cv::Point(c.x + 4, c.y - 2), 1, WHITE, -1, cv::LINE_AA);
                cv::ellipse(frame, cv::Point(c.x, c.y + 1), cv::Size(6, 5), 0, 10, 170, WHITE, 1, cv::LINE_AA);
            } else {
                std::vector<cv::Point> sp = {{c.x - 7, c.y - 3}, {c.x - 3, c.y - 3}, {c.x + 2, c.y - 8},
                                             {c.x + 2, c.y + 8}, {c.x - 3, c.y + 3}, {c.x - 7, c.y + 3}};
                cv::fillPoly(frame, sp, WHITE, cv::LINE_AA);
                cv::ellipse(frame, cv::Point(c.x + 3, c.y), cv::Size(5, 5), 0, -50, 50, WHITE, 1, cv::LINE_AA);
            }
        }

        int y_d = 276;
        for (const auto& ln : lines) {
            shadow_txt(frame, ln, cv::Point(50 + dx, y_d), FS, 0.5, WHITE, 1);
            y_d += 21;
        }

        // boton grande con flecha
        cv::Rect b = st.btn;
        cv::Scalar bcol = cur * (st.hover ? 0.34 : 0.22);
        anim_pill(frame, cv::Rect(b.x + dx, b.y + 3, b.width, b.height), cv::Scalar(0, 0, 0));
        if (st.hover) anim_pill(frame, cv::Rect(b.x + dx - 2, b.y - 2, b.width + 4, b.height + 4), WHITE);
        anim_pill(frame, cv::Rect(b.x + dx, b.y, b.width, b.height), bcol);
        cv::Size bsz = cv::getTextSize(btn_lbl, FD, 1.0, 2, &base);
        int tx = b.x + dx + (b.width - bsz.width) / 2 + 28;
        int ty = b.y + (b.height + bsz.height) / 2;
        cv::putText(frame, btn_lbl, cv::Point(tx, ty), FD, 1.0, WHITE, 2, cv::LINE_AA);
        int ax = tx - 56 + (int)(5 * std::sin(t * 6.0));
        int ay = b.y + b.height / 2;
        cv::arrowedLine(frame, cv::Point(ax, ay), cv::Point(ax + 34, ay), WHITE, 4, cv::LINE_AA, 0, 0.5);

        if (a_in < 1.0) cv::addWeighted(frame, a_in, baseFrame, 1.0 - a_in, 0, frame);

        cv::imshow(WIN, frame);
        int key = cv::waitKey(16);
        if (key == 27) closed = true;
    }
    cv::destroyWindow(WIN);
    return st.clicked && !closed;
}
void AnimalFaceNode::run_ui()
{
    // ── INTRO (Solo aparece una vez al iniciar el nodo) ──────────────────
    if (!show_intro_screen(is_english_.load())) {
        if (rclcpp::ok() && ui_running_.load() && mode_pub_) {
            auto msg = std_msgs::msg::String();
            msg.data = "idle";
            mode_pub_->publish(msg);
        }
        return;
    }
    // ─────────────────────────────────────────────────────────────────────

    // Bucle principal: al salir de la cámara regresa directamente al menú
    while (rclcpp::ok() && ui_running_) {
        Texts txt = make_texts(is_english_.load());
        std::string chosen = show_menu(previews_, txt);

        if (chosen.empty() || !ui_running_) break;

        set_filter(chosen);
        load_animal_assets(chosen);

        // Al cambiar de animal, parar el audio anterior
        stop_sound();

        cam_clicked_ = false;

        cv::namedWindow(CAM_WIN, cv::WINDOW_NORMAL);
        cv::setWindowProperty(CAM_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

        std::thread([]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            std::system("xdotool search --sync --name 'Animal Filter' "
                        "windowactivate --sync windowraise 2>/dev/null");
        }).detach();

        cv::setMouseCallback(CAM_WIN, on_cam_mouse, &cam_clicked_);

        while (rclcpp::ok() && !cam_clicked_ && ui_running_) {
            cv::Mat frame;
            if (get_last_frame(frame)) cv::imshow(CAM_WIN, frame);
            cv::waitKey(16);
        }
        cv::destroyWindow(CAM_WIN);
    }

    // Al salir del loop por completo, parar audio
    stop_sound();

    if (rclcpp::ok() && ui_running_.load() && mode_pub_) {
        auto msg = std_msgs::msg::String();
        msg.data = "idle";
        mode_pub_->publish(msg);
    }
    Texts txt = make_texts(is_english_.load());
    RCLCPP_INFO(get_logger(), "%s", txt.log_finished.c_str());
}

void AnimalFaceNode::set_filter(const std::string& animal)
{
    Texts txt = make_texts(is_english_);
    std::lock_guard<std::mutex> lock(filter_mutex_);
    try {
        current_filter_       = AnimalFilter(animal);
        g_animal_filter_ready = true;
        RCLCPP_INFO(get_logger(), "%s%s", txt.log_filter_set.c_str(), animal.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "%s '%s': %s",
                     txt.log_filter_error.c_str(), animal.c_str(), e.what());
    }
}

bool AnimalFaceNode::get_last_frame(cv::Mat& out)
{
    std::lock_guard<std::mutex> lock(filter_mutex_);
    if (!has_frame_) return false;
    out        = last_frame_;
    has_frame_ = false;
    return true;
}

void AnimalFaceNode::callback(
    const ImageMsg::ConstSharedPtr& img_msg,
    const Landmarks::ConstSharedPtr& lm_msg)
{
    Texts txt = make_texts(is_english_);
    try {
        cv::Mat frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;

        std::vector<cv::Point2f> landmarks;
        for (const auto& p : lm_msg->landmarks)
            landmarks.emplace_back(p.x * frame.cols, p.y * frame.rows);

        cv::Mat processed;
        bool mouth_triggered = false;
        bool mouth_turning_on = false;

        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            cv::flip(frame, frame, 0);

            // ── Detección de boca abierta (toggle) ────────────────────
            std::vector<cv::Point2f> lm_norm;
            for (const auto& p : lm_msg->landmarks)
                lm_norm.emplace_back(p.x, p.y);

            if (tongue_detector_) {
                auto result = tongue_detector_->check(lm_norm);
                if (result.triggered) {
                    // Fondo: toggle igual que antes
                    virtual_bg_->set_active(result.turning_on);

                    // Audio: guardar para reproducir/detener fuera del mutex
                    mouth_triggered  = true;
                    mouth_turning_on = result.turning_on;

                    RCLCPP_INFO(get_logger(),
                        result.turning_on
                            ? "Boca abierta! Fondo ON + sonido ON."
                            : "Boca abierta! Fondo OFF + sonido OFF.");
                }
            }

            // ── Aplicar fondo virtual ──────────────────────────────────
            if (virtual_bg_) {
                frame = virtual_bg_->apply(frame);
            }

            // ── Aplicar filtro de animal ───────────────────────────────
            if (g_animal_filter_ready.load()) {
                processed = current_filter_.apply_filter(frame, landmarks);
            } else {
                processed = frame.clone();
            }
        }

        // ── Toggle de audio fuera del mutex ───────────────────────────
        if (mouth_triggered) {
            play_sound(mouth_turning_on);
        }

        const int TW = 800, TH = 480;
        double sc = std::max((double)TW/processed.cols, (double)TH/processed.rows);
        cv::Mat zoomed;
        cv::resize(processed, zoomed, cv::Size(), sc, sc, cv::INTER_AREA);
        int x = (zoomed.cols-TW)/2, y = (zoomed.rows-TH)/2;
        processed = zoomed(cv::Rect(x,y,TW,TH));

        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            last_frame_ = processed.clone();
            has_frame_  = true;
        }

        image_pub_->publish(
            *cv_bridge::CvImage(img_msg->header, "bgr8", processed).toImageMsg());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "%s%s", txt.log_cb_error.c_str(), e.what());
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnimalFaceNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}