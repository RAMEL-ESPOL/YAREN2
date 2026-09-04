// AnimalFaceNode.cpp  —  LifecycleNode
#include "AnimalFaceNode.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
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

static cv::Rect card_rect(int idx) {
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

static void draw_card(cv::Mat& canvas, int idx, bool selected,
                      const std::vector<cv::Mat>& previews, const Texts& txt) {
    const FilterOption& f = FILTERS[idx];
    const std::string& label = txt.animal_labels[idx];
    cv::Rect r = card_rect(idx);
    cv::Scalar bg_col = selected ? cv::Scalar(55,55,65) : cv::Scalar(28,28,35);
    cv::rectangle(canvas, r, bg_col, cv::FILLED);
    if (selected)
        cv::rectangle(canvas, cv::Rect(r.x+2,r.y+2,r.width-4,r.height-4),
                      f.border_color * 0.3, cv::FILLED);
    cv::rectangle(canvas, r, f.border_color, selected ? 4 : 1);
    cv::Rect img_area(r.x+10, r.y+10, CARD_W-20, CARD_H-75);
    cv::rectangle(canvas, img_area, cv::Scalar(18,18,22), cv::FILLED);
    if (!previews[idx].empty()) {
        int px = img_area.x + (img_area.width  - previews[idx].cols) / 2;
        int py = img_area.y + (img_area.height - previews[idx].rows) / 2;
        overlay_rgba(canvas, previews[idx], px, py);
    }
    int base = 0;
    cv::Size ts = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &base);
    cv::Scalar tc = selected ? cv::Scalar(255,255,255) : cv::Scalar(160,160,170);
    cv::putText(canvas, label, {r.x+(CARD_W-ts.width)/2, r.y+CARD_H-18},
                cv::FONT_HERSHEY_SIMPLEX, 0.9, tc, 2, cv::LINE_AA);
    if (selected) {
        int ux = r.x+(CARD_W-ts.width)/2, uy = r.y+CARD_H-10;
        cv::line(canvas, {ux,uy}, {ux+ts.width,uy}, f.border_color, 2);
    }
}

static void draw_menu(cv::Mat& canvas, int sel,
                      const std::vector<cv::Mat>& previews, const Texts& txt) {
    canvas.setTo(cv::Scalar(12,12,18));
    cv::rectangle(canvas, {0,0,WIN_W,75}, cv::Scalar(20,20,30), cv::FILLED);
    int base = 0;
    cv::Size ts = cv::getTextSize(txt.menu_title, cv::FONT_HERSHEY_SIMPLEX,1.05,2,&base);
    cv::putText(canvas, txt.menu_title, {(WIN_W-ts.width)/2,55},
                cv::FONT_HERSHEY_SIMPLEX, 1.05, {220,220,230}, 2, cv::LINE_AA);
    cv::line(canvas, {40,72}, {WIN_W-40,72}, {50,50,65}, 2);
    cv::Scalar ac(100,100,110);
    std::vector<cv::Point> l={{BTN_PREV.x+30,BTN_PREV.y},{BTN_PREV.x+10,BTN_PREV.y+25},{BTN_PREV.x+30,BTN_PREV.y+50}};
    std::vector<cv::Point> r={{BTN_NEXT.x+10,BTN_NEXT.y},{BTN_NEXT.x+30,BTN_NEXT.y+25},{BTN_NEXT.x+10,BTN_NEXT.y+50}};
    cv::polylines(canvas, l, false, ac, 3, cv::LINE_AA);
    cv::polylines(canvas, r, false, ac, 3, cv::LINE_AA);
    for (int i = 0; i < (int)FILTERS.size(); ++i)
        draw_card(canvas, i, i==sel, previews, txt);
    bool has_sel = (sel >= 0);
    cv::Scalar bbg = has_sel ? cv::Scalar(0,160,0) : cv::Scalar(45,45,55);
    cv::Scalar bft = has_sel ? cv::Scalar(255,255,255) : cv::Scalar(90,90,100);
    cv::rectangle(canvas, cv::Rect(BTN_APPLY.x+3,BTN_APPLY.y+3,BTN_APPLY.width,BTN_APPLY.height), {0,0,0}, cv::FILLED);
    cv::rectangle(canvas, BTN_APPLY, bbg, cv::FILLED);
    cv::Size bs = cv::getTextSize(txt.btn_apply, cv::FONT_HERSHEY_SIMPLEX, 0.85, 2, &base);
    cv::putText(canvas, txt.btn_apply,
                {BTN_APPLY.x+(BTN_APPLY.width-bs.width)/2, BTN_APPLY.y+(BTN_APPLY.height+bs.height)/2-2},
                cv::FONT_HERSHEY_SIMPLEX, 0.85, bft, 2, cv::LINE_AA);
    cv::rectangle(canvas, BTN_CLOSE, cv::Scalar(180,30,30), cv::FILLED);
    int m=12;
    cv::line(canvas,{BTN_CLOSE.x+m,BTN_CLOSE.y+m},{BTN_CLOSE.x+BTN_CLOSE.width-m,BTN_CLOSE.y+BTN_CLOSE.height-m},{255,255,255},3,cv::LINE_AA);
    cv::line(canvas,{BTN_CLOSE.x+BTN_CLOSE.width-m,BTN_CLOSE.y+m},{BTN_CLOSE.x+m,BTN_CLOSE.y+BTN_CLOSE.height-m},{255,255,255},3,cv::LINE_AA);
}

struct MenuState { int selected=-1; bool apply_hit=false; bool close_hit=false; };

static void on_menu_mouse(int event, int x, int y, int, void* ud) {
    auto* s = reinterpret_cast<MenuState*>(ud);
    if (event != cv::EVENT_LBUTTONDOWN) return;
    if (BTN_CLOSE.contains({x,y})) { s->close_hit = true; return; }
    for (int i = 0; i < (int)FILTERS.size(); ++i)
        if (card_rect(i).contains({x,y})) { s->selected = i; return; }
    if (BTN_APPLY.contains({x,y}) && s->selected >= 0) s->apply_hit = true;
}

static void on_cam_mouse(int event, int, int, int, void* ud) {
    if (event == cv::EVENT_LBUTTONDOWN)
        *reinterpret_cast<std::atomic<bool>*>(ud) = true;
}

static std::string show_menu(const std::vector<cv::Mat>& previews, const Texts& txt) {
    cv::namedWindow(MENU_WIN, cv::WINDOW_NORMAL);
    cv::setWindowProperty(MENU_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    std::thread([&]() {
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
        draw_menu(canvas, ms.selected, previews, txt);
        cv::imshow(MENU_WIN, canvas);
        cv::waitKey(16);
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

void AnimalFaceNode::run_ui()
{
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

    // Al salir del loop, parar audio
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