#include "AnimalFaceNode.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include "ament_index_cpp/get_package_share_directory.hpp"

// ═══════════════════════════════════════════════════════════════
//  CONSTANTES DE LAYOUT  (800 × 480 — coordenadas lógicas)
//  La ventana es FULLSCREEN pero dibujamos en canvas 800×480
//  y luego escalamos si hace falta.
// ═══════════════════════════════════════════════════════════════
static const int  WIN_W      = 800;
static const int  WIN_H      = 480;
static const char MENU_WIN[] = "Selecciona tu filtro";
static const char CAM_WIN[]  = "Animal Filter";

// Tarjetas
static const int COLS       = 3;
static const int CARD_W     = 210;
static const int CARD_H     = 310;   // más altas para la imagen PNG
static const int CARD_PAD   = 30;
static const int TOP_OFFSET = 90;
static const cv::Rect BTN_PREV(10, TOP_OFFSET + CARD_H/2 - 25, 40, 50);
static const cv::Rect BTN_NEXT(WIN_W - 50, TOP_OFFSET + CARD_H/2 - 25, 40, 50);
// Botón Aplicar
static const cv::Rect BTN_APPLY(300, 430, 200, 48);

// Botón X (cerrar menú) — esquina superior derecha del canvas
static const cv::Rect BTN_CLOSE(WIN_W - 55, 10, 45, 45);

// ═══════════════════════════════════════════════════════════════
//  DATOS DE LOS FILTROS
// ═══════════════════════════════════════════════════════════════
struct FilterOption {
    std::string id;
    std::string label;
    cv::Scalar  border_color;   // BGR, borde de la tarjeta
};

static const std::vector<FilterOption> FILTERS = {
    { "bear",   "Oso",   { 43,  130, 210 } },   // naranja-marrón
    { "cat",    "Gato",  { 200,  80,  80 } },   // azul-violeta
    { "monkey", "Mono",  {  40, 180,  60 } }    // verde
};


// ═══════════════════════════════════════════════════════════════
//  OVERLAY RGBA sobre BGR (para pegar los PNG con canal alpha)
// ═══════════════════════════════════════════════════════════════
static void overlay_rgba(cv::Mat& bg, const cv::Mat& fg, int ox, int oy)
{
    if (fg.empty()) return;
    int y1 = std::max(oy, 0),          y2 = std::min(oy + fg.rows, bg.rows);
    int x1 = std::max(ox, 0),          x2 = std::min(ox + fg.cols, bg.cols);
    if (x1 >= x2 || y1 >= y2) return;

    cv::Mat fg_crop = fg(cv::Rect(x1-ox, y1-oy, x2-x1, y2-y1));
    cv::Mat bg_crop = bg(cv::Rect(x1,    y1,     x2-x1, y2-y1));

    std::vector<cv::Mat> ch;
    cv::split(fg_crop, ch);
    bool has_alpha = (ch.size() == 4);

    cv::Mat alpha, inv;
    if (has_alpha) {
        ch[3].convertTo(alpha, CV_32F, 1.0/255.0);
        inv = 1.0f - alpha;
    }

    // Convertir fondo a float
    cv::Mat bg_f; bg_crop.convertTo(bg_f, CV_32FC3, 1.0/255.0);

    cv::Mat fg_bgr;
    if (fg_crop.channels() == 4)
        cv::cvtColor(fg_crop, fg_bgr, cv::COLOR_BGRA2BGR);
    else
        fg_bgr = fg_crop;
    cv::Mat fg_f; fg_bgr.convertTo(fg_f, CV_32FC3, 1.0/255.0);

    cv::Mat out_f;
    if (has_alpha) {
        // Blend per-channel
        std::vector<cv::Mat> bg_ch, fg_ch_3;
        cv::split(bg_f, bg_ch);
        cv::split(fg_f, fg_ch_3);
        std::vector<cv::Mat> res(3);
        for (int c = 0; c < 3; ++c)
            res[c] = fg_ch_3[c].mul(alpha) + bg_ch[c].mul(inv);
        cv::merge(res, out_f);
    } else {
        out_f = fg_f;
    }

    cv::Mat out_u8; out_f.convertTo(out_u8, CV_8UC3, 255.0);
    out_u8.copyTo(bg_crop);
}

// ═══════════════════════════════════════════════════════════════
//  HELPERS DE LAYOUT
// ═══════════════════════════════════════════════════════════════
static cv::Rect card_rect(int idx)
{
    int col     = idx % COLS;
    int row     = idx / COLS;
    int total_w = COLS * CARD_W + (COLS - 1) * CARD_PAD;
    int x0      = (WIN_W - total_w) / 2 + col * (CARD_W + CARD_PAD);
    int y0      = TOP_OFFSET + row * (CARD_H + CARD_PAD);
    return { x0, y0, CARD_W, CARD_H };
}
// ═══════════════════════════════════════════════════════════════
//  CARGA DE PREVIEWS PNG
//  Resize proporcional + centrado + recorte transparente
// ═══════════════════════════════════════════════════════════════

static cv::Mat crop_transparent(const cv::Mat& img)
{
    if (img.empty())
        return img;

    // Solo aplica a imágenes RGBA
    if (img.channels() != 4)
        return img;

    std::vector<cv::Mat> ch;
    cv::split(img, ch);

    cv::Mat alpha = ch[3];

    // Buscar bounding box del contenido visible
    cv::Rect bbox = cv::boundingRect(alpha);

    // Validación
    if (bbox.width <= 0 || bbox.height <= 0)
        return img;

    return img(bbox).clone();
}

static std::vector<cv::Mat> load_previews()
{
    std::vector<cv::Mat> previews(FILTERS.size());

    try {

        std::string pkg =
            ament_index_cpp::get_package_share_directory("yaren_filters");

        for (int i = 0; i < (int)FILTERS.size(); ++i) {

            std::string path =
                pkg +
                "/imgs/animals_mask/" +
                FILTERS[i].id +
                "/" +
                FILTERS[i].id +
                "_open_open.png";

            cv::Mat img =
                cv::imread(path, cv::IMREAD_UNCHANGED);

            if (img.empty()) {
                std::cerr << "No se pudo cargar: " << path << std::endl;
                continue;
            }

            // ═══════════════════════════════════════
            // RECORTAR ESPACIO TRANSPARENTE
            // ═══════════════════════════════════════
            img = crop_transparent(img);

            // ═══════════════════════════════════════
            // TAMAÑO MÁXIMO DENTRO DE LA TARJETA
            // ═══════════════════════════════════════
            int max_w = CARD_W - 30;
            int max_h = CARD_H - 95;

            // Mantener aspect ratio
            double scale = std::min(
                (double)max_w / img.cols,
                (double)max_h / img.rows
            );

            int new_w = static_cast<int>(img.cols * scale);
            int new_h = static_cast<int>(img.rows * scale);

            // Resize proporcional
            cv::resize(
                img,
                previews[i],
                cv::Size(new_w, new_h),
                0,
                0,
                cv::INTER_AREA
            );
        }

    } catch (const std::exception& e) {

        std::cerr << "Error cargando previews: "
                  << e.what() << std::endl;
    }

    return previews;
}
// ═══════════════════════════════════════════════════════════════
//  DIBUJO DEL MENÚ
// ═══════════════════════════════════════════════════════════════
static void draw_card(cv::Mat& canvas, int idx, bool selected,
                      const std::vector<cv::Mat>& previews)
{
    const FilterOption& f = FILTERS[idx];
    cv::Rect r = card_rect(idx);

    // Fondo tarjeta
    cv::Scalar bg_col = selected ? cv::Scalar(55, 55, 65) : cv::Scalar(28, 28, 35);
    cv::rectangle(canvas, r, bg_col, cv::FILLED);

    // Sombra interior si seleccionado
    if (selected) {
        cv::rectangle(canvas,
                      cv::Rect(r.x+2, r.y+2, r.width-4, r.height-4),
                      f.border_color * 0.3, cv::FILLED);
    }

    // Borde (más grueso si seleccionado)
    cv::rectangle(canvas, r, f.border_color, selected ? 4 : 1);

    // ── Imagen PNG del animal ────────────────────────────────
    cv::Rect img_area(r.x + 10, r.y + 10, CARD_W - 20, CARD_H - 75);
    cv::rectangle(canvas, img_area, cv::Scalar(18, 18, 22), cv::FILLED);

    if (!previews[idx].empty()) {

    // Centrar preview dentro del área
    int px =
        img_area.x +
        (img_area.width - previews[idx].cols) / 2;

    int py =
        img_area.y +
        (img_area.height - previews[idx].rows) / 2;

    overlay_rgba(canvas, previews[idx], px, py);
    } else {
        // Fallback: círculo simple si no cargó la imagen
        cv::Point c(r.x + CARD_W/2, r.y + 10 + (CARD_H-75)/2);
        cv::circle(canvas, c, 60, f.border_color, cv::FILLED);
        cv::circle(canvas, c + cv::Point(-20,-10), 10, {255,255,255}, cv::FILLED);
        cv::circle(canvas, c + cv::Point( 20,-10), 10, {255,255,255}, cv::FILLED);
        cv::circle(canvas, c + cv::Point(-20,-10),  5, {0,0,0},       cv::FILLED);
        cv::circle(canvas, c + cv::Point( 20,-10),  5, {0,0,0},       cv::FILLED);
    }

    // ── Etiqueta centrada ────────────────────────────────────
    int base = 0;
    cv::Size ts = cv::getTextSize(f.label, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &base);
    cv::Scalar txt = selected ? cv::Scalar(255,255,255) : cv::Scalar(160,160,170);
    cv::putText(canvas, f.label,
                { r.x + (CARD_W - ts.width)/2, r.y + CARD_H - 18 },
                cv::FONT_HERSHEY_SIMPLEX, 0.9, txt, 2, cv::LINE_AA);

    // Subrayado si seleccionado
    if (selected) {
        int ux = r.x + (CARD_W - ts.width)/2;
        int uy = r.y + CARD_H - 10;
        cv::line(canvas, {ux, uy}, {ux + ts.width, uy}, f.border_color, 2);
    }
}
static void draw_navigation_arrows(cv::Mat& canvas) {
    // Como por ahora solo hay 3 y caben en pantalla, las dibujaremos 
    // con un color tenue, listas para cuando la lista crezca.
    cv::Scalar arrow_col(100, 100, 110); 
    
    // Flecha Izquierda (<)
    std::vector<cv::Point> pts_left = {
        {BTN_PREV.x + 30, BTN_PREV.y},
        {BTN_PREV.x + 10, BTN_PREV.y + 25},
        {BTN_PREV.x + 30, BTN_PREV.y + 50}
    };
    cv::polylines(canvas, pts_left, false, arrow_col, 3, cv::LINE_AA);

    // Flecha Derecha (>)
    std::vector<cv::Point> pts_right = {
        {BTN_NEXT.x + 10, BTN_NEXT.y},
        {BTN_NEXT.x + 30, BTN_NEXT.y + 25},
        {BTN_NEXT.x + 10, BTN_NEXT.y + 50}
    };
    cv::polylines(canvas, pts_right, false, arrow_col, 3, cv::LINE_AA);
}
static void draw_menu(cv::Mat& canvas, int selected_idx,
                      const std::vector<cv::Mat>& previews)
{
    // Fondo y cabecera
    canvas.setTo(cv::Scalar(12, 12, 18));
    cv::rectangle(canvas, {0, 0, WIN_W, 75}, cv::Scalar(20, 20, 30), cv::FILLED);

    // Título
    const std::string title = "Elige tu filtro de animal";
    int base = 0;
    cv::Size ts = cv::getTextSize(title, cv::FONT_HERSHEY_SIMPLEX, 1.05, 2, &base);
    cv::putText(canvas, title, {(WIN_W - ts.width)/2, 55},
                cv::FONT_HERSHEY_SIMPLEX, 1.05, {220, 220, 230}, 2, cv::LINE_AA);

    cv::line(canvas, {40, 72}, {WIN_W-40, 72}, {50, 50, 65}, 2);

    // Dibujar Flechas laterales
    draw_navigation_arrows(canvas);

    // Tarjetas
    for (int i = 0; i < (int)FILTERS.size(); ++i)
        draw_card(canvas, i, i == selected_idx, previews);

    // Botón Aplicar
    bool has_sel   = (selected_idx >= 0);
    cv::Scalar bbg = has_sel ? cv::Scalar(0, 160, 0)    : cv::Scalar(45, 45, 55);
    cv::Scalar bft = has_sel ? cv::Scalar(255,255,255)   : cv::Scalar(90, 90, 100);

    cv::rectangle(canvas, cv::Rect(BTN_APPLY.x+3, BTN_APPLY.y+3, BTN_APPLY.width, BTN_APPLY.height),
                  cv::Scalar(0,0,0), cv::FILLED);
    cv::rectangle(canvas, BTN_APPLY, bbg, cv::FILLED);
    
    const std::string btn_txt = "Aplicar";
    cv::Size bs = cv::getTextSize(btn_txt, cv::FONT_HERSHEY_SIMPLEX, 0.85, 2, &base);
    cv::putText(canvas, btn_txt,
                {BTN_APPLY.x + (BTN_APPLY.width - bs.width)/2,
                 BTN_APPLY.y + (BTN_APPLY.height + bs.height)/2 - 2},
                cv::FONT_HERSHEY_SIMPLEX, 0.85, bft, 2, cv::LINE_AA);

    // Botón X
    cv::rectangle(canvas, BTN_CLOSE, cv::Scalar(180, 30, 30), cv::FILLED);
    int m = 12;
    cv::line(canvas, {BTN_CLOSE.x+m, BTN_CLOSE.y+m}, {BTN_CLOSE.x+BTN_CLOSE.width-m, BTN_CLOSE.y+BTN_CLOSE.height-m}, {255,255,255}, 3, cv::LINE_AA);
    cv::line(canvas, {BTN_CLOSE.x+BTN_CLOSE.width-m, BTN_CLOSE.y+m}, {BTN_CLOSE.x+m, BTN_CLOSE.y+BTN_CLOSE.height-m}, {255,255,255}, 3, cv::LINE_AA);

    // SE ELIMINÓ EL "HINT INFERIOR" (Haz clic en la ventana...)
}

// ═══════════════════════════════════════════════════════════════
//  MOUSE CALLBACKS
// ═══════════════════════════════════════════════════════════════
struct MenuState {
    int  selected   = -1;
    bool apply_hit  = false;
    bool close_hit  = false;
};

static void on_menu_mouse(int event, int x, int y, int, void* ud)
{
    auto* s = reinterpret_cast<MenuState*>(ud);
    if (event != cv::EVENT_LBUTTONDOWN) return;

    if (BTN_CLOSE.contains({x, y})) { s->close_hit = true; return; }

    // Click en Flechas (lógica para scroll futuro)
    if (BTN_PREV.contains({x, y})) { /* Aquí moverías el offset de la lista */ return; }
    if (BTN_NEXT.contains({x, y})) { /* Aquí moverías el offset de la lista */ return; }

    for (int i = 0; i < (int)FILTERS.size(); ++i)
        if (card_rect(i).contains({x, y})) { s->selected = i; return; }

    if (BTN_APPLY.contains({x, y}) && s->selected >= 0)
        s->apply_hit = true;
}

static void on_cam_mouse(int event, int, int, int, void* ud)
{
    if (event == cv::EVENT_LBUTTONDOWN)
        *reinterpret_cast<std::atomic<bool>*>(ud) = true;
}

// ═══════════════════════════════════════════════════════════════
//  show_menu() — hilo principal, bloquea hasta Aplicar o Cerrar
//  Retorna "" si el usuario cerró el menú sin elegir.
// ═══════════════════════════════════════════════════════════════
static std::string show_menu(const std::vector<cv::Mat>& previews)
{
    // Pantalla completa sin barra de título
    cv::namedWindow(MENU_WIN, cv::WINDOW_NORMAL);
    cv::setWindowProperty(MENU_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

    MenuState ms;
    cv::setMouseCallback(MENU_WIN, on_menu_mouse, &ms);

    cv::Mat canvas(WIN_H, WIN_W, CV_8UC3);

    while (!ms.apply_hit && !ms.close_hit) {
        draw_menu(canvas, ms.selected, previews);
        cv::imshow(MENU_WIN, canvas);
        cv::waitKey(16);
    }

    cv::destroyWindow(MENU_WIN);

    if (ms.close_hit || ms.selected < 0) return "";
    return FILTERS[ms.selected].id;
}

// ═══════════════════════════════════════════════════════════════
//  AnimalFaceNode — CONSTRUCTOR
// ═══════════════════════════════════════════════════════════════
AnimalFaceNode::AnimalFaceNode()
    : Node("filtro_animales"),
      current_filter_("bear")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    image_sub_.subscribe(this, "/csi_camera/image_raw");
    landmarks_sub_.subscribe(this, "face_landmarks");

    sync_ = std::make_shared<Synchronizer>(
        ApproximateTimePolicy(10), image_sub_, landmarks_sub_);
    sync_->registerCallback(&AnimalFaceNode::callback, this);

    image_pub_ = image_transport::create_publisher(
        this, "filtered_image", qos.get_rmw_qos_profile());

    RCLCPP_INFO(get_logger(), "Nodo de filtros animales listo (modo mouse).");
}

void AnimalFaceNode::set_filter(const std::string& animal)
{
    std::lock_guard<std::mutex> lock(filter_mutex_);
    try {
        current_filter_ = AnimalFilter(animal);
        RCLCPP_INFO(get_logger(), "Filtro activo: %s", animal.c_str());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error al cargar filtro '%s': %s", animal.c_str(), e.what());
    }
}

bool AnimalFaceNode::get_last_frame(cv::Mat& out)
{
    std::lock_guard<std::mutex> lock(filter_mutex_);
    if (!has_frame_) return false;
    out = last_frame_;
    has_frame_ = false;
    return true;
}

// ═══════════════════════════════════════════════════════════════
//  CALLBACK — hilo del executor. Solo procesa, NO llama imshow.
// ═══════════════════════════════════════════════════════════════
void AnimalFaceNode::callback(
    const ImageMsg::ConstSharedPtr& img_msg,
    const Landmarks::ConstSharedPtr& landmarks_msg)
{
    try {
        cv::Mat frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;

        std::vector<cv::Point2f> landmarks;
        for (const auto& p : landmarks_msg->landmarks)
            landmarks.emplace_back(p.x * frame.cols, p.y * frame.rows);

        cv::Mat processed;
        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            cv::flip(frame, frame, 0);
            processed = current_filter_.apply_filter(frame, landmarks);
        }

        // === ZOOM + RECORTE (sin bordes negros y sin distorsión) ===
        const int TARGET_W = 800;
        const int TARGET_H = 480;

        // Calcular escala para llenar toda la pantalla
        double scale = std::max(
            static_cast<double>(TARGET_W) / processed.cols,
            static_cast<double>(TARGET_H) / processed.rows
        );

        cv::Mat zoomed;
        cv::resize(processed, zoomed, cv::Size(), scale, scale, cv::INTER_AREA);

        // Recortar el centro
        int x = (zoomed.cols - TARGET_W) / 2;
        int y = (zoomed.rows - TARGET_H) / 2;

        processed = zoomed(cv::Rect(x, y, TARGET_W, TARGET_H));
        // ===========================================================

        {
            std::lock_guard<std::mutex> lock(filter_mutex_);
            last_frame_ = processed.clone();
            has_frame_ = true;
        }

        image_pub_.publish(
            cv_bridge::CvImage(img_msg->header, "bgr8", processed).toImageMsg());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Error en callback: %s", e.what());
    }
}
// ═══════════════════════════════════════════════════════════════
//  MAIN — toda la UI vive aquí (hilo principal)
// ═══════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnimalFaceNode>();

    // Cargar previews una sola vez
    std::vector<cv::Mat> previews = load_previews();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    bool running = true;

    while (rclcpp::ok() && running) {
        // 1. Mostrar menú
        std::string chosen = show_menu(previews);

        // Si cerró con la X → salir completamente del modo
        if (chosen.empty()) {
            running = false;
            break;
        }

        node->set_filter(chosen);
        node->cam_clicked_ = false;

        // 2. Ventana de cámara
        cv::namedWindow(CAM_WIN, cv::WINDOW_NORMAL);
        cv::setWindowProperty(CAM_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        cv::setMouseCallback(CAM_WIN, on_cam_mouse, &node->cam_clicked_);

        // 3. Loop de cámara
        while (rclcpp::ok() && !node->cam_clicked_) {
            cv::Mat frame;
            if (node->get_last_frame(frame))
                cv::imshow(CAM_WIN, frame);
            cv::waitKey(16);
        }

        cv::destroyWindow(CAM_WIN);
    }

    // === IMPORTANTE: Avisar al face_screen que salimos ===
    {
        auto mode_pub = node->create_publisher<std_msgs::msg::String>("/yaren_mode", 10);
        auto idle_msg = std_msgs::msg::String();
        idle_msg.data = "idle";
        mode_pub->publish(idle_msg);
        
        // Pequeña espera para que llegue el mensaje
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    RCLCPP_INFO(node->get_logger(), "Filtro de animales finalizado por usuario.");

    executor.cancel();
    if (spin_thread.joinable()) spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
