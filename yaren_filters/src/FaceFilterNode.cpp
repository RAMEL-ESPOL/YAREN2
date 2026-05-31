// face_filter_node.cpp  —  LifecycleNode (Accesorios / Fondos)
// ─────────────────────────────────────────────────────────────
// on_configure  → carga assets, crea filtros (una sola vez)
// on_activate   → crea sync/pub, lanza hilo de UI
// on_deactivate → destruye sync/pub, para hilo de UI
// ─────────────────────────────────────────────────────────────

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
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
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <thread>
#include <mutex>
#include <filesystem>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace message_filters;
using ImageMsg    = sensor_msgs::msg::Image;
using Landmarks   = yaren_interfaces::msg::Landmarks;
using ApproximateTimePolicy =
    sync_policies::ApproximateTime<ImageMsg, Landmarks>;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ── Constantes de layout / structs ───────────────────────────

static const int  WIN_W      = 800;
static const int  WIN_H      = 480;
static const char MENU_WIN[] = "Face Filter Menu";
static const char CAM_WIN[]  = "Face Filter";

struct LangStrings {
    std::string menu_title, apply_btn, categories_title,
                disabled_txt, active_txt, of_txt;
    bool is_eng;
    std::string cat_label(const std::string& id) const {
        if (is_eng) {
            if (id=="hat")     return "Hat";
            if (id=="glasses") return "Glasses";
            if (id=="nose")    return "Nose";
            if (id=="mouth")   return "Mouth";
            if (id=="mask")    return "Mask";
        } else {
            if (id=="hat")     return "Sombrero";
            if (id=="glasses") return "Gafas";
            if (id=="nose")    return "Nariz";
            if (id=="mouth")   return "Boca";
            if (id=="mask")    return "Mascara";
        }
        return id;
    }
};

static LangStrings get_strings(bool en) {
    LangStrings s; s.is_eng = en;
    if (en) { s.menu_title="Configure your accessories"; s.apply_btn="Apply";
              s.categories_title="CATEGORIES"; s.disabled_txt="disabled";
              s.active_txt=" active"; s.of_txt=" of "; }
    else    { s.menu_title="Configura tus accesorios";   s.apply_btn="Aplicar";
              s.categories_title="CATEGORIAS"; s.disabled_txt="desactivado";
              s.active_txt=" activo"; s.of_txt=" de "; }
    return s;
}

struct Category { std::string id, icon_char; cv::Scalar color; std::string assets_subdir; };
static const std::vector<Category> CATS = {
    {"hat",     "HAT",  {50,180,255}, "hats"   },
    {"glasses", "GLASS",{200,100,50}, "glasses"},
    {"nose",    "NOSE", {60,200,100}, "noses"  },
    {"mouth",   "MOUTH",{100,60,220}, "mouths" },
    {"mask",    "MASK", {180,30,180}, "faces"  },
};

struct CatState {
    int  index=-1, max_idx=0;
    bool enabled=false;
    std::vector<cv::Mat> previews;
};

// ── Helpers de dibujo ────────────────────────────────────────

static void overlay_rgba_menu(cv::Mat& bg, const cv::Mat& fg, int ox, int oy) {
    if (fg.empty()) return;
    int y1=std::max(oy,0), y2=std::min(oy+fg.rows,bg.rows);
    int x1=std::max(ox,0), x2=std::min(ox+fg.cols,bg.cols);
    if (x1>=x2||y1>=y2) return;
    cv::Mat fc=fg(cv::Rect(x1-ox,y1-oy,x2-x1,y2-y1));
    cv::Mat bc=bg(cv::Rect(x1,y1,x2-x1,y2-y1));
    std::vector<cv::Mat> ch; cv::split(fc,ch);
    bool ha=(ch.size()==4);
    cv::Mat a32,inv;
    if (ha){ch[3].convertTo(a32,CV_32F,1./255.);inv=1.f-a32;}
    cv::Mat fg_bgr;
    if (fc.channels()==4) cv::cvtColor(fc,fg_bgr,cv::COLOR_BGRA2BGR); else fg_bgr=fc;
    cv::Mat bf,ff;
    bc.convertTo(bf,CV_32FC3,1./255.); fg_bgr.convertTo(ff,CV_32FC3,1./255.);
    cv::Mat out;
    if (ha){std::vector<cv::Mat> bc3,fc3,r(3);cv::split(bf,bc3);cv::split(ff,fc3);
        for(int c=0;c<3;++c) r[c]=fc3[c].mul(a32)+bc3[c].mul(inv);cv::merge(r,out);}
    else out=ff;
    cv::Mat o8; out.convertTo(o8,CV_8UC3,255.); o8.copyTo(bc);
}

static std::vector<cv::Mat> load_thumbs(const std::string& dir, int tw, int th) {
    std::vector<cv::Mat> out;
    try {
        namespace fs = std::filesystem;
        if (!fs::exists(dir)) return out;
        std::vector<std::string> files;
        for (auto& e : fs::directory_iterator(dir))
            if (e.path().extension()==".png"||e.path().extension()==".jpg")
                files.push_back(e.path().string());
        std::sort(files.begin(),files.end());
        for (auto& f : files) {
            cv::Mat img=cv::imread(f,cv::IMREAD_UNCHANGED);
            if (img.empty()) continue;
            if (img.channels()==4){std::vector<cv::Mat> ch;cv::split(img,ch);
                cv::Rect bb=cv::boundingRect(ch[3]);if(bb.area()>0)img=img(bb).clone();}
            double s=std::min((double)tw/img.cols,(double)th/img.rows);
            cv::Mat thumb;
            cv::resize(img,thumb,cv::Size((int)(img.cols*s),(int)(img.rows*s)),0,0,cv::INTER_AREA);
            out.push_back(thumb);
        }
    } catch(...) {}
    return out;
}

struct MenuMouseState {
    int hovered_cat=-1,hovered_arrow=-1,scroll_y=0,max_scroll_y=0,last_mouse_y=0;
    bool apply_hit=false,close_hit=false,is_dragging=false;
};
struct UIRects {
    cv::Rect toggle[5],prev[5],next[5],apply,close;
};
static UIRects g_ui;

static void drawRoundedRect(cv::Mat& img, cv::Rect rec, cv::Scalar color, int radius, int thickness=-1) {
    if (radius<0) radius=0;
    radius=std::min({radius,rec.width/2,rec.height/2});
    if (thickness<0){
        cv::rectangle(img,cv::Point(rec.x+radius,rec.y),cv::Point(rec.x+rec.width-radius,rec.y+rec.height),color,-1,cv::LINE_AA);
        cv::rectangle(img,cv::Point(rec.x,rec.y+radius),cv::Point(rec.x+rec.width,rec.y+rec.height-radius),color,-1,cv::LINE_AA);
        cv::circle(img,cv::Point(rec.x+radius,rec.y+radius),radius,color,-1,cv::LINE_AA);
        cv::circle(img,cv::Point(rec.x+rec.width-radius,rec.y+radius),radius,color,-1,cv::LINE_AA);
        cv::circle(img,cv::Point(rec.x+radius,rec.y+rec.height-radius),radius,color,-1,cv::LINE_AA);
        cv::circle(img,cv::Point(rec.x+rec.width-radius,rec.y+rec.height-radius),radius,color,-1,cv::LINE_AA);
    } else {
        cv::line(img,cv::Point(rec.x+radius,rec.y),cv::Point(rec.x+rec.width-radius,rec.y),color,thickness,cv::LINE_AA);
        cv::line(img,cv::Point(rec.x+radius,rec.y+rec.height),cv::Point(rec.x+rec.width-radius,rec.y+rec.height),color,thickness,cv::LINE_AA);
        cv::line(img,cv::Point(rec.x,rec.y+radius),cv::Point(rec.x,rec.y+rec.height-radius),color,thickness,cv::LINE_AA);
        cv::line(img,cv::Point(rec.x+rec.width,rec.y+radius),cv::Point(rec.x+rec.width,rec.y+rec.height-radius),color,thickness,cv::LINE_AA);
        cv::ellipse(img,cv::Point(rec.x+radius,rec.y+radius),cv::Size(radius,radius),180,0,90,color,thickness,cv::LINE_AA);
        cv::ellipse(img,cv::Point(rec.x+rec.width-radius,rec.y+radius),cv::Size(radius,radius),270,0,90,color,thickness,cv::LINE_AA);
        cv::ellipse(img,cv::Point(rec.x+rec.width-radius,rec.y+rec.height-radius),cv::Size(radius,radius),0,0,90,color,thickness,cv::LINE_AA);
        cv::ellipse(img,cv::Point(rec.x+radius,rec.y+rec.height-radius),cv::Size(radius,radius),90,0,90,color,thickness,cv::LINE_AA);
    }
}

static void draw_menu(cv::Mat& canvas, const std::vector<CatState>& states,
                      MenuMouseState* ms, bool is_english)
{
    LangStrings txt = get_strings(is_english);
    canvas.setTo(cv::Scalar(20,13,13));
    const cv::Scalar BG_HEADER(31,19,19), BORDER_COLOR(58,42,42),
                     TEXT_LIGHT(240,226,226), TEXT_MUTED(160,124,124),
                     APPLY_GREEN(52,101,22), CAT_HEADER_BG(30,14,30);
    static const char* CAT_ICO[] = {"HAT","EYE","NSE","MCH","MSK"};
    static const cv::Scalar ICON_BG[] = {{30,60,160},{20,120,200},{40,160,80},{160,40,120},{100,30,180}};
    static const cv::Scalar ACCENT[]  = {{30,60,160},{20,120,200},{40,160,80},{160,40,120},{100,30,180}};
    static const cv::Scalar ROW_BDR[] = {{30,60,160},{20,120,200},{40,160,80},{160,40,120},{100,30,180}};

    cv::rectangle(canvas,cv::Rect(0,0,WIN_W,55),BG_HEADER,cv::FILLED);
    cv::line(canvas,{0,55},{WIN_W,55},BORDER_COLOR,1);
    drawRoundedRect(canvas,cv::Rect(18,17,20,20),cv::Scalar(74,37,83),4);
    cv::putText(canvas,txt.menu_title,{46,35},cv::FONT_HERSHEY_SIMPLEX,0.6,TEXT_LIGHT,1,cv::LINE_AA);
    g_ui.close=cv::Rect(WIN_W-45,12,32,32);
    drawRoundedRect(canvas,g_ui.close,cv::Scalar(26,26,58),8);
    cv::line(canvas,{g_ui.close.x+10,g_ui.close.y+10},{g_ui.close.x+22,g_ui.close.y+22},cv::Scalar(113,113,248),2,cv::LINE_AA);
    cv::line(canvas,{g_ui.close.x+22,g_ui.close.y+10},{g_ui.close.x+10,g_ui.close.y+22},cv::Scalar(113,113,248),2,cv::LINE_AA);
    const int LEFT_W=380;
    cv::line(canvas,{LEFT_W,55},{LEFT_W,WIN_H-55},BORDER_COLOR,1);
    cv::rectangle(canvas,cv::Rect(0,55,LEFT_W,28),CAT_HEADER_BG,cv::FILLED);
    cv::line(canvas,{0,83},{LEFT_W,83},cv::Scalar(30,30,46),1);
    cv::putText(canvas,txt.categories_title,{14,73},cv::FONT_HERSHEY_SIMPLEX,0.33,TEXT_MUTED,1,cv::LINE_AA);
    int row_y=90;
    for (int i=0;i<(int)CATS.size();++i){
        const auto& cat=CATS[i]; const auto& st=states[i];
        g_ui.toggle[i]=cv::Rect(8,row_y,LEFT_W-16,54);
        if (st.enabled){
            drawRoundedRect(canvas,g_ui.toggle[i],cv::Scalar(ROW_BDR[i][0]/6,ROW_BDR[i][1]/6,ROW_BDR[i][2]/6),8);
            drawRoundedRect(canvas,g_ui.toggle[i],ROW_BDR[i],8,1);
            cv::Rect ib(g_ui.toggle[i].x+10,row_y+10,34,34);
            drawRoundedRect(canvas,ib,ICON_BG[i],8);
            cv::putText(canvas,CAT_ICO[i],{ib.x+2,ib.y+22},cv::FONT_HERSHEY_SIMPLEX,0.28,ACCENT[i],1,cv::LINE_AA);
            cv::putText(canvas,txt.cat_label(cat.id),{g_ui.toggle[i].x+54,row_y+24},cv::FONT_HERSHEY_SIMPLEX,0.48,TEXT_LIGHT,1,cv::LINE_AA);
            char buf[32]; snprintf(buf,sizeof(buf),"%d%s%d",st.index+1,txt.of_txt.c_str(),st.max_idx);            
            cv::putText(canvas,buf,{g_ui.toggle[i].x+54,row_y+40},cv::FONT_HERSHEY_SIMPLEX,0.36,TEXT_MUTED,1,cv::LINE_AA);
            cv::Point cc{g_ui.toggle[i].x+g_ui.toggle[i].width-20,row_y+27};
            cv::circle(canvas,cc,11,ROW_BDR[i],-1,cv::LINE_AA);
            cv::line(canvas,{cc.x-6,cc.y},{cc.x-2,cc.y+5},TEXT_LIGHT,2,cv::LINE_AA);
            cv::line(canvas,{cc.x-2,cc.y+5},{cc.x+6,cc.y-5},TEXT_LIGHT,2,cv::LINE_AA);
        } else {
            drawRoundedRect(canvas,g_ui.toggle[i],cv::Scalar(30,20,30),8);
            cv::Rect ib(g_ui.toggle[i].x+10,row_y+10,34,34);
            cv::Scalar ib_dim(ICON_BG[i][0]/2,ICON_BG[i][1]/2,ICON_BG[i][2]/2);
            drawRoundedRect(canvas,ib,ib_dim,8);
            cv::putText(canvas,CAT_ICO[i],{ib.x+2,ib.y+22},cv::FONT_HERSHEY_SIMPLEX,0.28,TEXT_MUTED,1,cv::LINE_AA);
            cv::putText(canvas,txt.cat_label(cat.id),{g_ui.toggle[i].x+54,row_y+24},cv::FONT_HERSHEY_SIMPLEX,0.48,TEXT_MUTED,1,cv::LINE_AA);
            cv::putText(canvas,txt.disabled_txt,{g_ui.toggle[i].x+54,row_y+40},cv::FONT_HERSHEY_SIMPLEX,0.36,cv::Scalar(90,58,90),1,cv::LINE_AA);
        }
        row_y+=59;
    }
    const int RX=LEFT_W+16,RY=65,RW=WIN_W-RX-16,RH=(WIN_H-55)-RY;
    cv::Rect right_view(RX,RY,RW,RH);
    cv::Mat right_canvas(RH,RW,CV_8UC3,cv::Scalar(20,13,13));
    const int CARD_H=176,CARD_SP=10;
    int active_count=0; for(const auto& s:states) if(s.enabled) active_count++;
    int total_h=active_count*(CARD_H+CARD_SP);
    ms->max_scroll_y=std::max(0,total_h-RH);
    ms->scroll_y=std::clamp(ms->scroll_y,0,ms->max_scroll_y);
    int cur_y=-ms->scroll_y;
    for(int i=0;i<(int)CATS.size();++i){
        g_ui.prev[i]=g_ui.next[i]=cv::Rect(0,0,0,0);
        if (!states[i].enabled) continue;
        if (cur_y+CARD_H<0||cur_y>RH){cur_y+=CARD_H+CARD_SP;continue;}
        const auto& st=states[i];
        cv::Rect card(0,cur_y,RW-15,CARD_H);
        drawRoundedRect(right_canvas,card,cv::Scalar(26,14,26),10);
        drawRoundedRect(right_canvas,card,cv::Scalar(58,42,42),10,1);
        std::string title=txt.cat_label(CATS[i].id)+txt.active_txt;
        std::transform(title.begin(),title.end(),title.begin(),::toupper);
        cv::putText(right_canvas,title,{card.x+12,card.y+18},cv::FONT_HERSHEY_SIMPLEX,0.33,cv::Scalar(160,124,124),1,cv::LINE_AA);
        cv::Rect pbox(card.x+12,card.y+24,card.width-24,100);
        drawRoundedRect(right_canvas,pbox,cv::Scalar(19,19,31),8);
        drawRoundedRect(right_canvas,pbox,cv::Scalar(58,42,42),8,1);
        if (st.index>=0&&st.index<(int)st.previews.size()&&!st.previews[st.index].empty()){
            const cv::Mat& thumb=st.previews[st.index];
            double sc=std::min((double)(pbox.width-16)/thumb.cols,82.0/thumb.rows);
            cv::Mat ts; cv::resize(thumb,ts,cv::Size((int)(thumb.cols*sc),(int)(thumb.rows*sc)),0,0,cv::INTER_AREA);
            overlay_rgba_menu(right_canvas,ts,pbox.x+(pbox.width-ts.cols)/2,pbox.y+(pbox.height-ts.rows)/2);
        }
        cv::Rect bprev(card.x+12,card.y+134,32,32),bnext(card.x+card.width-44,card.y+134,32,32);
        drawRoundedRect(right_canvas,bprev,cv::Scalar(80,36,80),8);
        drawRoundedRect(right_canvas,bnext,cv::Scalar(80,36,80),8);
        cv::putText(right_canvas,"<",{bprev.x+10,bprev.y+22},cv::FONT_HERSHEY_SIMPLEX,0.6,ACCENT[i],2,cv::LINE_AA);
        cv::putText(right_canvas,">",{bnext.x+10,bnext.y+22},cv::FONT_HERSHEY_SIMPLEX,0.6,ACCENT[i],2,cv::LINE_AA);
        char buf[32]; snprintf(buf,sizeof(buf),"%d%s%d",st.index+1,txt.of_txt.c_str(),st.max_idx);
        int tw=cv::getTextSize(buf,cv::FONT_HERSHEY_SIMPLEX,0.45,1,nullptr).width;
        cv::putText(right_canvas,buf,{card.x+(card.width-tw)/2,card.y+154},cv::FONT_HERSHEY_SIMPLEX,0.45,cv::Scalar(144,144,176),1,cv::LINE_AA);
        g_ui.prev[i]=cv::Rect(RX+bprev.x,RY+bprev.y,bprev.width,bprev.height)&right_view;
        g_ui.next[i]=cv::Rect(RX+bnext.x,RY+bnext.y,bnext.width,bnext.height)&right_view;
        cur_y+=CARD_H+CARD_SP;
    }
    if (ms->max_scroll_y>0){
        float vr=(float)RH/total_h; int hh=std::max(30,(int)(RH*vr));
        float sr=(float)ms->scroll_y/ms->max_scroll_y; int hy=(int)(sr*(RH-hh));
        drawRoundedRect(right_canvas,cv::Rect(RW-7,0,6,RH),cv::Scalar(30,20,30),3);
        drawRoundedRect(right_canvas,cv::Rect(RW-7,hy,6,hh),cv::Scalar(80,60,80),3);
    }
    right_canvas.copyTo(canvas(right_view));
    cv::rectangle(canvas,cv::Rect(0,WIN_H-55,WIN_W,55),BG_HEADER,cv::FILLED);
    cv::line(canvas,{0,WIN_H-55},{WIN_W,WIN_H-55},BORDER_COLOR,1);
    int px=14,py=WIN_H-55+13;
    for(int i=0;i<(int)CATS.size();++i){
        if(!states[i].enabled) continue;
        cv::Scalar pbg(ICON_BG[i][0]/3,ICON_BG[i][1]/3,ICON_BG[i][2]/3);
        int lw=cv::getTextSize(txt.cat_label(CATS[i].id),cv::FONT_HERSHEY_SIMPLEX,0.33,1,nullptr).width;
        cv::Rect pill(px,py,lw+26,26);
        drawRoundedRect(canvas,pill,pbg,6);
        cv::circle(canvas,{pill.x+10,pill.y+13},4,ACCENT[i],-1,cv::LINE_AA);
        cv::putText(canvas,txt.cat_label(CATS[i].id),{pill.x+18,pill.y+18},cv::FONT_HERSHEY_SIMPLEX,0.33,ACCENT[i],1,cv::LINE_AA);
        px+=pill.width+6;
    }
    g_ui.apply=cv::Rect(WIN_W-140,WIN_H-45,120,35);
    drawRoundedRect(canvas,g_ui.apply,APPLY_GREEN,8);
    std::vector<cv::Point> tri={{g_ui.apply.x+18,g_ui.apply.y+10},{g_ui.apply.x+18,g_ui.apply.y+25},{g_ui.apply.x+30,g_ui.apply.y+17}};
    cv::fillPoly(canvas,tri,cv::Scalar(255,255,255));
    cv::putText(canvas,txt.apply_btn,{g_ui.apply.x+36,g_ui.apply.y+23},cv::FONT_HERSHEY_SIMPLEX,0.55,cv::Scalar(255,255,255),1,cv::LINE_AA);
}

struct MenuContext { MenuMouseState* ms; std::vector<CatState>* states; };

static void on_menu_mouse(int event, int x, int y, int flags, void* ud) {
    auto* ctx=reinterpret_cast<MenuContext*>(ud);
    auto* ms=ctx->ms; auto* states=ctx->states;
    if (event==cv::EVENT_MOUSEWHEEL){
        int d=cv::getMouseWheelDelta(flags);
        if(d>0) ms->scroll_y-=40; else ms->scroll_y+=40;
        ms->scroll_y=std::clamp(ms->scroll_y,0,ms->max_scroll_y); return;
    }
    if (event==cv::EVENT_MOUSEMOVE){
        if(ms->is_dragging){int dy=y-ms->last_mouse_y;ms->scroll_y-=dy;
            ms->scroll_y=std::clamp(ms->scroll_y,0,ms->max_scroll_y);ms->last_mouse_y=y;} return;
    }
    if (event==cv::EVENT_LBUTTONUP){ms->is_dragging=false; return;}
    if (event!=cv::EVENT_LBUTTONDOWN) return;
    if (g_ui.close.contains({x,y})){ms->close_hit=true;return;}
    if (g_ui.apply.contains({x,y})){ms->apply_hit=true;return;}
    for(int i=0;i<(int)CATS.size();++i){
        if(g_ui.toggle[i].contains({x,y})){
            states->at(i).enabled=!states->at(i).enabled;
            if(states->at(i).enabled){
                if(i==4){for(int j=0;j<4;++j)states->at(j).enabled=false;}
                else if(states->size()>4) states->at(4).enabled=false;
            }
            if(states->at(i).enabled&&states->at(i).index<0&&states->at(i).max_idx>0)
                states->at(i).index=0;
            return;
        }
        if(states->at(i).enabled){
            if(g_ui.prev[i].contains({x,y})){auto& s=states->at(i);s.index=(s.index-1+s.max_idx)%s.max_idx;return;}
            if(g_ui.next[i].contains({x,y})){auto& s=states->at(i);s.index=(s.index+1)%s.max_idx;return;}
        }
    }
    if(x>380&&ms->max_scroll_y>0){ms->is_dragging=true;ms->last_mouse_y=y;}
}

static void on_cam_mouse(int event,int,int,int,void* ud){
    if(event==cv::EVENT_LBUTTONDOWN)*reinterpret_cast<std::atomic<bool>*>(ud)=true;
}

// ══════════════════════════════════════════════════════════════
//  FaceFilterNode — LifecycleNode
// ══════════════════════════════════════════════════════════════
class FaceFilterNode : public rclcpp_lifecycle::LifecycleNode {
public:
    std::atomic<bool> is_english_{ false };

    FaceFilterNode() : rclcpp_lifecycle::LifecycleNode("face_filter_node") {}

    // ── Lifecycle callbacks ──────────────────────────────────

    CallbackReturn on_configure(const rclcpp_lifecycle::State&) override {
        auto lang_qos = rclcpp::QoS(1).transient_local();
        language_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/yaren/is_english", lang_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                is_english_ = msg->data;
                RCLCPP_INFO(get_logger(), "Idioma: %s", is_english_ ? "English" : "Español");
            });

        std::string ap = ament_index_cpp::get_package_share_directory("yaren_filters") + "/imgs";
        std::vector<std::string> subdirs = {"hats","glasses","noses","mouths","faces"};
        states_.resize(CATS.size());
        for (int i = 0; i < (int)CATS.size(); ++i) {
            states_[i].previews = load_thumbs(ap + "/" + subdirs[i], 100, 70);
            states_[i].max_idx  = (int)states_[i].previews.size();
            states_[i].index    = states_[i].max_idx > 0 ? 0 : -1;
            states_[i].enabled  = false;
        }

        glasses_filter_    = std::make_shared<GlassesFilter>  (ap + "/glasses");
        mouth_filter_      = std::make_shared<MouthFilter>    (ap + "/mouths");
        nose_filter_       = std::make_shared<NoseFilter>     (ap + "/noses");
        hat_filter_        = std::make_shared<HatFilter>      (ap + "/hats");
        face_mask_filter_  = std::make_shared<FaceMaskFilter> (ap + "/faces");

        RCLCPP_INFO(get_logger(), "Configurado. Assets cargados en RAM.");
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("/yaren_mode", 10);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State&) override {
        // Aprovechamos para usar el QoS correcto de la cámara (sensor_data)
        image_sub_.subscribe(this, "/csi_camera/image_raw", rmw_qos_profile_sensor_data);
        landmarks_sub_.subscribe(this, "/face_landmarks");

        sync_ = std::make_shared<Synchronizer<ApproximateTimePolicy>>(
            ApproximateTimePolicy(10), image_sub_, landmarks_sub_);
        sync_->registerCallback(&FaceFilterNode::callback, this);

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("filtered_image", 10);
        image_pub_->on_activate();

        // Resetear estados de UI
        for (auto& s : states_) s.enabled = false;
        
        // ¡LA SOLUCIÓN MÁGICA AQUÍ ABAJO! 
        // Inicializamos la lista activa para que no explote cuando llegue el primer frame
        active_states_ = states_; 

        cam_clicked_ = false;
        ui_running_  = true;
        ui_thread_   = std::thread(&FaceFilterNode::run_ui, this);

        RCLCPP_INFO(get_logger(), "ACTIVADO.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override {
        ui_running_  = false;
        cam_clicked_ = true;
        if (ui_thread_.joinable()) ui_thread_.join();

        sync_.reset();
        if (image_pub_) {
            image_pub_->on_deactivate();
            image_pub_.reset();
        }
        cv::destroyAllWindows();

        RCLCPP_INFO(get_logger(), "DESACTIVADO.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State&) override {
        language_sub_.reset();
        glasses_filter_.reset(); mouth_filter_.reset();
        nose_filter_.reset();    hat_filter_.reset();
        face_mask_filter_.reset();
        states_.clear();
        RCLCPP_INFO(get_logger(), "Limpieza completada.");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override {
        ui_running_ = false; cam_clicked_ = true;
        if (ui_thread_.joinable()) ui_thread_.join();
        cv::destroyAllWindows();
        return CallbackReturn::SUCCESS;
    }

private:
    void run_ui() {
        while (rclcpp::ok() && ui_running_) {
            bool applied = show_menu_local();
            if (!applied || !ui_running_) break;

            apply_states();
            cam_clicked_ = false;

            cv::namedWindow(CAM_WIN, cv::WINDOW_NORMAL);
            cv::setWindowProperty(CAM_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
            std::thread([]() {
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                std::system("xdotool search --sync --name 'Face Filter' windowactivate --sync windowraise 2>/dev/null");
            }).detach();
            cv::setMouseCallback(CAM_WIN, on_cam_mouse, &cam_clicked_);

            while (rclcpp::ok() && !cam_clicked_ && ui_running_) {
                cv::Mat frame;
                if (get_last_frame(frame)) cv::imshow(CAM_WIN, frame);
                cv::waitKey(16);
            }
            cv::destroyWindow(CAM_WIN);
        }
        if (rclcpp::ok() && ui_running_.load() && mode_pub_) {
            auto msg = std_msgs::msg::String();
            msg.data = "idle";
            mode_pub_->publish(msg);
        }
    }

    bool show_menu_local() {
        cv::namedWindow(MENU_WIN, cv::WINDOW_NORMAL);
        cv::setWindowProperty(MENU_WIN, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
        std::thread([]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
            std::system("xdotool search --sync --name 'Face Filter Menu' windowactivate --sync windowraise 2>/dev/null");
        }).detach();
        MenuMouseState ms;
        MenuContext ctx{&ms, &states_};
        cv::setMouseCallback(MENU_WIN, on_menu_mouse, &ctx);
        cv::Mat canvas(WIN_H, WIN_W, CV_8UC3);
        while (!ms.apply_hit && !ms.close_hit && ui_running_) {
            draw_menu(canvas, states_, &ms, is_english_.load());
            cv::imshow(MENU_WIN, canvas);
            cv::setWindowProperty(MENU_WIN, cv::WND_PROP_TOPMOST, 1);
            cv::waitKey(16);
        }
        cv::destroyWindow(MENU_WIN);
        return ms.apply_hit;
    }

    void apply_states() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < (int)CATS.size(); ++i) {
            if (!states_[i].enabled || states_[i].index < 0) continue;
            const std::string& id = CATS[i].id;
            if      (id=="hat")     hat_filter_->setCurrentIndex(states_[i].index);
            else if (id=="glasses") glasses_filter_->setCurrentIndex(states_[i].index);
            else if (id=="nose")    nose_filter_->setCurrentIndex(states_[i].index);
            else if (id=="mouth")   mouth_filter_->setCurrentIndex(states_[i].index);
            else if (id=="mask")    face_mask_filter_->setCurrentIndex(states_[i].index);
        }
        active_states_ = states_;
    }

    bool get_last_frame(cv::Mat& out) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_frame_) return false;
        out = last_frame_; has_frame_ = false; return true;
    }

    void callback(const ImageMsg::ConstSharedPtr& img_msg,
                  const Landmarks::ConstSharedPtr& lm_msg)
    {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
            std::vector<cv::Point2f> landmarks;
            for (auto& p : lm_msg->landmarks)
                landmarks.emplace_back(p.x*frame.cols, p.y*frame.rows);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                cv::flip(frame, frame, 0);
                if (!landmarks.empty()) {
                    bool mask_on = (active_states_.size()>4 && active_states_[4].enabled);
                    if (mask_on) {
                        frame = face_mask_filter_->applyFilter(frame, landmarks, frame.size());
                    } else {
                        for (int i=0;i<(int)CATS.size();++i){
                            if(!active_states_[i].enabled) continue;
                            const std::string& id=CATS[i].id;
                            if      (id=="hat")     frame=hat_filter_->applyFilter(frame,landmarks,frame.size());
                            else if (id=="glasses") frame=glasses_filter_->applyFilter(frame,landmarks,frame.size());
                            else if (id=="nose")    frame=nose_filter_->applyFilter(frame,landmarks,frame.size());
                            else if (id=="mouth")   frame=mouth_filter_->applyFilter(frame,landmarks,frame.size());
                        }
                    }
                }
            }
            const int TW=WIN_W, TH=WIN_H;
            double sc=std::max((double)TW/frame.cols,(double)TH/frame.rows);
            cv::Mat zoomed; cv::resize(frame,zoomed,cv::Size(),sc,sc,cv::INTER_AREA);
            int cx=(zoomed.cols-TW)/2, cy=(zoomed.rows-TH)/2;
            cv::Mat cropped=zoomed(cv::Rect(cx,cy,TW,TH));
            {
                std::lock_guard<std::mutex> lock(mutex_);
                last_frame_=cropped.clone(); has_frame_=true;
            }
            if (image_pub_ && image_pub_->is_activated()) {
                image_pub_->publish(*cv_bridge::CvImage(img_msg->header, "bgr8", cropped).toImageMsg());
            }        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr language_sub_;
    std::shared_ptr<GlassesFilter>   glasses_filter_;
    std::shared_ptr<MouthFilter>     mouth_filter_;
    std::shared_ptr<NoseFilter>      nose_filter_;
    std::shared_ptr<HatFilter>       hat_filter_;
    std::shared_ptr<FaceMaskFilter>  face_mask_filter_;

    message_filters::Subscriber<ImageMsg,    rclcpp_lifecycle::LifecycleNode> image_sub_;
    message_filters::Subscriber<Landmarks,   rclcpp_lifecycle::LifecycleNode> landmarks_sub_;
    std::shared_ptr<Synchronizer<ApproximateTimePolicy>> sync_;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
    std::mutex                       mutex_;
    std::vector<CatState>            states_;
    std::vector<CatState>            active_states_;
    cv::Mat                          last_frame_;
    bool                             has_frame_{ false };
    std::thread                      ui_thread_;
    std::atomic<bool>                ui_running_{ false };
    std::atomic<bool>                cam_clicked_{ false };
};

// ══════════════════════════════════════════════════════════════
//  MAIN
// ══════════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceFilterNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}