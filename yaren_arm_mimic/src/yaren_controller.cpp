#include "yaren_controller.hpp"

// ═══════════════════════════════════════════════════════════════════
//  ANÁLISIS DE EJES (derivado del URDF - NO modificar sin reverificar)
//
//  Eje efectivo en frame link_2 (referencia global del torso):
//    joint_5  (hombro derecho  lateral/yx) : [ +1,  0,  0 ]
//    joint_9  (hombro izquierdo lateral/yx): [ -1,  0,  0 ]  ← OPUESTO a joint_5
//    joint_6  (hombro derecho  delante/zy) : [  0,  0, +1 ]
//    joint_10 (hombro izquierdo delante/zy): [  0,  0, -1 ]  ← OPUESTO a joint_6
//    joint_7  (codo derecho    yx, local)  : [ +1,  0,  0 ]
//    joint_11 (codo izquierdo  yx, local)  : [ +1,  0,  0 ]  ← IGUAL a joint_7
//    joint_8  (codo derecho    zy, local)  : [  0,  0, +1 ]
//    joint_12 (codo izquierdo  zy, local)  : [  0,  0, +1 ]  ← IGUAL a joint_8
//
//  body_tracker_node normaliza:
//    left_shoulder_yx  ← YA NEGADO en body_tracker
//    left_elbow_yx     ← YA NEGADO en body_tracker
//    left_shoulder_zy  ← NO negado en body_tracker
//    left_elbow_zy     ← NO negado en body_tracker
//
//  Consecuencias en el controlador para el brazo izquierdo:
//    shoulder_yx : ejes OPUESTOS + body_tracker ya negó → se cancelan → pasar directo
//    shoulder_zy : ejes OPUESTOS + body_tracker NO negó → NEGAR aquí
//    elbow_yx    : ejes IGUALES  + body_tracker ya negó → pasar directo
//    elbow_zy    : ejes IGUALES  + body_tracker NO negó → pasar directo (mismo eje local)
// ═══════════════════════════════════════════════════════════════════

DualArmTrajectoryController::DualArmTrajectoryController() : Node("body_trajectory_controller")
{
    // ── Límites de joints en radianes (copiados del URDF) ─────────────────
    joint_limits_["joint_5"]  = {-10.0f, 10.0f};
    joint_limits_["joint_6"]  = {-10.0f, 10.0f};
    joint_limits_["joint_7"]  = {-10.0f, 10.0f};
    joint_limits_["joint_8"]  = {-10.0f, 10.0f};

    joint_limits_["joint_9"]  = {-10.0f, 10.0f};
    joint_limits_["joint_10"] = {-10.0f, 10.0f};
    joint_limits_["joint_11"] = {-10.0f, 10.0f};
    joint_limits_["joint_12"] = {-10.0f, 10.0f};

    right_joints_ = {"joint_5", "joint_6", "joint_7", "joint_8"};
    left_joints_  = {"joint_9", "joint_10", "joint_11", "joint_12"};

    last_right_pos_ = calculateMidpoints(right_joints_);
    last_left_pos_  = calculateMidpoints(left_joints_);

    torso_tilt_ = 0.0f;

    // ── Variables de estado ────────────────────────────────────────────────
    new_data_available_ = false;
    goal_sent_          = false;

    // ── Todos los joints (1-4: torso/cabeza, 5-8: brazo der., 9-12: brazo izq.) ──
    all_joints_ = {
        "joint_1",  "joint_2",  "joint_3",  "joint_4",
        "joint_5",  "joint_6",  "joint_7",  "joint_8",
        "joint_9",  "joint_10", "joint_11", "joint_12"
    };

    // ── Cliente de acción de trayectoria ──────────────────────────────────
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this, "/joint_trajectory_controller/follow_joint_trajectory");

    // ── Suscripción a BodyPosition ─────────────────────────────────────────
    subscription_ = this->create_subscription<yaren_interfaces::msg::BodyPosition>(
        "body_tracker", 10,
        std::bind(&DualArmTrajectoryController::armTrackerCallback,
                  this, std::placeholders::_1));

    // ── Timer de envío de trayectoria (50 Hz) ─────────────────────────────
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&DualArmTrajectoryController::sendTrajectoryGoal, this));

    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_WARN(this->get_logger(), "Action server no disponible tras 10 segundos");
    } else {
        RCLCPP_INFO(this->get_logger(), "Action server disponible.");
    }

    RCLCPP_INFO(this->get_logger(), "Dual arm trajectory controller iniciado");
}

// ─── Punto medio del rango de cada joint (posición de reposo) ────────────────
std::map<std::string, float> DualArmTrajectoryController::calculateMidpoints(
    const std::vector<std::string>& joints)
{
    std::map<std::string, float> result;
    for (const auto& j : joints) {
        result[j] = (joint_limits_[j].first + joint_limits_[j].second) / 2.0f;
    }
    return result;
}

// ─── FIX #1: Convierte grados → radianes ─────────────────────────────────────
// body_tracker_node usa radian2Euler (×180/π) y publica en GRADOS.
// El action server espera RADIANES.
// Bug original: euler2Radian devolvía el valor sin convertir → 45° se enviaba
// como 45 rad (~2578°), clampeando todos los joints al límite máximo.
float DualArmTrajectoryController::euler2Radian(float euler)
{
    return euler * static_cast<float>(M_PI) / 180.0f;
}

// ─── Clampea la posición al rango del joint ──────────────────────────────────
float DualArmTrajectoryController::limitJointPosition(
    const std::string& joint, float position)
{
    auto it = joint_limits_.find(joint);
    if (it != joint_limits_.end()) {
        return std::max(it->second.first, std::min(it->second.second, position));
    }
    return position;
}

// ─── Mapea 4 ángulos del brazo a posiciones de joint en radianes ─────────────
//
//  Entrada (orden fijo del armTrackerCallback):
//    angles[0] = shoulder_zy  → arm_joints[0] = joint_5  / joint_9
//    angles[1] = shoulder_yx  → arm_joints[1] = joint_6  / joint_10
//    angles[2] = elbow_yx     → arm_joints[2] = joint_7  / joint_11
//    angles[3] = elbow_zy     → arm_joints[3] = joint_8  / joint_12
//
std::map<std::string, float> DualArmTrajectoryController::processArmData(
    const std::array<float, 4>& angles,
    const std::vector<std::string>& arm_joints,
    const bool is_right)
{
    std::map<std::string, float> positions;

    if (is_right) {
        // Brazo derecho: referencia, sin modificaciones
        positions[arm_joints[0]] = euler2Radian( angles[0]);  // shoulder_zy → joint_5
        positions[arm_joints[1]] = euler2Radian( angles[1]);  // shoulder_yx → joint_6
        positions[arm_joints[2]] = euler2Radian( angles[2]);  // elbow_yx    → joint_7
        positions[arm_joints[3]] = euler2Radian( angles[3]);  // elbow_zy    → joint_8

    } else {
        // Brazo izquierdo — ver análisis de ejes al inicio del archivo

        // FIX #2 — shoulder_zy: NEGAR
        //   joint_6 eje en link_2 = +Z | joint_10 eje en link_2 = -Z → OPUESTOS
        //   body_tracker NO normaliza ZY → sin negación el brazo izquierdo se movería
        //   en sentido contrario al derecho para el mismo gesto.
        positions[arm_joints[0]] = euler2Radian(-angles[0]);  // shoulder_zy → joint_9  ← NEGADO

        // shoulder_yx: PASAR DIRECTO
        //   joint_5 eje = +X | joint_9 eje = -X → OPUESTOS
        //   body_tracker YA negó left_shoulder_yx.
        //   Efecto neto: negación BT + ejes opuestos = movimiento simétrico correcto ✓
        positions[arm_joints[1]] = euler2Radian( angles[1]);  // shoulder_yx → joint_10

        // elbow_yx: PASAR DIRECTO
        //   joint_7 y joint_11 mismo eje local (+X).
        //   body_tracker YA negó left_elbow_yx.
        //   Mismo eje local + señal normalizada = misma flexión relativa de codo ✓
        positions[arm_joints[2]] = euler2Radian( angles[2]);  // elbow_yx    → joint_11

        // elbow_zy: PASAR DIRECTO
        //   joint_8 y joint_12 mismo eje local (+Z).
        //   body_tracker no normaliza pero mismo eje → misma rotación relativa ✓
        positions[arm_joints[3]] = euler2Radian( angles[3]);  // elbow_zy    → joint_12
    }

    // Clampear al rango del URDF
    for (const auto& joint : arm_joints) {
        positions[joint] = limitJointPosition(joint, positions[joint]);
    }

    return positions;
}

// ─── Callback: recibe BodyPosition y actualiza targets de joint ──────────────
void DualArmTrajectoryController::armTrackerCallback(
    const yaren_interfaces::msg::BodyPosition::SharedPtr msg)
{
    if (!msg->is_valid) {
        RCLCPP_DEBUG(this->get_logger(), "Datos invalidos, manteniendo posicion actual.");
        return;
    }

    std::array<float, 4> right_angles = {
        msg->right_shoulder_elbow_zy,
        msg->right_shoulder_elbow_yx,
        msg->right_elbow_wrist_yx,
        msg->right_elbow_wrist_zy
    };
    last_right_pos_ = processArmData(right_angles, right_joints_, true);

    std::array<float, 4> left_angles = {
        msg->left_shoulder_elbow_zy,
        msg->left_shoulder_elbow_yx,
        msg->left_elbow_wrist_yx,
        msg->left_elbow_wrist_zy
    };
    last_left_pos_ = processArmData(left_angles, left_joints_, false);

    new_data_available_ = true;
}

// ─── Callbacks del action client ─────────────────────────────────────────────
void DualArmTrajectoryController::goal_response_callback(
    const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal rechazado por el servidor");
        goal_sent_ = false;
    } else {
        RCLCPP_DEBUG(this->get_logger(), "Goal aceptado");
    }
}

void DualArmTrajectoryController::feedback_callback(
    GoalHandleFollowJointTrajectory::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> /*feedback*/)
{
    // Ampliar si se necesita supervisión en tiempo real
}

void DualArmTrajectoryController::result_callback(
    const GoalHandleFollowJointTrajectory::WrappedResult& result)
{
    goal_sent_ = false;

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_DEBUG(this->get_logger(), "Goal completado");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(),  "Goal abortado");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(),  "Goal cancelado");
            break;
        default:
            RCLCPP_WARN(this->get_logger(),  "Resultado desconocido");
            break;
    }
}

// ─── Envía la trayectoria al robot (llamado por timer, 50 Hz) ─────────────────
void DualArmTrajectoryController::sendTrajectoryGoal()
{
    if (!new_data_available_) {
        return;
    }

    auto goal_msg = FollowJointTrajectory::Goal();
    goal_msg.trajectory.joint_names = all_joints_;

    trajectory_msgs::msg::JointTrajectoryPoint point;

    // joints 1-4: torso/cabeza → fijos en 0
    // joints 5-8: brazo derecho
    // joints 9-12: brazo izquierdo
    point.positions = {
        0.0,
        0.0,
        0.0,
        0.0,
        last_right_pos_["joint_5"],
        last_right_pos_["joint_6"],
        last_right_pos_["joint_7"],
        last_right_pos_["joint_8"],
        last_left_pos_["joint_9"],
        last_left_pos_["joint_10"],
        last_left_pos_["joint_11"],
        last_left_pos_["joint_12"],
    };

    point.velocities.resize(point.positions.size(), 0.0);
    point.time_from_start        = rclcpp::Duration::from_seconds(0.15);
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(0.5);
    goal_msg.trajectory.points.push_back(point);

    auto opts = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    opts.goal_response_callback =
        std::bind(&DualArmTrajectoryController::goal_response_callback,
                  this, std::placeholders::_1);
    opts.feedback_callback =
        std::bind(&DualArmTrajectoryController::feedback_callback,
                  this, std::placeholders::_1, std::placeholders::_2);
    opts.result_callback =
        std::bind(&DualArmTrajectoryController::result_callback,
                  this, std::placeholders::_1);

    trajectory_client_->async_send_goal(goal_msg, opts);

    new_data_available_ = false;
    goal_sent_ = true;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DualArmTrajectoryController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}