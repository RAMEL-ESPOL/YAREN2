#ifndef KINEMATIC_VALIDATOR_HPP
#define KINEMATIC_VALIDATOR_HPP

#include <cmath>
#include <array>
#include <iostream>

/**
 * @brief Validador de cinemática 3D para mapeo humano-robot
 * 
 * Implementa:
 * 1. Corrección de distorsión perspectiva
 * 2. Validación de longitudes de miembros
 * 3. Cálculo robusto de ángulos esféricos
 * 4. Detección de singularidades cinemáticas
 */

namespace kinematic_validator {

// Parámetros antropométricos humanos (ajustables)
struct HumanLimbParams {
    float upper_arm_min = 0.25f;    // metros
    float upper_arm_max = 0.35f;    // metros
    float forearm_min = 0.20f;      // metros
    float forearm_max = 0.30f;      // metros
    
    // Rangos de confianza para profundidad relativa
    float depth_validity_threshold = 0.05f;  // Rechazo si Z cambia > 5cm entre frame
};

// Parámetros de límites articulares del robot
struct RobotJointLimits {
    // Shoulder azimuth (joint_5)
    float shoulder_azimuth_min = -0.75f;
    float shoulder_azimuth_max = 1.50f;
    
    // Shoulder elevation (joint_6) 
    float shoulder_elevation_min = 0.05f;
    float shoulder_elevation_max = 0.9848f;
    
    // Elbow rotation (joint_7)
    float elbow_rotation_min = -0.6981f;
    float elbow_rotation_max = 0.6981f;
    
    // Wrist elevation (joint_8)
    float wrist_elevation_min = 0.15f;
    float wrist_elevation_max = 1.50f;
};

/**
 * @brief Vector 3D simple
 */
struct Vector3 {
    float x, y, z;
    
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    Vector3 operator-(const Vector3& v) const {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }
    
    float magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    
    Vector3 normalized() const {
        float mag = magnitude();
        if (mag < 1e-6f) return Vector3(0, 0, 0);
        return Vector3(x / mag, y / mag, z / mag);
    }
    
    float dot(const Vector3& v) const {
        return x * v.x + y * v.y + z * v.z;
    }
};

/**
 * @brief Ángulos esféricos (pitch, yaw, roll)
 * pitch: rotación respecto eje X (arriba/abajo)
 * yaw: rotación respecto eje Y (izquierda/derecha)
 * roll: rotación respecto eje Z (giro)
 */
struct SphericalAngles {
    float pitch;  // [-π/2, π/2]
    float yaw;    // [-π, π]
    float roll;   // [-π, π]
};

/**
 * @brief Resultado de validación cinemática
 */
struct KinematicValidationResult {
    bool is_valid;
    std::string error_message;
    float confidence;  // [0, 1] - qué tan confiable es el resultado
};

/**
 * @brief Calcula la magnitud de un vector 3D
 */
inline float magnitude3D(float dx, float dy, float dz) {
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief Valida la longitud de un miembro contra parámetros antropométricos
 * 
 * @param limb_length Longitud calculada del miembro
 * @param expected_min Longitud mínima esperada
 * @param expected_max Longitud máxima esperada
 * @return Resultado de validación
 */
inline KinematicValidationResult validateLimbLength(
    float limb_length,
    float expected_min,
    float expected_max
) {
    KinematicValidationResult result;
    
    if (limb_length < expected_min * 0.9f) {
        result.is_valid = false;
        result.confidence = 0.0f;
        result.error_message = "Limb too short - likely perspective distortion";
        return result;
    }
    
    if (limb_length > expected_max * 1.1f) {
        result.is_valid = false;
        result.confidence = 0.0f;
        result.error_message = "Limb too long - likely perspective distortion";
        return result;
    }
    
    // Calcular confianza basada en qué tan cerca está del rango esperado
    float range = expected_max - expected_min;
    float center = (expected_max + expected_min) / 2.0f;
    float deviation = std::abs(limb_length - center) / (range * 0.5f);
    
    result.is_valid = true;
    result.confidence = std::max(0.0f, 1.0f - (deviation * 0.3f));
    return result;
}

/**
 * @brief Calcula ángulos esféricos desde vector 3D (corrección de perspectiva)
 * 
 * En lugar de proyectar a 2D y usar atan2, usa la geometría esférica completa.
 * Esto es más robusto ante distorsiones perspectivas.
 * 
 * @param v Vector 3D relativo (punto final - punto origen)
 * @return Ángulos esféricos en radianes
 */
inline SphericalAngles calculateSphericalAngles(const Vector3& v) {
    SphericalAngles angles;
    
    float magnitude = v.magnitude();
    
    if (magnitude < 1e-6f) {
        angles.pitch = 0.0f;
        angles.yaw = 0.0f;
        angles.roll = 0.0f;
        return angles;
    }
    
    // Pitch: ángulo respecto al plano horizontal (Y es vertical)
    // pitch = asin(-x / ||v||) 
    angles.pitch = std::asin(-v.x / magnitude);
    
    // Yaw: ángulo en el plano horizontal (Z-Y)
    // yaw = atan2(x, sqrt(y² + z²))
    float horizontal_dist = std::sqrt(v.y * v.y + v.z * v.z);
    angles.yaw = std::atan2(v.x, horizontal_dist);
    
    // Roll: rotación alrededor del eje del brazo (aproximación)
    // En un brazo real, esto viene de la rotación de la muñeca
    // Para simplificar, usamos atan2(z, y)
    angles.roll = std::atan2(v.z, v.y);
    
    return angles;
}

/**
 * @brief Valida si una postura es alcanzable por el robot
 * 
 * Implementa restricciones cinemáticas básicas:
 * - El codo no puede estar "adentro" del hombro
 * - Ángulos deben estar dentro de límites articulares
 * - No debe haber singularidades (codo = hombro)
 */
inline KinematicValidationResult validateRobotReachability(
    const SphericalAngles& angles,
    const RobotJointLimits& limits
) {
    KinematicValidationResult result;
    result.is_valid = true;
    result.confidence = 1.0f;
    
    // Convertir ángulos esféricos a ángulos articulares del robot
    // pitch ≈ shoulder elevation (joint_6)
    // yaw ≈ shoulder azimuth (joint_5)
    // roll ≈ elbow/wrist rotation (joint_7)
    
    float shoulder_elev = angles.pitch;
    float shoulder_azim = angles.yaw;
    float elbow_rot = angles.roll;
    
    // Validar limites articulares
    if (shoulder_elev < limits.shoulder_elevation_min ||
        shoulder_elev > limits.shoulder_elevation_max) {
        result.is_valid = false;
        result.confidence = 0.0f;
        result.error_message = "Shoulder elevation out of range";
        return result;
    }
    
    if (shoulder_azim < limits.shoulder_azimuth_min ||
        shoulder_azim > limits.shoulder_azimuth_max) {
        result.is_valid = false;
        result.confidence = 0.0f;
        result.error_message = "Shoulder azimuth out of range";
        return result;
    }
    
    if (elbow_rot < limits.elbow_rotation_min ||
        elbow_rot > limits.elbow_rotation_max) {
        result.is_valid = false;
        result.confidence = 0.0f;
        result.error_message = "Elbow rotation out of range";
        return result;
    }
    
    return result;
}

/**
 * @brief Detecta singularidades cinemáticas (codo apuntando exactamente al hombro)
 * 
 * @param elbow Vector posición del codo
 * @param shoulder Vector posición del hombro
 * @param threshold Tolerancia en metros
 * @return true si está en singularidad
 */
inline bool detectSingularity(const Vector3& elbow, const Vector3& shoulder, float threshold = 0.01f) {
    Vector3 diff = elbow - shoulder;
    return diff.magnitude() < threshold;
}

/**
 * @brief Convierte Vector3 de geometry_msgs::Point32
 */
inline Vector3 point32ToVector3(float x, float y, float z) {
    return Vector3(x, y, z);
}

/**
 * @brief Convierte ángulos esféricos de radianes a grados
 */
inline SphericalAngles radiansToDegreesSpherical(const SphericalAngles& rad) {
    return SphericalAngles{
        rad.pitch * 180.0f / M_PI,
        rad.yaw * 180.0f / M_PI,
        rad.roll * 180.0f / M_PI
    };
}

/**
 * @brief Aplica factor de suavizado exponencial (bajo pass filter)
 */
inline float exponentialSmooth(float new_value, float prev_value, float alpha = 0.2f) {
    return alpha * new_value + (1.0f - alpha) * prev_value;
}

/**
 * @brief Aplica suavizado a ángulos esféricos
 */
inline SphericalAngles smoothSphericalAngles(
    const SphericalAngles& new_angles,
    const SphericalAngles& prev_angles,
    float alpha = 0.2f
) {
    return SphericalAngles{
        exponentialSmooth(new_angles.pitch, prev_angles.pitch, alpha),
        exponentialSmooth(new_angles.yaw, prev_angles.yaw, alpha),
        exponentialSmooth(new_angles.roll, prev_angles.roll, alpha)
    };
}

}  // namespace kinematic_validator

#endif  // KINEMATIC_VALIDATOR_HPP