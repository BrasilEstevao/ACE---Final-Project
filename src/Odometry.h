#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

/**
 * @class Odometry
 * @brief Sistema de odometria baseado em encoders
 * 
 * Calcula posição (x, y, theta) e distâncias relativas
 * usando leituras de encoders nas rodas.
 */
class Odometry {
public:
    /**
     * @brief Construtor
     * @param wheelDist Distância entre rodas (m) - padrão 0.125m
     * @param wheelRadius Raio da roda (m) - padrão 0.034m
     * @param pulsesPerRev Pulsos de encoder por revolução - padrão 20
     */
    Odometry(float wheelDist = 0.125, float wheelRadius = 0.034, int pulsesPerRev = 20);
    
    /**
     * @brief Atualiza odometria com contagens de encoder
     * @param enc1 Pulsos do encoder esquerdo desde última atualização
     * @param enc2 Pulsos do encoder direito desde última atualização
     * @param dt Intervalo de tempo desde última atualização (s)
     */
    void update(int enc1, int enc2, float dt);
    
    /**
     * @brief Reseta odometria relativa (para manobras)
     */
    void resetRelative();
    
    /**
     * @brief Reseta odometria global
     */
    void resetGlobal();
    
    /**
     * @brief Verifica se distância relativa foi atingida
     * @param targetDistance Distância alvo (m)
     * @return true se distância >= alvo
     */
    bool reachedDistance(float targetDistance);
    
    /**
     * @brief Verifica se rotação relativa foi atingida
     * @param targetAngle Ângulo alvo (radianos)
     * @return true se |ângulo| >= |alvo|
     */
    bool reachedRotation(float targetAngle);
    
    // Getters - Odometria Global
    float getX() const { return _x; }
    float getY() const { return _y; }
    float getTheta() const { return _theta; }
    
    // Getters - Odometria Relativa (para manobras)
    float getRelativeDistance() const { return _relDistance; }
    float getRelativeRotation() const { return _relRotation; }
    
    // Getters - Velocidades estimadas
    float getLinearVelocity() const { return _v; }
    float getAngularVelocity() const { return _w; }
    
    /**
     * @brief Imprime estado da odometria
     */
    void printStatus();
    
private:
    // Parâmetros físicos
    float _wheelDist;      // Distância entre rodas (m)
    float _wheelRadius;    // Raio da roda (m)
    int _pulsesPerRev;     // Pulsos por revolução
    
    // Pose global (x, y, theta)
    float _x;              // Posição X (m)
    float _y;              // Posição Y (m)
    float _theta;          // Orientação (rad)
    
    // Deslocamentos relativos (para manobras)
    float _relDistance;    // Distância percorrida desde reset (m)
    float _relRotation;    // Rotação desde reset (rad)
    
    // Velocidades estimadas
    float _v;              // Velocidade linear (m/s)
    float _w;              // Velocidade angular (rad/s)
    
    // Velocidades das rodas
    float _v1, _v2;        // Velocidades lineares das rodas (m/s)
};

#endif // ODOMETRY_H