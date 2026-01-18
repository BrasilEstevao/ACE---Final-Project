#include "RobotStateMachine.h"
#include "config.h"
#include <Arduino.h>

// ============================================================================
// CONSTRUCTOR
// ============================================================================

RobotStateMachine::RobotStateMachine(L298NMotor* left, L298NMotor* right, LineSensor* sensor, Odometry* odom)
{
    _leftMotor = left;
    _rightMotor = right;
    _lineSensor = sensor;
    _odometry = odom;
    
    _state = STATE_IDLE;
    _newState = STATE_IDLE;
    _currentMode = MODE_LINE_FOLLOW;
    
    _tes = millis();
    _tis = 0;
    
    _kp = KP;
    _ki = KI;
    _kd = KD;
    _lastError = 0;
    _integral = 0;
    
    _baseSpeed = BASE_SPEED;
    _maxSpeed = MAX_SPEED;
    
    _storedJunction = JUNCTION_NONE;
    
    _leftSpeedNew = 0;
    _rightSpeedNew = 0;
    
    // Odometry-based turn configuration
    _useOdometry = true;
    _turn90Angle = PI / 2.0f;
    _turn180Angle = PI;
    _smallForwardDist = 0.05f;
    
    // Velocity control
    _targetV = 0.0f;
    _targetW = 0.0f;
    _usePIDControl = true;  // NOVO: escolhe entre PID ou controle direto
    
    // Goto targets
    _targetDistance = 0.0f;
    _targetAngle = 0.0f;
    
    // PID para controle de velocidade (ganhos reduzidos)
    // Como agora as correÃ§Ãµes sÃ£o aplicadas em m/s (nÃ£o PWM), 
    // os ganhos precisam ser muito menores
    _pidV_kp = 50.0f;   // Proporcional para velocidade linear
    _pidV_ki = 10.0f;   // Integral (reduzido para evitar overshoot)
    _pidV_kd = 5.0f;    // Derivativo
    _pidV_lastError = 0.0f;
    _pidV_integral = 0.0f;
    
    _pidW_kp = 20.0f;   // Proporcional para velocidade angular (MUITO reduzido)
    _pidW_ki = 2.0f;    // Integral
    _pidW_kd = 1.0f;    // Derivativo
    _pidW_lastError = 0.0f;
    _pidW_integral = 0.0f;
}

// ============================================================================
// STATE MACHINE UPDATE - MAIN LOOP
// ============================================================================

void RobotStateMachine::update()
{
    updateTIS();
    
    _odometry->update(CONTROL_LOOP_MS / 1000.0f);
    
    _lineSensor->read();
    
    updateStateTransitions();
       
    updateOutputs();
    
    applyOutputs();
}

// ============================================================================
// TIMING MANAGEMENT (TIS/TES)
// ============================================================================

void RobotStateMachine::updateTIS()
{
    _tis = millis() - _tes;
}

void RobotStateMachine::changeState(RobotState newState)
{
    if (_state != newState) {
        _state = newState;
        _tes = millis();  
        _tis = 0;
        
        if (newState == STATE_TURN_LEFT || 
            newState == STATE_TURN_RIGHT || 
            newState == STATE_TURN_AROUND ||
            newState == STATE_SMALL_FORWARD ||
            newState == STATE_GOTO_DISTANCE ||
            newState == STATE_GOTO_ANGLE) {
            _odometry->resetRelative();
            
            // Reset PID controllers
            _pidV_integral = 0.0f;
            _pidV_lastError = 0.0f;
            _pidW_integral = 0.0f;
            _pidW_lastError = 0.0f;
        }
    }
}

unsigned long RobotStateMachine::getTIS()
{
    return _tis;
}

// ============================================================================
// STATE TRANSITIONS
// ============================================================================

void RobotStateMachine::updateStateTransitions()
{
    JunctionType junction = _lineSensor->detectJunction();
    
    _newState = _state;
    
    switch (_state) 
    {
        case STATE_IDLE:
            break;
            
        case STATE_LINE_FOLLOW:
            if (junction != JUNCTION_NONE && junction != JUNCTION_LOST) {
                if (_currentMode == MODE_MAZE_SOLVE) 
                {
                    switch (junction)
                    {
                        case JUNCTION_LEFT:
                            _newState = STATE_SMALL_FORWARD;
                            _storedJunction = JUNCTION_LEFT;
                            break;
                        case JUNCTION_RIGHT:
                            _newState = STATE_SMALL_FORWARD;
                            _storedJunction = JUNCTION_RIGHT;
                            break;
                        case JUNCTION_T:
                            _newState = STATE_SMALL_FORWARD;
                            _storedJunction = JUNCTION_T;
                            break;
                        case JUNCTION_LOST:
                            _newState = STATE_TURN_AROUND;
                            break;
                        default:
                            break;
                    }
                }
            }
            if (junction == JUNCTION_LOST) {
                _newState = STATE_TURN_AROUND;
            }
            break;
            
        case STATE_SMALL_FORWARD:
        {
            bool forwardComplete = false;
            
            if (_useOdometry) {
                forwardComplete = (_odometry->getRelativeDistance() >= _smallForwardDist);
            } else {
                forwardComplete = (_tis > SMALL_FWD_MS);
            }
            
            if (forwardComplete) {
                junction = _lineSensor->detectJunction();

                Serial.print("Stored Junction = ");
                switch (_storedJunction) {
                    case JUNCTION_LEFT:  Serial.println("LEFT"); break;
                    case JUNCTION_RIGHT: Serial.println("RIGHT"); break;
                    case JUNCTION_T:     Serial.println("T"); break;
                    case JUNCTION_NONE:  Serial.println("NONE"); break;
                    case JUNCTION_LOST:  Serial.println("LOST"); break;
                    default:             Serial.println("UNKNOWN"); break;
                }

                switch (junction)
                {
                    case JUNCTION_T:
                        _newState = STATE_FINISHED;
                        break;

                    case JUNCTION_NONE:
                        switch (_storedJunction)
                        {
                            case JUNCTION_LEFT:
                            case JUNCTION_T:
                                _newState = STATE_TURN_LEFT;
                                break;

                            case JUNCTION_RIGHT:
                                _newState = STATE_LINE_FOLLOW;
                                break;

                            default:
                                break;
                        }
                        break;

                    case JUNCTION_LOST:
                        switch (_storedJunction)
                        {
                            case JUNCTION_LEFT:
                            case JUNCTION_T:
                                _newState = STATE_TURN_LEFT;
                                break;

                            case JUNCTION_RIGHT:
                                _newState = STATE_TURN_RIGHT;
                                break;

                            default:
                                break;
                        }
                        break;

                    default:
                        break;
                }
            }
            break;
        }
            
        case STATE_TURN_LEFT:
        {
            bool leftTurnComplete = false;
            
            if (_useOdometry) {
                float turnedAngle = fabs(_odometry->getRelativeAngle());
                leftTurnComplete = (turnedAngle >= _turn90Angle * 0.9f) && (junction == JUNCTION_NONE);
                
                if (_tis > LEFT_TURN_90_TIME_MS * 2) {
                    Serial.println("[WARNING] Left turn timeout - falling back to line");
                    leftTurnComplete = true;
                }
            } else {
                leftTurnComplete = (_tis > LEFT_TURN_90_TIME_MS) && (junction == JUNCTION_NONE);
            }
            
            if (leftTurnComplete) {
                _newState = STATE_LINE_FOLLOW;
            }
            break;
        }
            
        case STATE_TURN_RIGHT:
        {
            bool rightTurnComplete = false;
            
            if (_useOdometry) {
                float turnedAngle = fabs(_odometry->getRelativeAngle());
                rightTurnComplete = (turnedAngle >= _turn90Angle * 0.9f) && (junction == JUNCTION_NONE);
                
                if (_tis > RIGHT_TURN_90_TIME_MS * 2) {
                    Serial.println("[WARNING] Right turn timeout - falling back to line");
                    rightTurnComplete = true;
                }
            } else {
                rightTurnComplete = (_tis > RIGHT_TURN_90_TIME_MS) && (junction == JUNCTION_NONE);
            }
            
            if (rightTurnComplete) {
                _newState = STATE_LINE_FOLLOW;
            }
            break;
        }
            
        case STATE_TURN_AROUND:
        {
            bool uTurnComplete = false;
            
            if (_useOdometry) {
                float turnedAngle = fabs(_odometry->getRelativeAngle());
                uTurnComplete = (turnedAngle >= _turn180Angle * 0.9f) && (junction == JUNCTION_NONE);
                
                if (_tis > TURN_180_TIME_MS * 2) {
                    Serial.println("[WARNING] U-turn timeout - falling back to line");
                    uTurnComplete = true;
                }
            } else {
                uTurnComplete = (_tis > TURN_180_TIME_MS) && (junction == JUNCTION_NONE);
            }
            
            if (uTurnComplete) {
                _newState = STATE_LINE_FOLLOW;
            }
            break;
        }
            
        case STATE_OBSTACLE_AVOID:
            if (_tis > 1000) {
                _newState = STATE_LINE_FOLLOW;
            }
            break;
            
        case STATE_MAZE_SOLVE:
            _newState = STATE_LINE_FOLLOW;
            break;
            
        case STATE_LOST:
            if (_tis > 500) {
                _newState = STATE_TURN_RIGHT;
            }
            break;
            
        case STATE_FINISHED:
            break;
            
        case STATE_VELOCITY_CONTROL:
            break;
            
        case STATE_GOTO_DISTANCE:
        {
            float traveled = _odometry->getRelativeDistance();
            float remaining = fabs(_targetDistance) - fabs(traveled);
            
            // TolerÃ¢ncia de 2cm (mais realista)
            if (remaining <= 0.02f) {
                Serial.print("[GOTO] Distance complete: ");
                Serial.print(traveled * 100.0f, 2);
                Serial.print(" cm (target: ");
                Serial.print(_targetDistance * 100.0f, 2);
                Serial.println(" cm)");
                _newState = STATE_IDLE;
            }
            
            // Timeout safety
            float expectedTime = fabs(_targetDistance) / 0.15f * 1000.0f;
            if (_tis > expectedTime * 4) {
                Serial.println("[GOTO] Distance timeout");
                _newState = STATE_IDLE;
            }
            break;
        }
        
        case STATE_GOTO_ANGLE:
        {
            float turned = _odometry->getRelativeAngle();
            float remaining = fabs(_targetAngle) - fabs(turned);
            
            // TolerÃ¢ncia de 3Â°
            if (remaining <= 0.052f) {
                Serial.print("[GOTO] Angle complete: ");
                Serial.print(turned * 180.0f / PI, 1);
                Serial.print("Â° (target: ");
                Serial.print(_targetAngle * 180.0f / PI, 1);
                Serial.println("Â°)");
                _newState = STATE_IDLE;
            }
            
            // Timeout safety
            float expectedTime = fabs(_targetAngle) / 0.8f * 1000.0f;
            if (_tis > expectedTime * 4) {
                Serial.println("[GOTO] Angle timeout");
                _newState = STATE_IDLE;
            }
            break;
        }
    }
    
    changeState(_newState);
}

// ============================================================================
// UPDATE OUTPUTS
// ============================================================================

void RobotStateMachine::updateOutputs()
{
    switch (_state)
    {
        case STATE_IDLE:
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
            
        case STATE_LINE_FOLLOW:
            calculatePIDSpeeds();
            break;
            
        case STATE_SMALL_FORWARD:
            _leftSpeedNew = _baseSpeed;
            _rightSpeedNew = _baseSpeed;
            break;
            
        case STATE_TURN_LEFT:
            _leftSpeedNew = -TURN_SPEED;
            _rightSpeedNew = TURN_SPEED;
            break;
            
        case STATE_TURN_RIGHT:
            _leftSpeedNew = TURN_SPEED;
            _rightSpeedNew = -TURN_SPEED;
            break;
            
        case STATE_TURN_AROUND:
            _leftSpeedNew = TURN_SPEED;
            _rightSpeedNew = -TURN_SPEED;
            break;
            
        case STATE_VELOCITY_CONTROL:
            if (_usePIDControl) {
                calculateVelocityControlPID();
            } else {
                calculateVelocityControl();
            }
            break;
            
        case STATE_GOTO_DISTANCE:
        {
            float traveled = _odometry->getRelativeDistance();
            float remaining = fabs(_targetDistance) - fabs(traveled);
            
            // Velocidade base reduzida
            float speed = 0.12f;  // m/s (reduzido de 0.15)
            
            // DesaceleraÃ§Ã£o nos Ãºltimos 15cm
            if (remaining < 0.15f) {
                speed = 0.05f + (remaining / 0.15f) * 0.07f;  // 5-12 cm/s
            }
            
            if (_targetDistance < 0) speed = -speed;
            
            _targetV = speed;
            _targetW = 0.0f;  // FORÃ‡A rotaÃ§Ã£o zero
            
            if (_usePIDControl) {
                calculateVelocityControlPID();
            } else {
                calculateVelocityControl();
            }
            break;
        }
        
        case STATE_GOTO_ANGLE:
        {
            float turned = _odometry->getRelativeAngle();
            float remaining = fabs(_targetAngle) - fabs(turned);
            
            // Velocidade angular reduzida
            float angSpeed = 0.6f;  // rad/s (reduzido de 0.8)
            
            // DesaceleraÃ§Ã£o nos Ãºltimos 30Â°
            if (remaining < 0.52f) {
                angSpeed = 0.25f + (remaining / 0.52f) * 0.35f;  // 0.25-0.6 rad/s
            }
            
            if (_targetAngle < 0) angSpeed = -angSpeed;
            
            _targetV = 0.0f;  // FORÃ‡A movimento linear zero
            _targetW = angSpeed;
            
            if (_usePIDControl) {
                calculateVelocityControlPID();
            } else {
                calculateVelocityControl();
            }
            break;
        }
            
        case STATE_OBSTACLE_AVOID:
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
            
        case STATE_MAZE_SOLVE:
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
            
        case STATE_LOST:
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
            
        case STATE_FINISHED:
            _leftSpeedNew = 0;
            _rightSpeedNew = 0;
            break;
    }
}

// ============================================================================
// APPLY OUTPUTS
// ============================================================================

void RobotStateMachine::applyOutputs()
{
    _leftMotor->setSpeed(_leftSpeedNew);
    _rightMotor->setSpeed(_rightSpeedNew);
}

// ============================================================================
// PID CALCULATION (Line Following)
// ============================================================================

void RobotStateMachine::calculatePIDSpeeds()
{
    int position = _lineSensor->getPosition();
    int error = position;
    
    _integral += error;
    _integral = constrain(_integral, -10000, 10000);
    
    int derivative = error - _lastError;
    _lastError = error;
    
    float correction = _kp * error + _ki * _integral + _kd * derivative;
    
    int leftSpeed = _baseSpeed - correction;
    int rightSpeed = _baseSpeed + correction;
    
    _leftSpeedNew = constrain(leftSpeed, -_maxSpeed, _maxSpeed);
    _rightSpeedNew = constrain(rightSpeed, -_maxSpeed, _maxSpeed);
}

// ============================================================================
// VELOCITY CONTROL - MODO DIRETO (SEM PID)
// ============================================================================

void RobotStateMachine::calculateVelocityControl()
{
    // Converte v (m/s) e w (rad/s) para velocidades das rodas
    float wheelbase = WHEEL_DISTANCE;
    float v_left = _targetV + (_targetW * wheelbase / 2.0f);
    float v_right = _targetV - (_targetW * wheelbase / 2.0f);
    
    // ConversÃ£o m/s -> PWM
    // CORRIGIDO: Fator de calibraÃ§Ã£o mais realista
    // Assumindo ~0.3 m/s = PWM 255 (baseado em testes tÃ­picos)
    float velocityToPWM = 255.0f / 0.3f;  // ~850
    
    int leftPWM = (int)(v_left * velocityToPWM);
    int rightPWM = (int)(v_right * velocityToPWM);
    
    _leftSpeedNew = constrain(leftPWM, -255, 255);
    _rightSpeedNew = constrain(rightPWM, -255, 255);
}

// ============================================================================
// VELOCITY CONTROL - COM PID (FECHADO)
// ============================================================================

void RobotStateMachine::calculateVelocityControlPID()
{
    // LÃª velocidades atuais
    float currentV = _odometry->getVelocity();
    float currentW = _odometry->getAngularVelocity();
    
    // Calcula erros
    float errorV = _targetV - currentV;
    float errorW = _targetW - currentW;
    
    // ========================================================================
    // PID PARA VELOCIDADE LINEAR (v)
    // ========================================================================
    _pidV_integral += errorV * (CONTROL_LOOP_MS / 1000.0f);
    _pidV_integral = constrain(_pidV_integral, -0.1f, 0.1f);  // Anti-windup reduzido
    
    float derivativeV = (errorV - _pidV_lastError) / (CONTROL_LOOP_MS / 1000.0f);
    _pidV_lastError = errorV;
    
    // SaÃ­da do PID em m/s (nÃ£o PWM ainda!)
    float correctionV = _pidV_kp * errorV + 
                        _pidV_ki * _pidV_integral + 
                        _pidV_kd * derivativeV;
    
    // ========================================================================
    // PID PARA VELOCIDADE ANGULAR (w)
    // ========================================================================
    _pidW_integral += errorW * (CONTROL_LOOP_MS / 1000.0f);
    _pidW_integral = constrain(_pidW_integral, -0.1f, 0.1f);  // Anti-windup reduzido
    
    float derivativeW = (errorW - _pidW_lastError) / (CONTROL_LOOP_MS / 1000.0f);
    _pidW_lastError = errorW;
    
    // SaÃ­da do PID em rad/s (nÃ£o PWM ainda!)
    float correctionW = _pidW_kp * errorW + 
                        _pidW_ki * _pidW_integral + 
                        _pidW_kd * derivativeW;
    
    // ========================================================================
    // CONVERSÃƒO PARA PWM DAS RODAS
    // ========================================================================
    
    // Aplica correÃ§Ãµes aos setpoints (ainda em m/s e rad/s)
    float v_corrected = _targetV + correctionV;
    float w_corrected = _targetW + correctionW;
    
    // Converte para velocidades lineares das rodas (m/s)
    float wheelbase = WHEEL_DISTANCE;
    float v_left = v_corrected + (w_corrected * wheelbase / 2.0f);
    float v_right = v_corrected - (w_corrected * wheelbase / 2.0f);
    
    // ConversÃ£o m/s -> PWM (calibrado)
    // Baseado em testes: ~0.3 m/s = PWM 255
    float velocityToPWM = 255.0f / 0.3f;
    
    int leftPWM = (int)(v_left * velocityToPWM);
    int rightPWM = (int)(v_right * velocityToPWM);
    
    _leftSpeedNew = constrain(leftPWM, -255, 255);
    _rightSpeedNew = constrain(rightPWM, -255, 255);
}

// ============================================================================
// CONTROL METHODS
// ============================================================================

void RobotStateMachine::start()
{
    changeState(STATE_LINE_FOLLOW);
    _lastError = 0;
    _integral = 0;
    _odometry->reset();
}

void RobotStateMachine::stop()
{
    changeState(STATE_IDLE);
}

void RobotStateMachine::setMode(RobotMode mode)
{
    _currentMode = mode;
    changeState(STATE_IDLE);
}

void RobotStateMachine::setSpeed(int speed)
{
    _baseSpeed = constrain(speed, 0, 255);
}

void RobotStateMachine::setPID(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void RobotStateMachine::setVelocityPID(float kp_v, float ki_v, float kd_v, float kp_w, float ki_w, float kd_w)
{
    _pidV_kp = kp_v;
    _pidV_ki = ki_v;
    _pidV_kd = kd_v;
    _pidW_kp = kp_w;
    _pidW_ki = ki_w;
    _pidW_kd = kd_w;
}

void RobotStateMachine::setPIDControlMode(bool usePID)
{
    _usePIDControl = usePID;
    
    if (usePID) {
        Serial.println("[CONTROL] Velocity PID enabled (closed-loop)");
    } else {
        Serial.println("[CONTROL] Direct velocity control (open-loop)");
    }
}

void RobotStateMachine::setTurnAngles(float turn90, float turn180)
{
    _turn90Angle = turn90;
    _turn180Angle = turn180;
}

void RobotStateMachine::setVelocity(float v, float w)
{
    _targetV = v;
    _targetW = w;
    
    if (_currentMode == MODE_VELOCITY_CONTROL && _state == STATE_IDLE) {
        changeState(STATE_VELOCITY_CONTROL);
    }
}

void RobotStateMachine::gotoDistance(float distance)
{
    _targetDistance = distance;
    _odometry->resetRelative();
    changeState(STATE_GOTO_DISTANCE);
    
    Serial.print("[GOTO] Moving ");
    Serial.print(distance * 100.0f, 2);
    Serial.println(" cm");
}

void RobotStateMachine::gotoAngle(float angle)
{
    _targetAngle = angle;
    _odometry->resetRelative();
    changeState(STATE_GOTO_ANGLE);
    
    Serial.print("[GOTO] Turning ");
    Serial.print(angle * 180.0f / PI, 1);
    Serial.println("Â°");
}

RobotState RobotStateMachine::getState()
{
    return _state;
}

RobotMode RobotStateMachine::getMode()
{
    return _currentMode;
}

bool RobotStateMachine::isPIDControlEnabled()
{
    return _usePIDControl;
}

void RobotStateMachine::printStatus()
{
    Serial.print("State: ");
    switch(_state) {
        case STATE_IDLE: Serial.print("IDLE"); break;
        case STATE_LINE_FOLLOW: Serial.print("LINE_FOLLOW"); break;
        case STATE_SMALL_FORWARD: Serial.print("SMALL_FWD"); break;
        case STATE_TURN_LEFT: Serial.print("TURN_LEFT"); break;
        case STATE_TURN_RIGHT: Serial.print("TURN_RIGHT"); break;
        case STATE_TURN_AROUND: Serial.print("TURN_AROUND"); break;
        case STATE_OBSTACLE_AVOID: Serial.print("OBSTACLE_AVOID"); break;
        case STATE_MAZE_SOLVE: Serial.print("MAZE_SOLVE"); break;
        case STATE_LOST: Serial.print("LOST"); break;
        case STATE_FINISHED: Serial.print("FINISHED"); break;
    }
    
    Serial.print(" | TIS: ");
    Serial.print(_tis);
    Serial.print("ms");
    
    Serial.print(" | Mode: ");
    switch(_currentMode) {
        case MODE_LINE_FOLLOW: Serial.print("LINE_FOLLOW"); break;
        case MODE_OBSTACLE_AVOID: Serial.print("OBSTACLE_AVOID"); break;
        case MODE_MAZE_SOLVE: Serial.print("MAZE_SOLVE"); break;
        case MODE_NAVIGATE: Serial.print("NAVIGATE"); break;
        case MODE_VELOCITY_CONTROL: Serial.print("VELOCITY_CONTROL"); break;
    }
    
    Serial.print(" | Control: ");
    Serial.print(_usePIDControl ? "PID" : "DIRECT");
    
    Serial.print(" | Speed: L=");
    Serial.print(_leftMotor->getSpeed());
    Serial.print(" R=");
    Serial.print(_rightMotor->getSpeed());
    
    Serial.print(" | Odom: Î¸=");
    Serial.print(_odometry->getRelativeAngle() * 180.0f / PI, 1);
    Serial.print("Â° d=");
    Serial.print(_odometry->getRelativeDistance(), 3);
    Serial.println("m");
}