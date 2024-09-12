#include <TMCStepper.h>
#include <AccelStepper.h>

// Pines y configuración para los motores
constexpr auto STEP_PIN_HOMBRO = 6;
constexpr auto DIR_PIN_HOMBRO = 3;
constexpr auto ENABLE_PIN_HOMBRO = 51;
constexpr auto R_SENSE = 0.11f;
constexpr auto DRIVER_ADDRESS = 0b00;
#define SERIAL_PORT Serial3

constexpr auto motorPin1 = 8;
constexpr auto motorPin2 = 9;
constexpr auto motorPin3 = 10;
constexpr auto motorPin4 = 11;
constexpr auto HALFSTEP = 8;

constexpr auto LEVA_HOMBRO_PIN = 5; // Pin del sensor de fin de carrera del hombro
constexpr auto LEVA_CODO_PIN = 4;   // Pin del sensor de fin de carrera del codo

// Factores de conversión de grados a pasos
constexpr auto PASOS_POR_REVOLUCION_HOMBRO = 15450.0;  // Pasos por revolución del hombro
constexpr auto PASOS_POR_REVOLUCION_CODO = 3964.0;     // Pasos por revolución del codo

// Variables globales para las velocidades
float VELOCIDAD_RAPIDA = 800;   // Velocidad rápida en pasos por segundo
float VELOCIDAD_LENTA = 400;     // Velocidad lenta para ajustes finos
float ACELERACION_HOMBRO = 800;  // Aceleración para el hombro
float ACELERACION_CODO = 800;    // Aceleración para el codo

// Decalajes específicos para el homing
constexpr auto DECALAJE_HOMBRO = 102;  // En grados
constexpr auto DECALAJE_CODO = -44;     // En grados

TMC2209Stepper driverHombro(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper hombro(AccelStepper::DRIVER, STEP_PIN_HOMBRO, DIR_PIN_HOMBRO);
AccelStepper codo(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);

// Estructura para almacenar las coordenadas en milímetros
struct Coordenadas {
    double x;
    double y;
};

// Variable para almacenar la posición actual del efector
Coordenadas posicionActual = { 0, 0 }; // Inicialmente en (0, 0)

// Función para convertir grados a pasos para el hombro
long convertirGradosAPasosHombro(float grados) {
    return (long)((-grados / 360.0) * PASOS_POR_REVOLUCION_HOMBRO);
}

// Función para convertir grados a pasos para el codo
//long convertirGradosAPasosCodo(float grados) {
//    return (long)((grados / 360.0) * PASOS_POR_REVOLUCION_CODO);
//}
// Función para convertir grados a pasos para el codo (eje Y invertido)
long convertirGradosAPasosCodo(float grados) {
    return (long)((-grados / 360.0) * PASOS_POR_REVOLUCION_CODO); // Invertir el eje Y
}


// Función que convierte coordenadas de ajedrez a coordenadas en milímetros
Coordenadas convertirCoordenadasAjedrezAMm(String casilla) {
    Coordenadas coord;

    // Verificar que la casilla tenga exactamente 2 caracteres (una letra y un número)
    if (casilla.length() != 2) {
        Serial.println("Error: Casilla inválida");
        coord.x = 0;
        coord.y = 0;
        return coord;
    }

    // Obtener la columna (letra) y fila (número)
    char columna = casilla.charAt(0);  // Letra de la columna ('a' - 'h')
    char fila = casilla.charAt(1);     // Número de la fila ('1' - '8')

    // Ajustar la columna a las coordenadas X (40 mm por casilla)
    coord.x = (columna - 'd') * 40 - 20;

    // Ajustar la fila a las coordenadas Y (40 mm por casilla)
    coord.y = (fila - '4') * 40 - 20;

    return coord;
}

// Función para mover el brazo SCARA a las coordenadas X, Y
void moverBrazoSCARA(double x, double y) {
    // Mapear las coordenadas X e Y a pasos de los motores
    long pasosHombro = convertirGradosAPasosHombro(x);
    long pasosCodo = convertirGradosAPasosCodo(y);

    // Mover el hombro
    hombro.moveTo(pasosHombro);
    while (hombro.distanceToGo() != 0) {
        hombro.run();
    }

    // Mover el codo
    codo.moveTo(pasosCodo);
    while (codo.distanceToGo() != 0) {
        codo.run();
    }

    Serial.println("Movimiento completado.");
}

// Función para mover al origen y luego al destino
void moverBrazoDeOrigenADestino(Coordenadas origen, Coordenadas destino) {
    Serial.println("Moviendo al origen...");
    moverBrazoSCARA(origen.x, origen.y);  // Mover al origen
    Serial.println("Moviendo al destino...");
    moverBrazoSCARA(destino.x, destino.y);  // Mover al destino
}

// Función para recibir órdenes de movimiento
void recibirOrdenesMovimiento() {
    if (Serial.available() >= 6) {
        String comando = Serial.readStringUntil('\n');  // Leer el comando completo

        if (comando.length() == 6) {
            char pieza = comando.charAt(0);      // Letra de la pieza
            String origen = comando.substring(1, 3);  // Casilla de origen
            char captura = comando.charAt(3);    // Carácter de captura (' ' o 'x')
            String destino = comando.substring(4, 6);  // Casilla de destino

            // Convertir las casillas de origen y destino a coordenadas en mm
            Coordenadas coordOrigen = convertirCoordenadasAjedrezAMm(origen);
            Coordenadas coordDestino = convertirCoordenadasAjedrezAMm(destino);

            // Mover el brazo SCARA de la posición actual al origen y luego al destino
            moverBrazoDeOrigenADestino(coordOrigen, coordDestino);

            // Actualizar la posición actual del efector
            posicionActual = coordDestino;
        }
        else {
            Serial.println("Comando inválido.");
        }
    }
}

// Función para realizar el homing del motor del hombro con rotación CW o CCW según el estado inicial
void realizarHomingHombro() {
    Serial.println("Iniciando homing del motor del hombro...");
    pinMode(LEVA_HOMBRO_PIN, INPUT_PULLUP);
    hombro.setMaxSpeed(800);
    hombro.setAcceleration(500);

    // Verificar el estado inicial del sensor (prominente o rebajado)
    bool estadoInicialProminente = (digitalRead(LEVA_HOMBRO_PIN) == LOW);

    if (estadoInicialProminente) {
        // La leva está en la parte prominente: rotar en sentido CW (horario)
        Serial.println("Leva en posicion prominente. Moviendo en sentido CW...");
        while (digitalRead(LEVA_HOMBRO_PIN) == LOW) {
            hombro.setSpeed(800);  // Mover en sentido CW
            hombro.runSpeed();
        }
    }
    else {
        // La leva está en la parte rebajada: rotar en sentido CCW (antihorario)
        Serial.println("Leva en posición rebajada. Moviendo en sentido CCW...");
        while (digitalRead(LEVA_HOMBRO_PIN) == HIGH) {
            hombro.setSpeed(-800);  // Mover en sentido CCW
            hombro.runSpeed();
        }
    }

    // Mover hasta que el sensor vuelva a activarse (parte prominente)
    while (digitalRead(LEVA_HOMBRO_PIN) == HIGH) {
        hombro.setSpeed(-VELOCIDAD_LENTA);  // Movimiento fino con velocidad lenta
        hombro.runSpeed();
    }


    // Ajustar el decalaje después del homing
    long pasosDecalajeHombro = convertirGradosAPasosHombro(DECALAJE_HOMBRO);
    hombro.moveTo(hombro.currentPosition() + pasosDecalajeHombro);
    while (hombro.distanceToGo() != 0) {
        hombro.run();
    }

    hombro.setCurrentPosition(0);  // Establecer la posición corregida como 0
    Serial.println("Homing y ajuste del motor del hombro completados.");
}

// Función para realizar el homing del motor del codo con rotación CW o CCW según el estado inicial
void realizarHomingCodo() {
    Serial.println("Iniciando homing del motor del codo...");
    pinMode(LEVA_CODO_PIN, INPUT_PULLUP);
    codo.setMaxSpeed(VELOCIDAD_RAPIDA);
    codo.setAcceleration(ACELERACION_CODO);

    // Verificar el estado inicial del sensor (prominente o rebajado)
    bool estadoInicialProminente = (digitalRead(LEVA_CODO_PIN) == LOW);

    if (estadoInicialProminente) {
        // La leva está en la parte prominente: rotar en sentido CW
        Serial.println("Leva en posición prominente. Moviendo en sentido CW...");
        while (digitalRead(LEVA_CODO_PIN) == LOW) {
            codo.setSpeed(VELOCIDAD_RAPIDA);  // Mover en sentido CW
            codo.runSpeed();
        }
    }
    else {
        // La leva está en la parte rebajada: rotar en sentido CCW
        Serial.println("Leva en posición rebajada. Moviendo en sentido CCW...");
        while (digitalRead(LEVA_CODO_PIN) == HIGH) {
            codo.setSpeed(-VELOCIDAD_RAPIDA);  // Mover en sentido CCW
            codo.runSpeed();
        }
    }

    // Mover hasta que el sensor vuelva a activarse (parte prominente)
    while (digitalRead(LEVA_CODO_PIN) == HIGH) {
        codo.setSpeed(-VELOCIDAD_LENTA);  // Movimiento fino con velocidad lenta
        codo.runSpeed();
    }

    // Ajustar el decalaje después del homing
    long pasosDecalajeCodo = convertirGradosAPasosCodo(DECALAJE_CODO);
    codo.moveTo(codo.currentPosition() + pasosDecalajeCodo);
    while (codo.distanceToGo() != 0) {
        codo.run();
    }

    codo.setCurrentPosition(0);  // Establecer la posición corregida como 0
    Serial.println("Homing y ajuste del motor del codo completados.");
}

void realizarHoming() {
    realizarHomingHombro();
    realizarHomingCodo();
    Serial.println("Esperando órdenes de movimiento...");

}

void setup() {
    Serial.begin(9600);
    SERIAL_PORT.begin(115200);

    // Configuración del driver y los motores
    driverHombro.begin();
    driverHombro.rms_current(800);   // Corriente RMS a 800 mA
    driverHombro.microsteps(16);     // Microstepping (16 microsteps)
    driverHombro.en_spreadCycle(false);
    driverHombro.pwm_autoscale(true);

    pinMode(ENABLE_PIN_HOMBRO, OUTPUT);
    digitalWrite(ENABLE_PIN_HOMBRO, LOW);  // Habilitar el driver

    // Configuración de velocidades y aceleraciones
    hombro.setMaxSpeed(VELOCIDAD_RAPIDA);
    hombro.setAcceleration(ACELERACION_HOMBRO);
    codo.setMaxSpeed(VELOCIDAD_RAPIDA);
    codo.setAcceleration(ACELERACION_CODO);

    Serial.println("Sistema inicializado. Realizando homing...");

    // Llamar al proceso de homing
    realizarHoming();
}

void loop() {
    // Escuchar órdenes de movimiento
    recibirOrdenesMovimiento();
}
